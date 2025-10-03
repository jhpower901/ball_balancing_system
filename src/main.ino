/*
  ESP32-S3 + OV2640/OV5640
  - OV2640: GRAYSCALE 직접 수신
  - OV5640: YUV422로 수신 후 Y만 추출
  - 초기화: 고정 반지름 라이트 허프 (한 번)
  - 추적: 라디얼 샘플링 + 최소자승 원 피팅 (매 프레임)
*/

#include <Arduino.h>
#include "esp_camera.h"
#include <math.h>
#include <string.h>

// ====== 카메라 설정 (보드 핀맵은 반드시 맞춰주세요) ======
static camera_config_t camcfg = {
  .pin_pwdn = -1,
  .pin_reset = -1,
  .pin_xclk = 15,
  .pin_sccb_sda = 4,
  .pin_sccb_scl = 5,

  .pin_d7 = 16,
  .pin_d6 = 17, 
  .pin_d5 = 18, 
  .pin_d4 = 12,
  .pin_d3 = 10,
  .pin_d2 = 8,
  .pin_d1 = 9,
  .pin_d0 = 11,

  .pin_vsync = 6,
  .pin_href  = 7,
  .pin_pclk  = 13,

  .xclk_freq_hz = 20000000,    // 20MHz 권장

  .ledc_timer   = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,
  
  .pixel_format = PIXFORMAT_GRAYSCALE, // OV2640에선 GRAYSCALE 권장
  .frame_size   = FRAMESIZE_QQVGA,     // 160x120부터 시작

  .jpeg_quality = 10,
  .fb_count     = 2,

  .grab_mode    = CAMERA_GRAB_LATEST   // 지연 최소화
};

// ====== 파라미터 ======
static const int   W = 160, H = 120;     // camcfg.frame_size와 일치해야 함
static const int   R_FIXED = 22;         // 거의 고정된 반지름(px) — 실제 영상으로 맞추세요
static const int   EDGE_THR = 28;        // Sobel 임계값(환경에 따라 20~60 조정)
static const int   K_DIR = 32;           // 라디얼 샘플링 방향 개수 (24~48 권장)
static const int   RAD_WIN = 4;          // r 주변 탐색폭 (±RAD_WIN)

// ====== 버퍼 ======
static uint8_t  Y[W*H];      // 그레이스케일 이미지
static uint8_t  edges[W*H];  // 0/255
static uint16_t acc[W*H];    // 허프 누산 (초기화용)

// ====== 라디얼 샘플링 LUT (Q15) ======
static int16_t cosLUT[64], sinLUT[64]; // 64방향용 (K_DIR<=64)
static inline void buildTrigLUT(){
  for(int i=0;i<64;i++){
    float a = (2.0f*M_PI*i)/64.0f;
    cosLUT[i] = (int16_t)lrintf(cosf(a)*32767.0f);
    sinLUT[i] = (int16_t)lrintf(sinf(a)*32767.0f);
  }
}
static inline int lutIdxFromAngleIndex(int i){ return i & 63; }

// ====== 간단 박스블러 (노이즈 심할 때만 켜세요) ======
static inline void boxBlur3x3(uint8_t* img){
  for(int y=1;y<H-1;y++){
    for(int x=1;x<W-1;x++){
      int s=0;
      for(int dy=-1;dy<=1;dy++)
        for(int dx=-1;dx<=1;dx++)
          s += img[(y+dy)*W + (x+dx)];
      Y[y*W+x] = (uint8_t)(s/9);
    }
  }
}

// ====== Sobel 에지 (|Gx|+|Gy| 근사) ======
static inline void sobelEdge(const uint8_t* g, uint8_t* out, int thr){
  memset(out, 0, W*H);
  for(int y=1;y<H-1;y++){
    for(int x=1;x<W-1;x++){
      int p00=g[(y-1)*W + (x-1)], p01=g[(y-1)*W + x],     p02=g[(y-1)*W + (x+1)];
      int p10=g[y*W     + (x-1)], p11=g[y*W     + x],     p12=g[y*W     + (x+1)];
      int p20=g[(y+1)*W + (x-1)], p21=g[(y+1)*W + x],     p22=g[(y+1)*W + (x+1)];
      int gx = (p02 + 2*p12 + p22) - (p00 + 2*p10 + p20);
      int gy = (p20 + 2*p21 + p22) - (p00 + 2*p01 + p02);
      int mag = abs(gx) + abs(gy);
      out[y*W+x] = (mag>thr)?255:0;
    }
  }
}

// ====== 초기화용: 고정 반지름 라이트 허프(정수 오프셋) ======
struct Off { int8_t dx, dy; };
static Off circleOffsets[600]; // 2πR ≈ 140, 여유롭게
static int nOffsets=0;
void buildCircleOffsets(int r){
  nOffsets = 0;
  auto push8 = [&](int a, int b){
    if (nOffsets + 8 > (int)(sizeof(circleOffsets)/sizeof(circleOffsets[0]))) return;
      circleOffsets[nOffsets++]={(int8_t)+a,(int8_t)+b};
      circleOffsets[nOffsets++]={(int8_t)+b,(int8_t)+a};
      circleOffsets[nOffsets++]={(int8_t)-a,(int8_t)+b};
      circleOffsets[nOffsets++]={(int8_t)-b,(int8_t)+a};
      circleOffsets[nOffsets++]={(int8_t)+a,(int8_t)-b};
      circleOffsets[nOffsets++]={(int8_t)+b,(int8_t)-a};
      circleOffsets[nOffsets++]={(int8_t)-a,(int8_t)-b};
      circleOffsets[nOffsets++]={(int8_t)-b,(int8_t)-a};
  };
  int x=0, y=r, d=1-r;
  while (x<=y){
    push8(x,y);
    x++;
    if (d < 0) d += (x<<1) + 1;
    else { y--; d += ((x - y) << 1) + 1; }
  }
}

static void houghFixedR(const uint8_t* edge, uint16_t* acc, int r, int& cx, int& cy){
  memset(acc, 0, sizeof(acc[0])*W*H);
  for(int y=r; y<H-r; y++){
    for(int x=r; x<W-r; x++){
      if(edge[y*W+x]==0) continue;
      for(int k=0;k<nOffsets;k++){
        int xx = x - circleOffsets[k].dx;
        int yy = y - circleOffsets[k].dy;
        acc[yy*W+xx]++; // 경계는 r 마진으로 안전
      }
    }
  }
  // 최대값 + 3x3 가중평균 보정
  uint16_t best=0; int bx=0, by=0;
  for(int y=r; y<H-r; y++){
    for(int x=r; x<W-r; x++){
      uint16_t v=acc[y*W+x];
      if(v>best){best=v; bx=x; by=y;}
    }
  }
  int sw=0,sx=0,sy=0;
  for(int dy=-1;dy<=1;dy++)
    for(int dx=-1;dx<=1;dx++){
      int xx=bx+dx, yy=by+dy;
      uint16_t w = acc[yy*W+xx];
      sw+=w; sx+=w*xx; sy+=w*yy;
    }
  if(sw>0){ cx=sx/sw; cy=sy/sw; } else { cx=bx; cy=by; }
}

// ====== 최소자승 원 피팅(Pratt 간단형) ======
static bool fitCircle_Pratt(const float* xs, const float* ys, int n, float& cx, float& cy, float& r){
  if(n<3) return false;
  double sumx=0,sumy=0,sumx2=0,sumy2=0,sumxy=0,sumz=0,sumxz=0,sumyz=0;
  for(int i=0;i<n;i++){
    double x=xs[i], y=ys[i];
    double x2=x*x, y2=y*y, z=x2+y2;
    sumx+=x; sumy+=y; sumx2+=x2; sumy2+=y2; sumxy+=x*y;
    sumz+=z; sumxz+=x*z; sumyz+=y*z;
  }
  double A = 2*(sumx2 + sumy2) - 2*( (sumx*sumx) + (sumy*sumy) )/n;
  double Bx= 2*(sumxz - (sumx*sumz)/n);
  double By= 2*(sumyz - (sumy*sumz)/n);
  double Cx= 2*(sumx2 + sumy2 - (sumz*sumx)/n);
  // 간소화 버전: 정규방정식 근사(작은 샘플에서 충분)
  // 보다 정확한 Pratt/Taubin은 추가 항이 있지만, 실전으론 이걸로도 안정적
  if (fabs(A) < 1e-9) return false;
  cx = (float)(Bx / A);
  cy = (float)(By / A);
  r  = 0.0f;
  // 평균 반지름
  for(int i=0;i<n;i++){
    float dx = xs[i]-cx, dy = ys[i]-cy;
    r += sqrtf(dx*dx+dy*dy);
  }
  r /= n;
  return true;
}

// ====== 라디얼 샘플링 트래커 ======
static void radialTrackerStep(int& cx, int& cy, int r, int kDir, int win){
  // 각도 등분
  const int maxK = (kDir>64)?64:kDir;
  float xs[64], ys[64];
  int n=0;

  for(int i=0;i<maxK;i++){
    int li = lutIdxFromAngleIndex( (i*64)/maxK );
    // 라디얼 방향 단위 벡터 (Q15 → float)
    float ux = (float)cosLUT[li]/32767.0f;
    float uy = (float)sinLUT[li]/32767.0f;

    // 중심에서 해당 방향으로 r±win 범위 1D 탐색 → 에지 기울기 최대 지점
    int best_t = 0;
    int best_grad = -1;
    for(int dt = -win; dt <= win; dt++){
      float rx = cx + (r+dt)*ux;
      float ry = cy + (r+dt)*uy;
      int x = (int)lrintf(rx);
      int y = (int)lrintf(ry);
      if ((unsigned)x >= (unsigned)W || (unsigned)y >= (unsigned)H) continue;

      // 1D 주변 기울기 근사: 중심에서 바깥쪽으로 한 픽셀 차분
      int x2 = x + (ux>=0?1:-1);
      int y2 = y + (uy>=0?1:-1);
      if ((unsigned)x2 >= (unsigned)W || (unsigned)y2 >= (unsigned)H) continue;
      int g = abs((int)Y[y*W+x] - (int)Y[y2*W+x2]);
      if (g > best_grad){ best_grad=g; best_t= r+dt; }
    }

    // 베스트 지점 좌표 저장
    float px = cx + best_t*ux;
    float py = cy + best_t*uy;
    // 이미지 경계 체크
    int ix=(int)lrintf(px), iy=(int)lrintf(py);
    if ((unsigned)ix < (unsigned)W && (unsigned)iy < (unsigned)H){
      xs[n]=px; ys[n]=py; n++;
    }
  }

  if (n >= 6){
    float fcx, fcy, fr;
    if (fitCircle_Pratt(xs, ys, n, fcx, fcy, fr)){
      // EMA로 약간 평활화(튀는 프레임 억제)
      const float alpha = 0.6f;
      float ncx = alpha*fcx + (1-alpha)*cx;
      float ncy = alpha*fcy + (1-alpha)*cy;
      cx = (int)lrintf(ncx);
      cy = (int)lrintf(ncy);
      // r은 거의 고정이므로 유지(필요시 fr도 EMA)
    }
  }
}

// ====== 센서 모드 보정(AE/AGC/AWB 고정 추천) ======
static void lock_sensor_exposure(){
  sensor_t *s = esp_camera_sensor_get();
  if(!s) return;
  // 조명 일정하면 자동 기능을 고정하는 게 임계값 안정에 유리
  s->set_gain_ctrl(s, 0);     // AGC off
  s->set_exposure_ctrl(s, 0); // AEC off
  s->set_awb_gain(s, 0);      // AWB off
  // 필요 시 수동 노출/게인 설정 (보드/환경에 맞춰 조정)
  // s->set_aec_value(s, 300);
  // s->set_agc_gain(s, 8);
}

// ====== OV5640 YUV에서 Y만 추출 ======
static inline void y_from_yuv422(const uint8_t* src, int sw, int sh, uint8_t* dst){
  // YUYV YUYV ... (짝수 byte가 Y) 가정
  const int N = sw*sh;
  for(int i=0;i<N;i++){
    dst[i] = src[i*2];
  }
}

// ====== 전역 상태 ======
static bool tracker_ready = false;
static int  cX= W/2, cY= H/2;

// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  delay(100);

  // 카메라 초기화
  if (esp_camera_init(&camcfg) != ESP_OK) {
    Serial.println("Camera init failed");
    while(1) delay(1000);
  }

  // 센서 PID에 따라 픽셀 포맷/플랜 처리 분기
  sensor_t* s = esp_camera_sensor_get();
  uint16_t pid = s ? s->id.PID : 0;
  Serial.printf("Sensor PID: 0x%04X\n", pid);

  if (pid == OV5640_PID){
    // OV5640은 GRAYSCALE 직접 출력이 제한적 → YUV422로 받고 Y만 추출
    camcfg.pixel_format = PIXFORMAT_YUV422;
    esp_camera_deinit();
    if (esp_camera_init(&camcfg) != ESP_OK) {
      Serial.println("Reinit(YUV422) failed");
      while(1) delay(1000);
    }
  } else {
    // OV2640은 GRAYSCALE로 진행
  }

  // LUT / 오프셋 빌드
  buildTrigLUT();
  buildCircleOffsets(R_FIXED);

  // 자동 노출 등 잠금(환경 따라 조정)
  lock_sensor_exposure();

  Serial.println("Ready.");
}

// ====== LOOP ======
void loop() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;

  // Y 버퍼 획득
  if (fb->format == PIXFORMAT_GRAYSCALE && fb->len == W*H){
    memcpy(Y, fb->buf, W*H);
  } else if (fb->format == PIXFORMAT_YUV422){
    // OV5640: Y만 추출
    y_from_yuv422(fb->buf, W, H, Y);
  } else {
    // 예상치 못한 포맷/크기
    esp_camera_fb_return(fb);
    return;
  }
  esp_camera_fb_return(fb);

  // (옵션) 블러
  // boxBlur3x3(Y);

  if (!tracker_ready){
    // 초기화: 에지 + 고정R 허프로 중심 1회 찾기
    sobelEdge(Y, edges, EDGE_THR);
    int cx, cy;
    houghFixedR(edges, acc, R_FIXED, cx, cy);
    // 간단 유효성
    if (cx>R_FIXED && cy>R_FIXED && cx<W-R_FIXED && cy<H-R_FIXED){
      cX = cx; cY = cy;
      tracker_ready = true;
    }
  } else {
    // 추적: 라디얼 샘플링 + 최소자승 보정
    radialTrackerStep(cX, cY, R_FIXED, K_DIR, RAD_WIN);
  }

  // 출력 (좌표)
  Serial.printf("C,(%d,%d),R,%d\n", cX, cY, R_FIXED);

  // 타이밍 여유 조정(필요시 제거)
  // delay(1);
}
