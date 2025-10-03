// circle_tracker.c : PC테스트용 경량 코어
// 빌드: cl /O2 /LD circle_tracker.c /Fe:circle_tracker.dll
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

typedef struct { int dx, dy; } Off;

static Off circleOffsets[8192];
static int nOffsets = 0;

EXPORT void build_circle_offsets(int r){
    nOffsets = 0;
    int x=0, y=r, d=1-r;
    // 여유체크 생략(8192 충분 가정). 필요시 안전 체크 추가
    #define PUSH8(a,b) \
      circleOffsets[nOffsets++] = (Off){+(a),+(b)}; \
      circleOffsets[nOffsets++] = (Off){+(b),+(a)}; \
      circleOffsets[nOffsets++] = (Off){-(a),+(b)}; \
      circleOffsets[nOffsets++] = (Off){-(b),+(a)}; \
      circleOffsets[nOffsets++] = (Off){+(a),-(b)}; \
      circleOffsets[nOffsets++] = (Off){+(b),-(a)}; \
      circleOffsets[nOffsets++] = (Off){-(a),-(b)}; \
      circleOffsets[nOffsets++] = (Off){-(b),-(a)};
    while(x<=y){
        PUSH8(x,y);
        x++;
        if (d<0) d += (x<<1)+1;
        else { y--; d += ((x-y)<<1)+1; }
    }
    #undef PUSH8
}

// 간단 nearest downscale (원본 그레이 8bit 가정)
EXPORT void downscale_gray_nearest(
    const uint8_t* src, int sw, int sh,
    uint8_t* dst, int dw, int dh)
{
    for(int y=0;y<dh;y++){
        int sy = (int)((y*(double)sh)/dh);
        if (sy>=sh) sy=sh-1;
        for(int x=0;x<dw;x++){
            int sx = (int)((x*(double)sw)/dw);
            if (sx>=sw) sx=sw-1;
            dst[y*dw + x] = src[sy*sw + sx];
        }
    }
}

// 3x3 박스블러(옵션)
EXPORT void box_blur3x3(uint8_t* img, int W, int H){
    uint8_t* tmp = (uint8_t*)malloc(W*H);
    if(!tmp) return;
    memcpy(tmp, img, W*H);
    for(int y=1;y<H-1;y++){
        for(int x=1;x<W-1;x++){
            int s=0;
            for(int dy=-1;dy<=1;dy++)
            for(int dx=-1;dx<=1;dx++)
                s += tmp[(y+dy)*W + (x+dx)];
            img[y*W+x] = (uint8_t)(s/9);
        }
    }
    free(tmp);
}

// Sobel → edges(0/255)
EXPORT void sobel_edges(const uint8_t* g, int W, int H, int thr, uint8_t* edges){
    memset(edges, 0, W*H);
    for(int y=1;y<H-1;y++){
        for(int x=1;x<W-1;x++){
            int p00=g[(y-1)*W + (x-1)], p01=g[(y-1)*W + x],     p02=g[(y-1)*W + (x+1)];
            int p10=g[y*W     + (x-1)], p11=g[y*W     + x],     p12=g[y*W     + (x+1)];
            int p20=g[(y+1)*W + (x-1)], p21=g[(y+1)*W + x],     p22=g[(y+1)*W + (x+1)];
            int gx = (p02 + 2*p12 + p22) - (p00 + 2*p10 + p20);
            int gy = (p20 + 2*p21 + p22) - (p00 + 2*p01 + p02);
            int mag = (gx<0?-gx:gx) + (gy<0?-gy:gy);
            edges[y*W+x] = (mag>thr)?255:0;
        }
    }
}

// 고정 반지름 허프(간단형) → (cx, cy) 반환
EXPORT void hough_fixed_r_center(
    const uint8_t* edges, int W, int H, int r,
    int* out_cx, int* out_cy)
{
    // 누산기 uint16
    uint16_t* acc = (uint16_t*)calloc(W*H, sizeof(uint16_t));
    if(!acc){ *out_cx=W/2; *out_cy=H/2; return; }

    for(int y=r;y<H-r;y++){
        for(int x=r;x<W-r;x++){
            if(edges[y*W+x]==0) continue;
            for(int k=0;k<nOffsets;k++){
                int xx = x - circleOffsets[k].dx;
                int yy = y - circleOffsets[k].dy;
                acc[yy*W + xx]++;
            }
        }
    }
    // 최대값 + 3x3 보정
    uint16_t best=0; int bx=r, by=r;
    for(int y=r;y<H-r;y++){
        for(int x=r;x<W-r;x++){
            uint16_t v = acc[y*W+x];
            if(v>best){ best=v; bx=x; by=y; }
        }
    }
    int sw=0,sx=0,sy=0;
    for(int dy=-1;dy<=1;dy++)
    for(int dx=-1;dx<=1;dx++){
        int xx=bx+dx, yy=by+dy;
        uint16_t w = acc[yy*W+xx];
        sw+=w; sx+=w*xx; sy+=w*yy;
    }
    if (sw>0){ *out_cx = sx/sw; *out_cy = sy/sw; }
    else { *out_cx = bx; *out_cy = by; }

    free(acc);
}
