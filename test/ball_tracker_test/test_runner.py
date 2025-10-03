import ctypes, os
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

# --- 설정 ---
DLL_PATH = os.path.join(os.path.dirname(__file__), "circle_tracker.dll")
IMG_PATH = os.path.join(os.path.dirname(__file__), "sample.jpg")
W, H = 160, 120     # ESP 실험 해상도 매칭
R_FIXED = 10        # 픽셀 반지름 (이미지에 맞춰 조정)
EDGE_THR = 60       # Sobel 임계값

# --- DLL 로드 ---
lib = ctypes.CDLL(DLL_PATH)
lib.build_circle_offsets.restype = None
lib.downscale_gray_nearest.restype = None
lib.box_blur3x3.restype = None
lib.sobel_edges.restype = None
lib.hough_fixed_r_center.restype = None

lib.build_circle_offsets.argtypes = [ctypes.c_int]
lib.downscale_gray_nearest.argtypes = [
    ctypes.POINTER(ctypes.c_uint8), ctypes.c_int, ctypes.c_int,
    ctypes.POINTER(ctypes.c_uint8), ctypes.c_int, ctypes.c_int
]
lib.box_blur3x3.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_int, ctypes.c_int]
lib.sobel_edges.argtypes = [
    ctypes.POINTER(ctypes.c_uint8), ctypes.c_int, ctypes.c_int, ctypes.c_int,
    ctypes.POINTER(ctypes.c_uint8)
]
lib.hough_fixed_r_center.argtypes = [
    ctypes.POINTER(ctypes.c_uint8), ctypes.c_int, ctypes.c_int, ctypes.c_int,
    ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)
]

# --- 이미지 로드(그레이) & 리사이즈 ---
img = Image.open(IMG_PATH).convert("L")        # grayscale
sw, sh = img.size
src = np.array(img, dtype=np.uint8).reshape(sh, sw)

dst = np.zeros((H, W), dtype=np.uint8)
# downscale (nearest) in C
lib.downscale_gray_nearest(
    src.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), sw, sh,
    dst.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), W, H
)


# (옵션) 블러
lib.box_blur3x3(dst.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), W, H)

# 에지
edges = np.zeros_like(dst, dtype=np.uint8)
lib.sobel_edges(
    dst.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), W, H, EDGE_THR,
    edges.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
)

# 허프 준비 (반지름 오프셋 한번만)
lib.build_circle_offsets(R_FIXED)

# 중심 찾기
cx = ctypes.c_int(0)
cy = ctypes.c_int(0)
lib.hough_fixed_r_center(
    edges.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
    W, H, R_FIXED,
    ctypes.byref(cx), ctypes.byref(cy)
)
print(f"center: ({cx.value}, {cy.value}), r={R_FIXED}")

# --- 결과 Plot ---
fig, ax = plt.subplots(1,2, figsize=(10,4))
ax[0].set_title("Downscaled Gray")
ax[0].imshow(dst, cmap="gray", vmin=0, vmax=255)
circ = plt.Circle((cx.value, cy.value), R_FIXED, fill=False, linewidth=2)
ax[0].add_patch(circ)
ax[0].plot(cx.value, cy.value, 'rx')

ax[1].set_title("Edges")
ax[1].imshow(edges, cmap="gray", vmin=0, vmax=255)
circ2 = plt.Circle((cx.value, cy.value), R_FIXED, fill=False, linewidth=2)
ax[1].add_patch(circ2)
ax[1].plot(cx.value, cy.value, 'rx')

for a in ax: a.set_axis_off()
plt.tight_layout()
plt.show()
