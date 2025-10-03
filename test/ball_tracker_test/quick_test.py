# quick_test.py
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

W,H = 160,120
R = 22
THR = 28

def downscale_nearest(img, W, H):
    return img.resize((W,H), Image.NEAREST)

def sobel_edges(gray, thr):
    g = np.asarray(gray, dtype=np.uint8)
    g = g.astype(np.int16)
    Ht, Wd = g.shape
    edges = np.zeros_like(g, dtype=np.uint8)
    for y in range(1,Ht-1):
        for x in range(1,Wd-1):
            p00=g[y-1,x-1]; p01=g[y-1,x]; p02=g[y-1,x+1]
            p10=g[y,  x-1]; p11=g[y,  x]; p12=g[y,  x+1]
            p20=g[y+1,x-1]; p21=g[y+1,x]; p22=g[y+1,x+1]
            gx = (p02+2*p12+p22) - (p00+2*p10+p20)
            gy = (p20+2*p21+p22) - (p00+2*p01+p02)
            mag = abs(gx)+abs(gy)
            edges[y,x] = 255 if mag>thr else 0
    return edges

def build_offsets(r):
    pts=[]
    x=0; y=r; d=1-r
    def push8(a,b):
        pts.extend([( a, b),( b, a),(-a, b),(-b, a),( a,-b),( b,-a),(-a,-b),(-b,-a)])
    while x<=y:
        push8(x,y)
        x+=1
        if d<0: d += (x<<1)+1
        else:
            y-=1
            d += ((x-y)<<1)+1
    return np.array(pts, dtype=np.int32)

def hough_fixed_r(edges, r):
    Ht,Wd = edges.shape
    acc = np.zeros((Ht,Wd), dtype=np.uint16)
    off = build_offsets(r)
    ys, xs = np.nonzero(edges)
    for y,x in zip(ys,xs):
        if y<r or y>=Ht-r or x<r or x>=Wd-r: continue
        cx = x - off[:,0]; cy = y - off[:,1]
        m = (cx>=0)&(cx<Wd)&(cy>=0)&(cy<Ht)
        acc[cy[m], cx[m]] += 1
    by,bx = np.unravel_index(np.argmax(acc), acc.shape)
    # 3x3 보정
    y0=max(1,by-1); y1=min(Ht-1,by+2)
    x0=max(1,bx-1); x1=min(Wd-1,bx+2)
    win = acc[y0:y1, x0:x1]
    yy,xx = np.mgrid[y0:y1, x0:x1]
    w = win.sum()
    if w>0:
        cy = int((win*yy).sum()/w); cx = int((win*xx).sum()/w)
    else:
        cy, cx = by, bx
    return cx, cy, acc

if __name__ == "__main__":
    img = Image.open("sample.jpg").convert("L")
    small = downscale_nearest(img, W,H)
    edges = sobel_edges(small, THR)
    cx, cy, acc = hough_fixed_r(edges, R)
    print("center:", cx, cy)

    fig,ax=plt.subplots(1,2, figsize=(10,4))
    ax[0].imshow(small, cmap="gray"); ax[0].add_patch(plt.Circle((cx,cy),R, fill=False, lw=2)); ax[0].plot(cx,cy,'rx'); ax[0].set_title("gray")
    ax[1].imshow(edges, cmap="gray"); ax[1].add_patch(plt.Circle((cx,cy),R, fill=False, lw=2)); ax[1].plot(cx,cy,'rx'); ax[1].set_title("edges")
    for a in ax: a.set_axis_off()
    plt.tight_layout(); plt.show()
