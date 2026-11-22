import numpy as np, json, sys
import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
from pose import Pose, J
def side(p,ax,title):
    zz=np.arange(-1.0,0.9,0.006); yy=np.arange(-0.02,1.5,0.006); Zg,Yg=np.meshgrid(zz,yy)
    best=np.full(Zg.shape,1.0)
    for x in (0.0,0.06,0.12,0.18,0.24,0.30,0.34):
        v=p.value(np.full(Zg.size,x),Yg.ravel(),Zg.ravel()).reshape(Zg.shape); best=np.minimum(best,v)
    m=best<p.thick
    ax.contourf(zz,yy,m.astype(float),levels=[0.5,1.5],colors=['#4a5a6a'])
    ax.axhline(0,color='k',lw=.8); ax.set_aspect('equal'); ax.set_title(title); ax.grid(alpha=.3)
    ys=Yg[m]; zs=Zg[m]
    return dict(miny=float(ys.min()),maxy=float(ys.max()),minz=float(zs.min()),maxz=float(zs.max()))
def hand_floor(p,el=None):
    pts=[(-0.33,0.0),(-0.28,0.0),(-0.36,0.02)]
    return min(p.point(4,z,y)[1] for z,y in pts)
def solve_arm(kw,lo=-60,hi=60):
    best=None
    for a in np.arange(lo,hi,0.25):
        p=Pose(**dict(kw,sh=a)); h=hand_floor(p)
        if best is None or abs(h)<abs(best[1]): best=(a,h)
    return best
