# v3: figure re-sliced from an SDF volume after reclining + lengthening the upper body
import numpy as np, warnings; warnings.filterwarnings("ignore")
from shapely.geometry import Polygon, MultiPolygon, box, Point, LineString
from shapely.ops import unary_union
from scipy.ndimage import map_coordinates
import contourpy
from functools import lru_cache
P=0.025; T=0.008
V=np.load('vol.npz'); D=V['D']; M=V['M']; ZS=V['ZS']; RES,X0,X1,Y0,Y1=V['grid']
def geoms(g):
    if g is None or g.is_empty: return []
    return [x for x in getattr(g,'geoms',[g]) if x.geom_type=='Polygon' and not x.is_empty]
def U(lst): return unary_union([x for x in lst if x is not None and not x.is_empty]) if lst else Polygon()
def close(g,r): return g.buffer(r,join_style=1).buffer(-r,join_style=1)
def opn(g,r): return g.buffer(-r,join_style=1).buffer(r,join_style=1)
# ---------------- figure deformation ----------------
DY=0.12; SX=1.05
PHI=np.radians(18); STRETCH=1.25; YH=0.50; ZH=-0.25     # upper body recline + lengthening about the hip
ARM_TOP=0.80; ARM_Z=-0.38
THICK=0.005
def fy(y): return np.where(y<=0.10,y,np.where(y>=0.50,y-DY,0.10+(y-0.10)*(0.40-DY)/0.40))
def fy_inv(y): return np.where(y<=0.10,y,np.where(y>=0.38,y+DY,0.10+(y-0.10)*0.40/(0.40-DY)))
c,s=np.cos(PHI),np.sin(PHI)
HEAD_Y=1.075; HB=(HEAD_Y-YH)*STRETCH
def up_fwd(y,z):
    yb=np.where(y<=HEAD_Y,(y-YH)*STRETCH,HB+(y-HEAD_Y)); zb=z-ZH
    return (YH-DY)+zb*s+yb*c, ZH+zb*c-yb*s
def up_inv(y,z):
    dz=z-ZH; dy=y-(YH-DY)
    zb=dz*c+dy*s; ybp=(-dz*s+dy*c)
    yb=np.where(ybp<=HB,ybp/STRETCH,(HEAD_Y-YH)+(ybp-HB))
    return YH+yb, ZH+zb
ARM_Y1,ARM_Z1=up_fwd(ARM_TOP,ARM_Z)
ARM_DZ=ARM_Z1-ARM_Z; ARM_SY=ARM_Y1/ARM_TOP
def sample(F,x,y,z):
    # mostly nearest-slice (keeps thin rings whole), short smooth blend at the mid-plane between slices
    k=(z-ZS[0])/P
    i=(x-X0)/RES; j=(y-Y0)/RES
    k0=np.clip(np.floor(k),0,len(ZS)-1); k1=np.clip(k0+1,0,len(ZS)-1); t=np.clip(k-k0,0,1)
    t=np.clip((t-0.35)/0.30,0,1); t=t*t*(3-2*t)
    v0=map_coordinates(F,[k0,j,i],order=1,mode='nearest'); v1=map_coordinates(F,[k1,j,i],order=1,mode='nearest')
    v=(1-t)*v0+t*v1
    out=(k<-0.4)|(k>len(ZS)-0.6)|(i<0)|(i>F.shape[2]-1)|(j<0)|(j>F.shape[1]-1)
    v[out]=0.5
    return v
GX=np.arange(-0.42,0.42+1e-9,0.002); GY=np.arange(-0.01,1.42,0.002)
XX,YY=np.meshgrid(GX,GY)
def field(F,zn):
    x=XX.ravel(); y=YY.ravel(); best=np.full(x.shape,0.5,np.float32)
    # lower body
    xo=x/SX; yo=fy_inv(y); zo=np.full_like(x,zn)
    ok=(yo<YH)|(zo>-0.19)
    v=sample(F,xo,yo,zo); best=np.where(ok,np.minimum(best,v),best)
    # upper body (torso, shoulders, head)
    yo,zo=up_inv(y,np.full_like(x,zn)); xo=x/SX
    ok=(yo>=YH)&(zo<=-0.19)&((np.abs(xo)<0.245)|(yo>=ARM_TOP))
    v=sample(F,xo,yo,zo); best=np.where(ok,np.minimum(best,v),best)
    # arms (rear legs): follow the shoulder back, rescaled to reach the floor
    yo=y/ARM_SY; zo=np.full_like(x,zn)-ARM_DZ; xo=x/SX
    ok=(np.abs(xo)>=0.245)&(yo<ARM_TOP)&(zo<=-0.19)
    v=sample(F,xo,yo,zo); best=np.where(ok,np.minimum(best,v),best)
    return best.reshape(XX.shape)
def to_poly(fld,level):
    cg=contourpy.contour_generator(GX,GY,fld,fill_type=contourpy.FillType.OuterOffset)
    polys=[]
    pts,offs=cg.filled(-10,level)
    for p,o in zip(pts,offs):
        rings=[p[o[k]:o[k+1]] for k in range(len(o)-1)]
        if len(rings[0])<4: continue
        polys.append(Polygon(rings[0],[r for r in rings[1:] if len(r)>=4]).buffer(0))
    return U(polys)
@lru_cache(maxsize=None)
def reslice(zn):
    zn=round(float(zn),4)
    det=to_poly(field(D,zn),THICK); mas=to_poly(field(M,zn),0.0)
    det=close(det,0.003).simplify(0.0006); mas=mas.simplify(0.0006)
    return det,mas
# ---------------- sitter envelope (bucket) ----------------
SEAT_Y=0.44; TILT=np.radians(4); RECLINE=np.radians(18)
ZF=0.17; DEPTH=0.385; ZBS=ZF-DEPTH
KS=0.30; KB=1.6; KH=4.0
def ys0(z):
    R=0.04; y=SEAT_Y-(ZF-min(z,ZF))*np.tan(TILT)
    if z>ZF-R: d=min(z-(ZF-R),R); y-=R-np.sqrt(R*R-d*d)
    if z>ZF: y-=(z-ZF)*0.5
    if z>0.30: y-=(z-0.30)*6
    return y
YSR=ys0(ZBS)
SH_Y=YSR+0.60
def zb0(y):
    z=ZBS-(y-YSR)*np.tan(RECLINE)
    z+=0.020*np.exp(-((y-(YSR+0.20))/0.07)**2)
    z-=0.035*np.exp(-((y-(YSR+0.03))/0.05)**2)
    return z
def w(y):
    if y<SH_Y-0.04: return 0.25
    if y>SH_Y+0.08: return 0.115
    t=(y-(SH_Y-0.04))/0.12; return 0.25+(0.115-0.25)*(0.5-0.5*np.cos(np.pi*t))
def kb(y): return KB if y<SH_Y else KB+(KH-KB)*min(1,(y-SH_Y)/0.1)
@lru_cache(maxsize=None)
def envelope(z,dy=0.003):
    z=round(float(z),4); rects=[]; ysz=ys0(z)
    for y in np.arange(0.08,1.7,dy):
        if y<ysz: continue
        xm=w(y); a=(y-ysz)/KS; xm=min(xm,np.sqrt(a)) if a>=0 else 0
        zz=z-zb0(y)
        if zz<=0: continue
        xm=min(xm,np.sqrt(zz/kb(y)))
        if xm>0.002: rects.append(box(-xm,y,xm,y+dy))
    if not rects: return Polygon()
    return opn(U(rects).buffer(0.0015),0.01)
