# v2: original slice outlines preserved (offset-thickened), curvy bucket envelope, no seat lengthening
import numpy as np, pickle, warnings; warnings.filterwarnings("ignore")
from shapely.geometry import Polygon, MultiPolygon, box, Point, LineString
from shapely.ops import unary_union
import shapely
P=0.025; T=0.008
S=pickle.load(open('sections.pkl','rb'))
def geoms(g):
    if g is None or g.is_empty: return []
    return [x for x in getattr(g,'geoms',[g]) if x.geom_type=='Polygon' and not x.is_empty]
def U(lst): return unary_union([x for x in lst if x is not None and not x.is_empty]) if lst else Polygon()
def close(g,r): return g.buffer(r,join_style=1).buffer(-r,join_style=1)
def opn(g,r): return g.buffer(-r,join_style=1).buffer(r,join_style=1)
def fill(g,maxhole=None):
    out=[]
    for p in geoms(g):
        keep=[h for h in p.interiors if maxhole is not None and Polygon(h).area>maxhole]
        out.append(Polygon(p.exterior,keep))
    return U(out)
# ---------------- parameters ----------------
DY=0.12; M_HEAD=5; SX_LOW=1.06; SX_UP=1.0
THICK=0.005            # offset each side -> min member ~12 mm
SEAT_Y=0.44; TILT=np.radians(4); RECLINE=np.radians(15)
ZF=0.17; DEPTH=0.385; ZBS=ZF-DEPTH
KS=0.30; KB=1.4; KH=4.0
def fy(y):
    y=np.asarray(y,float)
    return np.where(y<=0.10,y,np.where(y>=0.50,y-DY,0.10+(y-0.10)*(0.40-DY)/0.40))
def tr(g,sx):
    if g.is_empty: return g
    g=g.segmentize(0.003)
    return shapely.transform(g,lambda c: np.c_[c[:,0]*sx,fy(c[:,1])])
def thick(u): return close(u.buffer(THICK,join_style=1),0.003)
HEAD_Y=1.075
# ---------------- envelope (bucket) ----------------
def ys0(z):
    R=0.04; y=SEAT_Y-(ZF-min(z,ZF))*np.tan(TILT)
    if z>ZF-R: d=min(z-(ZF-R),R); y-=R-np.sqrt(R*R-d*d)
    if z>ZF: y-=(z-ZF)*0.5
    if z>0.30: y-=(z-0.30)*6
    return y
YSR=ys0(ZBS)
SH_Y=YSR+0.56
def zb0(y):
    z=ZBS-(y-YSR)*np.tan(RECLINE)
    z+=0.020*np.exp(-((y-(YSR+0.20))/0.07)**2)
    z-=0.040*np.exp(-((y-(YSR+0.03))/0.05)**2)
    return z
def w(y):
    if y<SH_Y-0.04: return 0.25
    if y>SH_Y+0.08: return 0.115
    t=(y-(SH_Y-0.04))/0.12; return 0.25+(0.115-0.25)*(0.5-0.5*np.cos(np.pi*t))
def kb(y): return KB if y<SH_Y else KB+(KH-KB)*min(1,(y-SH_Y)/0.1)
from functools import lru_cache
@lru_cache(maxsize=None)
def envelope(z,dy=0.003):
    z=round(float(z),4)
    return _envelope(z,dy)
def _envelope(z,dy):
    rects=[]; ysz=ys0(z)
    for y in np.arange(0.08,1.6,dy):
        if y<ysz: continue
        xm=w(y)
        a=(y-ysz)/KS; xm=min(xm,np.sqrt(a)) if a>=0 else 0
        zz=z-zb0(y)
        if zz<=0: continue
        xm=min(xm,np.sqrt(zz/kb(y)))
        if xm>0.002: rects.append(box(-xm,y,xm,y+dy))
    if not rects: return Polygon()
    e=U(rects).buffer(0.0015)
    return opn(e,0.01)
