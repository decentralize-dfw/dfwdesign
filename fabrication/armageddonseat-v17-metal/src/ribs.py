import numpy as np, pickle
from shapely.geometry import Polygon, MultiPolygon, box, Point, LineString
import warnings; warnings.filterwarnings("ignore")
from shapely.ops import unary_union, transform
from shapely import affinity
P=0.025
S=pickle.load(open('sections.pkl','rb'))
def geoms(g):
    if g is None or g.is_empty: return []
    return list(getattr(g,'geoms',[g]))
def fill(u,r=0.018):
    g=u.buffer(r,join_style=1).buffer(-r,join_style=1)
    return unary_union([Polygon(p.exterior) for p in geoms(g) if p.area>2e-5])
def hull_sides(u):
    out=[]
    for side in (box(-1,-1,0,2),box(0,-1,1,2)):
        h=u.intersection(side)
        if not h.is_empty: out.append(h.convex_hull)
    return unary_union(out)
# ---- parameters ----
DY=0.12           # vertical drop of upper body
K_LOW=3           # lower body shifted forward (pitches)
M_HEAD=5          # head shifted back (pitches)
SX_LOW=1.12; SX_UP=1.08
SEAT_FRONT_Y=0.445; SEAT_TILT=np.radians(5); BACK_RECLINE=np.radians(18)
def fy(y):
    y=np.asarray(y,float)
    return np.where(y<=0.10,y,np.where(y>=0.50,y-DY,0.10+(y-0.10)*(0.40-DY)/0.40))
def tr(g,sx):
    g=g.segmentize(0.004) if hasattr(g,'segmentize') else g
    return transform(lambda x,y,z=None:(np.asarray(x)*sx,fy(y)),g)
zo=sorted(S.keys())
raw={}  # new z -> list of (polygon, tag)
def add(z,g,tag):
    z=round(z,4)
    for p in geoms(g): raw.setdefault(z,[]).append((p,tag))
HEAD_Y=1.075
for z in zo:
    u=S[z]
    if u is None or u.is_empty: continue
    f=fill(u)
    if z<=-0.175+1e-6:
        head=u.intersection(box(-1,HEAD_Y,1,2)).convex_hull; body=f.difference(box(-1,HEAD_Y,1,2))
        if z>=-0.25:  # hip discs (thigh roots) -> hulls
            hip=u.intersection(box(-1,0,1,0.6)); body=body.difference(box(-1,0,1,0.6)).union(hull_sides(hip.intersection(box(-0.25,0,0.25,0.6))) if not hip.is_empty else Polygon())
        add(z-M_HEAD*P,tr(head,1.0),'head')
        add(z,tr(body,SX_UP),'body')
    else:
        if z<=0.15: f=hull_sides(u)
        else: f=fill(u).union(hull_sides(u.intersection(box(-1,0.47,1,1))) if z<=0.25 else Polygon())
        add(z+K_LOW*P,tr(f,SX_LOW),'low')
# filler lap ribs between
f11=hull_sides(S[-0.15])
for k in range(K_LOW):
    add(-0.15+k*P,tr(f11,SX_LOW),'low')
Z=sorted(raw)
# ---- seat & back envelope ----
ZF=0.255                     # seat front (start of nose)
def ys_base(z): return SEAT_FRONT_Y-(ZF-z)*np.tan(SEAT_TILT)
def ys(z):
    R=0.04; y=ys_base(min(z,ZF))
    if z>ZF-R:
        d=min(z-(ZF-R),R); y-= R-np.sqrt(R*R-d*d)
    if z>ZF: y-= (z-ZF)*0.5
    if z>0.36: y-= (z-0.36)*6.0
    return y
ZBS=ZF-0.42                     # back line at seat level
YSR=ys(ZBS)
def zb(y):
    z=ZBS-(y-YSR)*np.tan(BACK_RECLINE)
    z+=0.022*np.exp(-((y-(YSR+0.21))/0.07)**2)           # lumbar bulge
    z-=0.045*np.exp(-((y-(YSR+0.03))/0.06)**2)            # sacral relief
    return z
SH_Y=YSR+0.57
def w(y): return 0.25 if y<SH_Y else 0.115
def envelope(z):
    rects=[]
    ys_=ys(z)
    for y in np.arange(0.0,1.6,0.004):
        if y+0.004<ys_ or y<0.08: continue
        if z>zb(y): rects.append(box(-w(y),max(y,ys_),w(y),y+0.004))
    if not rects: return Polygon()
    e=unary_union(rects).buffer(0.0025)
    return e.buffer(-0.03,join_style=1).buffer(0.03,join_style=1).union(e.intersection(box(-1,0.08,1,ys_+0.03)))
pickle.dump(dict(raw=raw,Z=Z,ZF=ZF,ZBS=ZBS,YSR=YSR),open('raw.pkl','wb'))
if __name__=='__main__':
    print(len(Z),Z[0],Z[-1],'ZF',ZF,'ZBS',ZBS,'YSR',YSR, 'seat front y',ys(ZF-0.06))
