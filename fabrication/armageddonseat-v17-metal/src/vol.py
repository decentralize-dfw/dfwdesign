# Build signed-distance volumes of the original figure from its 35 slices (original coordinates).
import numpy as np, pickle, warnings; warnings.filterwarnings("ignore")
from PIL import Image, ImageDraw
from scipy.ndimage import distance_transform_edt as edt
from shapely.geometry import box, Polygon
from shapely.ops import unary_union
S=pickle.load(open('sections.pkl','rb'))
RES=0.002; X0,X1,Y0,Y1=-0.42,0.42,-0.02,1.36
NX=int(round((X1-X0)/RES)); NY=int(round((Y1-Y0)/RES))
ZS=sorted(S)
def geoms(g):
    if g is None or g.is_empty: return []
    return [x for x in getattr(g,'geoms',[g]) if x.geom_type=='Polygon']
def U(l): return unary_union([x for x in l if x is not None and not x.is_empty]) if l else Polygon()
def close(g,r): return g.buffer(r,join_style=1).buffer(-r,join_style=1)
def fill(g): return U([Polygon(p.exterior) for p in geoms(g)])
def raster(g):
    im=Image.new('1',(NX,NY),0); dr=ImageDraw.Draw(im)
    def pts(c): return [((x-X0)/RES,(y-Y0)/RES) for x,y in c]
    for p in geoms(g):
        dr.polygon(pts(p.exterior.coords),fill=1)
        for h in p.interiors: dr.polygon(pts(h.coords),fill=0)
    return np.array(im,dtype=bool)   # [row=y, col=x]
def sdf(mask):
    if not mask.any(): return np.full(mask.shape,0.5,np.float32)
    return ((edt(~mask)-edt(mask))*RES).astype(np.float32)
def mass_polys(zo,u):
    out=[]
    if zo<=-0.175+1e-6:
        head=u.intersection(box(-1,1.075,1,2))
        if not head.is_empty: out.append(fill(close(head.buffer(0.004),0.02)))
        body=u.difference(box(-1,1.075,1,2))
        cage=body.intersection(box(-0.24,0.52,0.24,1.0))
        big=[c for c in geoms(close(cage.buffer(0.003),0.02)) if (c.bounds[2]-c.bounds[0])>0.12 and abs(c.centroid.x)<0.12]
        if big and zo>=-0.40: out.append(fill(U(big)))
        if zo>=-0.25:
            hip=body.intersection(box(-0.25,0.4,0.25,0.6))
            for s in (box(-1,0,0,2),box(0,0,1,2)):
                hh=hip.intersection(s)
                if hh.area>2e-4: out.append(hh.convex_hull.buffer(0.004))
    else:
        if zo<=0.15:
            for s in (box(-1,0,0,2),box(0,0,1,2)):
                hh=u.intersection(s).intersection(box(-1,0.3,1,1))
                if not hh.is_empty: out.append(hh.convex_hull.buffer(0.006))
        elif zo<=0.25:
            kn=u.intersection(box(-1,0.46,1,1))
            for s in (box(-1,0,0,2),box(0,0,1,2)):
                hh=kn.intersection(s)
                if hh.area>1e-4: out.append(hh.convex_hull.buffer(0.004))
    return U(out)
D=np.zeros((len(ZS),NY,NX),np.float32); M=np.zeros_like(D)
for k,zo in enumerate(ZS):
    u=S[zo]
    D[k]=sdf(raster(u)) if u is not None and not u.is_empty else 0.5
    M[k]=sdf(raster(mass_polys(zo,u))) if u is not None and not u.is_empty else 0.5
np.savez_compressed('vol.npz',D=D,M=M,ZS=np.array(ZS),grid=np.array([RES,X0,X1,Y0,Y1]))
print(D.shape, D.min(), M.min())
