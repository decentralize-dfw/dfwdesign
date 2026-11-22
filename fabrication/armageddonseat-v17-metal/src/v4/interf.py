# exact interference check between orthogonal plates (z-plates vs x-plates): overlapping material on the shared strip
import pickle,sys,numpy as np
from shapely.geometry import box
from wf import geoms, T
v=sys.argv[1]; d=pickle.load(open(f'out/{v}/plates.pkl','rb')); P=d['plates']
Zp=[p for p in P if p['plane']=='z']; Xp=[p for p in P if p['plane']=='x']
def yints(poly,u0,u1):
    g=poly.intersection(box(u0,-1,u1,3)); out=[]
    for c in geoms(g): out.append((c.bounds[1],c.bounds[3],c))
    return g
bad=[]
for zp in Zp:
    for xp in Xp:
        # strip of the z-plate where the x-plate passes (x in xp.pos±T/2), and strip of the x-plate where the z-plate passes
        a=zp['poly'].intersection(box(xp['pos']-T/2,-1,xp['pos']+T/2,3))
        b=xp['poly'].intersection(box(zp['pos']-T/2,-1,zp['pos']+T/2,3))
        if a.is_empty or b.is_empty: continue
        # project both onto y and intersect
        from shapely.ops import unary_union
        ya=unary_union([box(0,c.bounds[1],1,c.bounds[3]) for c in geoms(a) if c.area>1e-7])
        yb=unary_union([box(0,c.bounds[1],1,c.bounds[3]) for c in geoms(b) if c.area>1e-7])
        ov=ya.intersection(yb)
        if not ov.is_empty and ov.area>0.0008: bad.append((zp['name'],xp['name'],round(ov.area*1000,1)))
print(v,'interferences (mm of overlap):',bad[:20],'count',len(bad))
# parallel plates closer than one thickness must not overlap in their plane
par=[]
for i,a in enumerate(P):
    for b in P[i+1:]:
        if a['plane']==b['plane'] and abs(a['pos']-b['pos'])<T+0.0005:
            ar=a['poly'].intersection(b['poly']).area
            if ar>1e-6: par.append((a['name'],b['name'],round(ar*1e6)))
print(v,'parallel overlaps (mm2):',par)
