import pickle,numpy as np,warnings;warnings.filterwarnings("ignore")
from shapely.geometry import Point,box,Polygon
from ribs import geoms
d=pickle.load(open('ribs_final.pkl','rb'));ribs=d['ribs'];R=d['R'];Z=d['Z'];T=d['T'];K=d['KEEL'];keel=d['keel']
P=0.025;issues=[]
NUT={12:(0.0105,0.024),10:(0.0090,0.020),8:(0.0075,0.016)}   # acorn nut+washer: radius, height incl washer
for r in R:
    zs=[z for z in Z if r['z0']-1e-6<=z<=r['z1']+1e-6]
    for end,nb in ((min(zs),round(min(zs)-P,4)),(max(zs),round(max(zs)+P,4))):
        if nb in ribs and not ribs[nb][0].is_empty:
            rad,h=NUT[r['d']]
            hit=ribs[nb][0].intersects(Point(r['x'],r['y']).buffer(rad+0.002))
            if hit: issues.append(('NUT->RIB',r['name'],end,nb))
        if NUT[r['d']][1]>P-T and nb in ribs and ribs[nb][0].intersects(Point(r['x'],r['y']).buffer(NUT[r['d']][0]+0.002)):
            pass
# thin features
for z in Z:
    g=ribs[z][0]
    if g.is_empty: continue
    op=g.buffer(-0.004,join_style=1).buffer(0.004,join_style=1)
    lost=g.difference(op.buffer(0.0005))
    big=[c for c in geoms(lost) if c.area>2e-6]
    if big: issues.append(('THIN<8mm',z,len(big),round(sum(c.area for c in big)*1e6,1)))
# hole ligaments
for z in Z:
    g=ribs[z][0]
    for n in ribs[z][1]:
        r=[q for q in R if q['name']==n][0]
        solid=Polygon(max(geoms(g),key=lambda p:p.area).exterior) if False else None
        ring=Point(r['x'],r['y']).buffer(r['d']/2000+0.0004+0.004)
        filled=g.union(Point(r['x'],r['y']).buffer(r['d']/2000+0.0005))
        if not ring.difference(filled).is_empty and ring.difference(filled).area>1e-7:
            issues.append(('LIGAMENT<4mm',n,z,round(ring.difference(filled).area*1e6,2)))
# keel clashes with ribs outside its slot zone
kz0,kz1=keel.bounds[0],keel.bounds[2]
for z in Z:
    if K['z0']-1e-6<=z<=K['z1']+1e-6: continue
    if kz0-T/2<z<kz1+T/2:
        if ribs[z][0].intersects(box(-0.005,K['y0'],0.005,K['y1'])): issues.append(('KEEL->RIB',z))
print(len(issues)); [print(i) for i in issues]
