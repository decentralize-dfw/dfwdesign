# Variant C "Izgara": waffle of frontal + sagittal plates sliced from the solid figure (1:1 proportions).
# Sagittal plates stand (slots from the top), frontal plates drop onto them (slots from the bottom).
# Only the topmost interval of a crossing line is jointed; frontal plates get through-slots for lower intervals.
from wf import *
fig=Figure(s=1.0,phi=13,kx=1.08,mode='filled')
env=Envelope(seat_y=0.545,zf=0.15,depth=0.40,recline=15)
PZ=0.05; PX=0.05
ZP=[float(z) for z in np.round(np.arange(-0.525,0.43,PZ),4)]
XP=[float(x) for x in np.round(np.arange(-0.35,0.351,PX),4)]
h=(T+0.0006)/2; CH=0.0015
def carve(plane,pos,ur):
    g=slice_fig(fig,plane,pos,ur); e=slice_env(env,plane,pos,ur)
    g=close(g,0.006).difference(e)
    return U([c for c in geoms(opn(g,0.004)) if c.area>4e-4]),e
ZG={};XG={};ZE={};XE={}
for z in ZP:
    g,e=carve('z',z,(-0.42,0.42))
    if not g.is_empty: ZG[z]=g; ZE[z]=e
for x in XP:
    g,e=carve('x',x,(-0.6,0.46))
    if not g.is_empty: XG[x]=g; XE[x]=e
def intervals(g,u):
    L=g.intersection(LineString([(u,-0.1),(u,1.6)]))
    out=[]
    for s in getattr(L,'geoms',[L]):
        if s.is_empty or s.geom_type!='LineString': continue
        ys=[c[1] for c in s.coords]; out.append((min(ys),max(ys)))
    return sorted(out)
def overlap(A,B):
    out=[]
    for a0,a1 in A:
        for b0,b1 in B:
            lo,hi=max(a0,b0),min(a1,b1)
            if hi-lo>0.03: out.append((lo,hi,max(a1,b1)))
    return out
zcut={z:[] for z in ZG}; xcut={x:[] for x in XG}; zkeep={z:[] for z in ZG}; xkeep={x:[] for x in XG}; joints_raw=[]
for z in ZG:
    for x in XG:
        ov=overlap(intervals(ZG[z],x),intervals(XG[x],z))
        if not ov: continue
        ov.sort()
        for k,(lo,hi,top) in enumerate(ov):
            m=0.5*(lo+hi)
            if k==len(ov)-1:   # topmost: true half-lap
                zcut[z].append(Polygon([(x-h-CH,lo-0.03),(x-h-CH,lo-0.003),(x-h,lo-0.003+CH),(x-h,m),(x+h,m),(x+h,lo-0.003+CH),(x+h+CH,lo-0.003),(x+h+CH,lo-0.03)]))
                xcut[x].append(Polygon([(z-h-CH,top+0.03),(z-h-CH,top+0.003),(z-h,top+0.003-CH),(z-h,m),(z+h,m),(z+h,top+0.003-CH),(z+h+CH,top+0.003),(z+h+CH,top+0.03)]))
                joints_raw.append((z,x,m))
            else:              # lower interval: frontal plate gets a through-slot, sagittal plate passes through it
                zcut[z].append(box(x-h,lo-0.004,x+h,hi+0.004))
            zkeep[z].append(box(x-0.02,lo-0.02,x+0.02,hi+0.02)); xkeep[x].append(box(z-0.02,lo-0.02,z+0.02,hi+0.02))
BOLTS=[dict(name='BOLT_L',d=16,axis='z',u=-0.075,v=0.47,zr=(-0.30,0.10)),dict(name='BOLT_R',d=16,axis='z',u=0.075,v=0.47,zr=(-0.30,0.10))]
def finish(g,cuts,keep,e,holes):
    g=hollow(g,band=0.024,keep=U(keep).buffer(0.004) if keep else None,minwin=0.0012)
    g=g.difference(U(cuts)) if cuts else g
    if holes: g=g.difference(U(holes))
    g=opn(g,0.003)
    anchors=U(keep) if keep else Polygon()
    g=U([c for c in geoms(g) if c.area>1e-4 and c.intersects(anchors)])
    return close(g,0.0015).intersection(box(-1,0,1,2)).simplify(0.0002,preserve_topology=True)
plates=[];bolts=[];joints=[]
zname={};xname={}
for i,z in enumerate(sorted(ZG)):
    holes=[]; keep=list(zkeep[z])
    for b in BOLTS:
        if b['zr'][0]-1e-6<=z<=b['zr'][1]+1e-6:
            keep.append(Point(b['u'],b['v']).buffer(0.024)); holes.append(Point(b['u'],b['v']).buffer(BOLT[16]['hole']/2,quad_segs=16))
            ZG[z]=U([ZG[z],Point(b['u'],b['v']).buffer(0.024)])
    g=finish(ZG[z],zcut[z],keep,ZE[z],holes)
    if g.is_empty: continue
    n=f'Z{i+1:02d}'; zname[z]=n
    plates.append(dict(name=n,plane='z',pos=z,poly=g,moves=[dict(v=[0,0.50,0],t0=0.25,t1=0.6),dict(v=[0,0,0.7*(z+0.05)],t0=0.65,t1=1.0)]))
for j,x in enumerate(sorted(XG)):
    g=finish(XG[x],xcut[x],xkeep[x],XE[x],[])
    if g.is_empty: continue
    n=f'X{j+1:02d}'; xname[x]=n
    plates.append(dict(name=n,plane='x',pos=x,poly=g,kind='comb',moves=[dict(v=[1.1*x,0,0],t0=0.65,t1=1.0)]))
for z,x,m in joints_raw:
    if z in zname and x in xname: joints.append(dict(a=zname[z],b=xname[x],p=[x,m,z],k='slot'))
for b in BOLTS:
    zs=[z for z in zname if b['zr'][0]-1e-6<=z<=b['zr'][1]+1e-6]
    bb=dict(name=b['name'],d=16,axis='z',u=b['u'],v=b['v'],s0=min(zs)-T/2,s1=max(zs)+T/2,nutside=1,
            moves=[dict(v=[0,0,0.6],t0=0.0,t1=0.22),dict(v=[0,0.5,0],t0=0.25,t1=0.6)])
    bolts.append(bb)
    for z in zs: joints.append(dict(a=zname[z],b=b['name'],p=[b['u'],b['v'],z],k='axis'))
steps=['Yan plakalar X01–X15 sırayla dik konur (bir tezgah şablonu tutar), yuvaları yukarı bakar.',
       'Ön plakalar Z01–Z19 yukarıdan indirilir; her biri tüm yan plakaların üst yuvalarına aynı anda oturur.',
       'İki M16 cıvata oturak hizasındaki deliklerden sürülür, pul + somun ile sıkılır.']
if __name__=='__main__':
    info=export('C',plates,bolts,joints,dict(env.pose(),footY=0.0,s=1.0),steps)
    print(info,'Z',len(zname),'X',len(xname),'joints',len(joints_raw))
