# Variant B "Hibrit" (based on A): original frontal slices, original proportions (1:1), upper body leaned back 13 deg.
# Joints: centre keel + 2 shoulder yokes + 2 foot plates + 2 hand plates (all laser-cut combs) and one big bolt per comb.
from comb import *
import pickle, sys
fig=Figure(s=1.0,phi=13,kx=1.08,mode='detail')
env=Envelope(seat_y=0.545,zf=0.15,depth=0.40,recline=15)
ZALL=[float(z) for z in np.round(np.arange(-0.55,0.30,0.025),4)]   # frontal feet ribs replaced by foot plates
RAW={}
for z in ZALL:
    g=slice_fig(fig,'z',z,(-0.42,0.42))
    if -0.41<z<-0.27: g=g.difference(box(-1,-0.1,-0.24,0.13)).difference(box(0.24,-0.1,1,0.13))  # hand blobs -> hand plates
    if not g.is_empty and g.area>3e-4: RAW[z]=g
Z=sorted(RAW)
ENV={z:slice_env(env,'z',z,(-0.42,0.42)) for z in Z}
def ys(z): return float(env.ys(np.array([z]))[0])
def ell(cx,cy,a,b): return affinity.scale(Point(cx,cy).buffer(1,quad_segs=32),a,b)
def bez(p0,p1,p2,w,n=30):
    t=np.linspace(0,1,n)[:,None]; pts=(1-t)**2*np.array(p0)+2*(1-t)*t*np.array(p1)+t**2*np.array(p2)
    return LineString(pts).buffer(w/2,cap_style=1,join_style=1)
# ---------------- features measured on the figure ----------------
def thigh_rings(z,g):
    out=[]
    for sgn in (-1,1):
        part=g.intersection(box(min(0,sgn),0.40,max(0,sgn),0.66)).intersection(box(-0.3,0.40,0.3,0.66))
        cs=[c for c in geoms(part) if c.area>2e-5]
        if len(cs)<3: continue
        pts=np.array([c.centroid.coords[0] for c in cs]); cen=pts.mean(0); r=np.mean(np.linalg.norm(pts-cen,axis=1))
        out.append((cen,r))
    return out
def shoulder_top(z,xc):
    g=RAW[z].intersection(box(xc-0.02,0.75,xc+0.02,1.08))
    return max(c.bounds[3] for c in geoms(g)) if not g.is_empty else None
def arm_bottom(z,sgn):
    g=RAW[z].intersection(box(min(0.25*sgn,0.42*sgn),0.0,max(0.25*sgn,0.42*sgn),0.6))
    return min(c.bounds[1] for c in geoms(g)) if not g.is_empty else None
LAP=[]  # frontal lap ribs replaced by sagittal lap plates
Z=[z for z in Z if not (-0.19<z<0.17)]
PELVIS=-0.2; KNEEP=0.175
KNEE=[z for z in Z if 0.175-1e-6<=z<=0.25+1e-6]
ARMS=[z for z in Z if arm_bottom(z,1) is not None and arm_bottom(z,1)<0.2 and z<-0.38]
# ---------------- combs ----------------
combs=[]
kz=[z for z in Z if ARMS[1]-1e-6<=z<=KNEE[-1]+1e-6]
combs.append(Comb('K01',0.0,'below',kz,lambda s:0.325,lambda s:0.415,bolt_off=0.048))
for sgn,n in ((-1,'L'),(1,'R')):
    xc=0.27*sgn
    tops=[(z,shoulder_top(z,xc)) for z in Z if z<-0.3]
    tops=[(z,t) for z,t in tops if t is not None and t>0.75]
    zs=[z for z,_ in tops]
    a,b=np.polyfit([z for z,_ in tops],[t for _,t in tops],1)
    lo=lambda s,a=a,b=b,m=max(t-(a*z+b) for z,t in tops):a*s+b+m+0.004
    combs.append(Comb('Y'+n,xc,'above',zs,lo,lambda s,lo=lo:lo(s)+0.05,bolt_off=-0.048*sgn))
    # hand plate: side profile of the hand on the floor, under the arm ribs
    hx=0.33*sgn; hp=slice_fig(Figure(s=1.0,phi=13,kx=1.08,mode='filled'),'x',hx,(-0.45,-0.26)).intersection(box(-0.45,0,-0.26,0.085))
    combs.append(Comb('H'+n,hx,'below',ARMS,lambda s:0.012,lambda s:0.056,bolt_off=-0.045*sgn,floor=True,extra=close(U([hp,box(-0.42,0,-0.30,0.03)]),0.01)))
    # foot plate: side profile of the foot (toes forward), under the shin ribs
    fx=0.142*sgn; fp=slice_fig(Figure(s=1.0,phi=13,kx=1.08,mode='filled'),'x',fx,(0.16,0.45)).intersection(box(0.16,0,0.45,0.10))
    combs.append(Comb('F'+n,fx,'below',KNEE+[z for z in Z if 0.25<z<0.29],lambda s:0.012,lambda s:0.056,bolt_off=-0.045*sgn,floor=True,extra=close(U([fp,box(0.17,0,0.42,0.03)]),0.01)))
# ---------------- lap: sagittal plates on two frontal combs ----------------
figF=Figure(s=1.0,phi=13,kx=1.08,mode='filled')
LAPX=[float(x) for x in np.round(np.arange(-0.2125,0.2126,0.025),4)]
LO,HI=0.355,0.425
lapcombs=[Comb('LP',PELVIS,'below',LAPX,lambda s:LO,lambda s:HI,bolt_off=-0.045,pad_w=0.02),
          Comb('LK',KNEEP,'below',LAPX,lambda s:LO,lambda s:HI,bolt_off=0.045,pad_w=0.02)]
def lap_plate(x):
    g=slice_fig(figF,'x',x,(-0.30,0.30)).intersection(box(-0.30,HI-0.005,0.30,0.70))
    strip=Polygon([(z,ys(z)+0.004) for z in np.linspace(env.ZBS-0.02,env.ZF+0.06,30)]+[(z,ys(z)-0.05) for z in np.linspace(env.ZF+0.06,env.ZBS-0.02,30)])
    g=U([g,strip,box(PELVIS-0.03,HI-0.005,KNEEP+0.03,HI+0.03)])
    e=slice_env(env,'x',x,(-0.30,0.30))
    g=close(g,0.01).difference(e)
    g=hollow(g,band=0.02,keep=U([box(PELVIS-0.045,0,PELVIS+0.07,2),box(KNEEP-0.07,0,KNEEP+0.045,2)]),minwin=0.0005)
    pads=[c.host_pad(x,g) for c in lapcombs]; cuts=[c.host_cut(x) for c in lapcombs]; holes=[c.host_hole() for c in lapcombs]
    g=U([g]+pads); g=connect(g,pads,e)
    g=close(g,0.008).difference(e); g=U([g]+[p.difference(e) for p in pads])
    g=g.difference(U(cuts)).difference(U(holes))
    g=opn(g,0.0035).union(g.intersection(U(pads)))
    g=U([c for c in geoms(g) if c.area>8e-5 and c.intersects(U(pads).buffer(0.002))]).difference(U(cuts)).difference(U(holes))
    return close(g,0.0015).intersection(box(-1,0,1,2)).simplify(0.0002,preserve_topology=True)
def lap_comb_apply(z,g):
    for c in lapcombs:
        if abs(c.pos-z)<1e-6:
            g=g.difference(box(-0.225,HI,0.225,0.75))
            g=U([g,band_poly(c.sa-T/2-0.02,c.sb+T/2+0.02,c.ylo,c.yhi)])
            h=SLOT/2; cuts=[]
            for xs in c.hosts:
                cuts.append(Polygon([(xs-h,c.ymid(0)),(xs+h,c.ymid(0)),(xs+h,HI-CH),(xs+h+CH,HI),(xs+h+CH,HI+0.03),(xs-h-CH,HI+0.03),(xs-h-CH,HI),(xs-h,HI-CH)]))
            hb=c.bolt_uv()
            g=g.difference(U(cuts))
    return g
# ---------------- ribs ----------------
plates=[]; bolts=[]; joints=[]
def make_rib(z):
    g=RAW[z]; e=ENV[z]; add=[]
    if z in LAP:
        for cen,r in thigh_rings(z,g):
            add.append(Point(*cen).buffer(r+0.006).difference(Point(*cen).buffer(r-0.006)))
        rings=thigh_rings(z,g)
        if len(rings)==2:
            (c1,r1),(c2,r2)=sorted(rings,key=lambda t:t[0][0]); top=ys(z)
            add.append(bez((c1[0]+r1*0.5,top-0.012),(0,top-0.004),(c2[0]-r2*0.5,top-0.012),0.016))
    if z in KNEE:
        add.append(bez((-0.15,0.47),(0,0.40),(0.15,0.47),0.018))
    if z in ARMS[1:3]:
        xa=0.30; add.append(bez((-xa,0.30),(0,0.46),(xa,0.30),0.022))
    g=U([g]+add).difference(e)
    # contact skin: continuous curved edge along the carve (seat chord, back, head cradle)
    hulls=[]
    for zone in (box(-0.3,0.40,0.3,env.YSR+0.06),box(-0.26,env.YSR+0.08,0.26,env.SH+0.02),box(-0.26,env.SH+0.02,0.26,2)):
        part=U([c for c in geoms(g.intersection(zone)) if c.area>1.5e-4])
        if not part.is_empty and part.intersects(e.buffer(0.004)) and (part.bounds[2]-part.bounds[0])>0.06:
            if zone.bounds[3]<env.YSR+0.1: hulls+= [c.convex_hull for c in geoms(close(part,0.02)) if (c.bounds[2]-c.bounds[0])>0.06]
            else: hulls.append(part.convex_hull)
    if hulls and not e.is_empty: g=U([g,e.buffer(0.014,join_style=1).difference(e).intersection(U(hulls).buffer(0.003))])
    pads=[]; cuts=[]; holes=[]
    for c in combs:
        if any(abs(z-h)<1e-6 for h in c.hosts):
            pads.append(c.host_pad(z,g)); cuts.append(c.host_cut(z))
            if c.has_bolt: holes.append(c.host_hole())
    g=U([g]+pads)
    g=connect(g,pads,e)
    g=close(g,0.008).difference(e)
    g=U([g]+[p.difference(e) for p in pads])
    g=g.difference(U(cuts)).difference(U(holes))
    g=opn(g,0.0035).union(g.intersection(U(pads)))
    g=U([c for c in geoms(g) if c.area>8e-5 and c.intersects(U(pads).buffer(0.002))])
    g=g.difference(U(cuts)).difference(U(holes))
    g=lap_comb_apply(z,g)
    return close(g,0.0015).intersection(box(-1,0,1,2)).simplify(0.0002,preserve_topology=True)
LIFT=0.30
for i,z in enumerate(Z):
    g=make_rib(z)
    if g.is_empty: continue
    plates.append(dict(name=f'R{i+1:02d}',plane='z',pos=z,poly=g,moves=[dict(v=[0,LIFT,0],t0=0.40,t1=0.72),dict(v=[0,0,0.55*(z+0.15)],t0=0.72,t1=1.0)]))
for c in combs:
    yo=0.62 if c.side=='above' else 0.0
    mv=[dict(v=[0,LIFT+0.30,0],t0=0.22,t1=0.48),dict(v=[0,0,0],t0=0.72,t1=1.0)] if c.side=='above' else [dict(v=[0,-0.04 if not c.floor else 0,0],t0=0.40,t1=0.72)]
    plates.append(dict(name=c.name,plane='x',pos=c.pos,poly=c.outline(),kind='comb',moves=mv))
    b=c.bolt('z'); b['moves']=[dict(v=[0,0,0.55],t0=0.0,t1=0.25),dict(v=[0,LIFT,0],t0=0.40,t1=0.72)]
    bolts.append(b)
    bu,bv=c.bolt_uv()
    rib_of={p['pos']:p['name'] for p in plates if p['plane']=='z'}
    for h in c.hosts:
        if h in rib_of:
            joints.append(dict(a=rib_of[h],b=c.name,p=[c.pos,c.ymid(h),h],k='slot'))
            joints.append(dict(a=rib_of[h],b=b['name'],p=[bu,bv,h],k='axis'))
for x in LAPX:
    g=lap_plate(x)
    if g.is_empty: continue
    plates.append(dict(name=f'S{int(round((x+0.2125)/0.025))+1:02d}',plane='x',pos=x,poly=g,moves=[dict(v=[0,0.55,0],t0=0.22,t1=0.45),dict(v=[0.9*x,0,0],t0=0.72,t1=1.0)]))
rib_of={p['pos']:p['name'] for p in plates if p['plane']=='z'}
lap_of={p['pos']:p['name'] for p in plates if p['plane']=='x' and p['name'].startswith('S')}
for c in lapcombs:
    b=c.bolt('x'); b['moves']=[dict(v=[0.55,0,0],t0=0.0,t1=0.2),dict(v=[0,0.55,0],t0=0.22,t1=0.45)]; bolts.append(b)
    bu,bv=c.bolt_uv()
    for x in c.hosts:
        if x in lap_of:
            joints.append(dict(a=lap_of[x],b=rib_of.get(c.pos,''),p=[x,c.ymid(0),c.pos],k='slot'))
            joints.append(dict(a=lap_of[x],b=b['name'],p=[x,bv,bu],k='axis'))
steps=['Omurga laması K01, el plakaları HL/HR ve ayak plakaları FL/FR yere/sehpaya dik konur (yuvalar yukarı).',
       'Gövde, pelvis ve diz kaburgaları numara sırasıyla yukarıdan yuvalarına indirilir.',
       'Omuz tarakları YL/YR omuz dilleri üzerine oturtulur.',
       'Oturak plakaları S01–S18 pelvis ve diz kaburgalarının üst yuvalarına yukarıdan geçirilir.',
       'M16 cıvatalar taraklann yanındaki deliklerden sürülür, pul + somun ile sıkılır.']
if __name__=='__main__':
    info=export('B',plates,bolts,joints,dict(env.pose(),footY=0.06,s=1.0),steps)
    print(info, 'ribs',len([p for p in plates if p['plane']=='z']))
