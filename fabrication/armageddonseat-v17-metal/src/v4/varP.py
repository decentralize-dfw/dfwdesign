# Re-posed variants of the original figure: the figure itself is the chair (lap = seat, chest = back, arms/legs = legs).
# Laser-cut plates + M16 bolts only. Usage: python3 varP.py V1|V2|V3
from comb import *
from pose import Pose, J
import json, sys, pickle
NAME=sys.argv[1]
PZ=json.load(open('poses2.json'))[NAME]
PZ={k:(tuple(v) if isinstance(v,list) else v) for k,v in PZ.items()}
fig=Pose(**PZ,mode='detail'); figF=Pose(**PZ,mode='filled'); s=PZ['s']
def pt(seg,key): return fig.point(seg,*J[key])
HIPp=pt(5,'HIP'); KNEEp=pt(6,'KNEE'); ANKp=pt(6,'ANK'); SHp=pt(1,'SH'); ELp=pt(2,'EL'); WRp=pt(3,'WR')
thigh_top_front=fig.point(5,0.12,0.575)[1]; thigh_top_rear=fig.point(5,-0.15,0.575)[1]
ENVP=dict(V1=dict(recline=12,tilt=4,depth=0.40),V2=dict(recline=12,tilt=4,depth=0.39),V3=dict(recline=14,tilt=7,depth=0.40))[NAME]
_zf=KNEEp[0]-0.06*s
if NAME=='V1': ENVP.update(front_slope=0.5,drop_at=0.13)
elif NAME=='V2': ENVP.update(front_slope=0.5,drop_at=max(0.13,ANKp[0]+0.07-_zf))
else:   # lounge: the sitter's calves rest on the figure's shins (leg rest), feet on its feet
    shin_slope=(KNEEp[1]-ANKp[1])/(ANKp[0]-KNEEp[0]); ENVP.update(front_slope=shin_slope*0.92,drop_at=ANKp[0]+0.04-_zf)
seat_y=min(thigh_top_front-0.012,0.55)
env=Envelope(seat_y=seat_y,zf=_zf,**ENVP)
print(NAME,'seat',round(seat_y,3),'rear',round(env.YSR,3),'ZF',round(env.ZF,3),'ZBS',round(env.ZBS,3))
def ys(z): return float(env.ys(np.array([z]))[0])
def ell(cx,cy,a,b): return affinity.scale(Point(cx,cy).buffer(1,quad_segs=32),a,b)
def bez(p0,p1,p2,w,n=30):
    t=np.linspace(0,1,n)[:,None]; pts=(1-t)**2*np.array(p0)+2*(1-t)*t*np.array(p1)+t**2*np.array(p2)
    return LineString(pts).buffer(w/2,cap_style=1,join_style=1)
# ---------------- slices ----------------
ZALL=[float(z) for z in np.round(np.arange(-0.95,0.75,0.025),4)]
RAW={}
for z in ZALL:
    g=slice_fig(fig,'z',z,(-0.45,0.45))
    if not g.is_empty and g.area>3e-4: RAW[z]=g
def mat_at(z,x0,x1,y0,y1):
    if z not in RAW: return Polygon()
    return RAW[z].intersection(box(x0,y0,x1,y1))
# arm columns (rear legs): ribs where arm material runs from near the floor to >0.3 m
def arm_col(z):
    g=mat_at(z,0.20*s,0.45,0,0.6)
    return (not g.is_empty) and g.bounds[1]<0.14 and g.bounds[3]>0.3
ARMS=[z for z in sorted(RAW) if arm_col(z) and z<HIPp[0]+0.05]
_am=[mat_at(z,0.18*s,0.45,0.15,0.5) for z in ARMS]
ARMX=float(np.mean([g.centroid.x for g in _am if not g.is_empty]))
print('ARMX',round(ARMX,3),'ARMS',ARMS)
# feet: drop ribs that only hold floor material in front of the shins (foot plates carry the feet)
def floor_only(z): return RAW[z].bounds[3]<0.12
for z in list(RAW):
    if z>KNEEp[0]-0.02 and floor_only(z): RAW.pop(z)
    elif z<HIPp[0] and z not in ARMS:   # hand blobs on the floor away from the arm columns -> hand plates
        RAW[z]=RAW[z].difference(box(-1,-0.1,-0.20*s,0.13)).difference(box(0.20*s,-0.1,1,0.13))
        if RAW[z].is_empty or RAW[z].area<3e-4: RAW.pop(z)
if abs(PZ.get('kn',0))>5:   # inclined shins: use solid shin sections so the shin bone plate can hold every slice
    for z in list(RAW):
        if z>KNEEp[0]-0.05:
            sf=slice_fig(figF,'z',z,(-0.45,0.45)).intersection(box(-0.3,0.0,0.3,KNEEp[1]-0.02))
            parts=[RAW[z]]
            for side in (box(-0.3,0.0,0,KNEEp[1]-0.02),box(0,0.0,0.3,KNEEp[1]-0.02)):
                pc=U([c for c in geoms(U([RAW[z],sf]).intersection(side)) if c.area>2e-5])
                if not pc.is_empty:
                    hx=pc.centroid.x; keep=box(hx-0.016,-1,hx+0.016,2)
                    parts.append(hollow(pc.convex_hull,band=0.013,keep=keep,minwin=0.0003))
            RAW[z]=U(parts)
Z=sorted(RAW)
ENV={z:slice_env(env,'z',z,(-0.45,0.45)) for z in Z}
# ---------------- profile helpers (sagittal, filled figure) ----------------
def profile(x,z0,z1,y0,y1):
    g=slice_fig(figF,'x',x,(z0-0.05,z1+0.05)).intersection(box(z0,y0,z1,y1))
    return g
def lowest(g,z):
    L=g.intersection(LineString([(z,-0.1),(z,2)]))
    ys_=[c[1] for s_ in getattr(L,'geoms',[L]) if not s_.is_empty and s_.geom_type=='LineString' for c in s_.coords]
    return (min(ys_),max(ys_)) if ys_ else None
def fn_from(samples):
    zz=np.array([a for a,_ in samples]); vv=np.array([b for _,b in samples])
    return lambda q,zz=zz,vv=vv: float(np.interp(q,zz,vv))
# ---------------- combs ----------------
combs=[]
# keel: under the lap from the rear arm column to the knee, following the thigh line
r_th=0.072*s; kz0=(ARMS[1] if len(ARMS)>2 else ARMS[0]); kz1=KNEEp[0]+0.03
def thigh_axis(z):
    t_=(z-HIPp[0])/(KNEEp[0]-HIPp[0]); return HIPp[1]+np.clip(t_,0,1)*(KNEEp[1]-HIPp[1])
KH=0.09 if NAME!='V3' else 0.11
LAP=[z for z in Z if HIPp[0]+0.01<=z<=KNEEp[0]-0.03]
KNEE=[z for z in Z if KNEEp[0]-0.03<z<=KNEEp[0]+0.05]
def _keel_ok(z):
    if z in LAP or z in KNEE or z in ARMS[1:3]: return True
    hi=thigh_axis(z)-r_th-0.012
    return not mat_at(z,-0.02,0.02,hi,hi+0.30).is_empty
kz=[z for z in Z if kz0-1e-6<=z<=kz1+1e-6 and _keel_ok(z)]
combs.append(Comb('K01',0.0,'below',kz,lambda q:thigh_axis(q)-r_th-0.012-KH,lambda q:thigh_axis(q)-r_th-0.012,bolt_off=0.048))
for sgn,n in ((-1,'L'),(1,'R')):
    # shoulder yoke
    xc=0.27*s*sgn
    tops=[]
    for z in Z:
        if z>HIPp[0]: continue
        g=mat_at(z,xc-0.02,xc+0.02,0.6*s,1.2)
        if not g.is_empty: tops.append((z,g.bounds[3]))
    top=max(t for _,t in tops); tops=[(z,t) for z,t in tops if t>top-0.16]
    zs=sorted(z for z,_ in tops)
    zs=[z for z in zs if zs[0]-1e-6<=z<=zs[0]+0.15]
    combs.append(Comb('Y'+n,xc,'above',zs,lambda q,m=top:m+0.004,lambda q,m=top:m+0.054,bolt_off=-0.048*sgn))
    # hand plate under the arm columns + the hand side profile
    hx=ARMX*sgn
    hp=profile(hx,ARMS[0]-0.02,ARMS[-1]+0.12,0,0.09)
    combs.append(Comb('H'+n,hx,'below',ARMS,lambda q:0.0,lambda q:0.05,bolt_off=-0.045*sgn,floor=True,
                      extra=close(U([hp,box(ARMS[0]-0.01,0,ARMS[-1]+0.06,0.03)]),0.01)))
    # leg plate: shin bone (lower half of the shin) running down into the foot on the floor
    fx=sgn*abs(fig.fx(6,0.13))*s
    zs_leg=[z for z in Z if KNEEp[0]-0.02<=z]
    prof=profile(fx,KNEEp[0]-0.06,0.75,0,KNEEp[1]+0.02)
    samp_lo=[];samp_hi=[]
    inclined=abs(PZ.get('kn',0))>5
    alpha=np.arctan2(abs(ANKp[0]-KNEEp[0]),abs(KNEEp[1]-ANKp[1]))
    rv=min(0.032*s/max(np.sin(alpha),0.2),0.09)
    for z in np.arange(KNEEp[0]-0.04,max(zs_leg)+0.03,0.005):
        if inclined:
            t_=np.clip((z-KNEEp[0])/(ANKp[0]-KNEEp[0]),0,1); yc=KNEEp[1]+t_*(ANKp[1]-KNEEp[1])
            lo=yc-rv; hi=yc
            if z>ANKp[0]-0.01 or lo<0.035: lo,hi=0.0,0.05
            samp_lo.append((z,lo)); samp_hi.append((z,hi)); continue
        r=lowest(prof,z)
        if r is None: continue
        lo=max(0.0,r[0]); th_=r[1]-r[0]; hi=lo+min(0.5*th_,0.05) if lo>0.02 else 0.05
        samp_lo.append((z,0.0 if lo<0.02 else lo)); samp_hi.append((z,hi))
    ylo=fn_from(samp_lo); yhi=fn_from(samp_hi)
    hosts=[z for z in zs_leg if not mat_at(z,fx-0.005,fx+0.005,ylo(z),yhi(z)).is_empty]
    foot=prof.intersection(box(-1,0,1,0.10))
    combs.append(Comb('F'+n,fx,'below',hosts,ylo,yhi,bolt_off=-0.045*sgn,floor=True,extra=close(U([foot]),0.01)))
    # upper-arm bone plate (only when the upper arm is inclined)
    ang=np.degrees(np.arctan2(abs(ELp[0]-SHp[0]),abs(ELp[1]-SHp[1])))
    if ang>12:
        xs_=[mat_at(z,0.18*s,0.45,0.3*s,0.9*s).centroid.x for z in Z if min(SHp[0],ELp[0])<=z<=max(SHp[0],ELp[0]) and not mat_at(z,0.18*s,0.45,0.3*s,0.9*s).is_empty]
        ax_=(float(np.mean(xs_)) if xs_ else ARMX)*sgn
        za,zb_=sorted([SHp[0],ELp[0]])
        span=(za,zb_-0.30*(zb_-za)) if SHp[0]>ELp[0] else (za+0.30*(zb_-za),zb_)
        prof=profile(ax_,span[0]-0.03,span[1]+0.03,0.25*s,1.1*s)
        sl=[];sh_=[]
        for z in np.arange(span[0],span[1],0.005):
            r=lowest(prof,z)
            if r is None: continue
            hi=r[1]; th_=r[1]-r[0]; sl.append((z,hi-min(0.5*th_,0.04))); sh_.append((z,hi))
        if sl:
            alo=fn_from(sl); ahi=fn_from(sh_)
            hosts=[z for z in Z if span[0]<=z<=span[1] and not mat_at(z,ax_-0.005,ax_+0.005,alo(z),ahi(z)).is_empty]
            if len(hosts)>=2: combs.append(Comb('A'+n,ax_,'above',hosts,alo,ahi,bolt=False,pad_w=0.016))
# ---------------- ribs ----------------
def thigh_rings(z,g):
    out=[]
    for sgn in (-1,1):
        part=g.intersection(box(min(0,sgn),0.0,max(0,sgn),2)).intersection(box(-0.3,thigh_axis(z)-0.11*s,0.3,thigh_axis(z)+0.11*s))
        cs=[c for c in geoms(part) if c.area>2e-5]
        if len(cs)<3: continue
        pts=np.array([c.centroid.coords[0] for c in cs]); cen=pts.mean(0); r=np.mean(np.linalg.norm(pts-cen,axis=1))
        if 0.03<r<0.11: out.append((cen,r))
    return out
LAP=[z for z in Z if HIPp[0]+0.01<=z<=KNEEp[0]-0.03]
KNEE=[z for z in Z if KNEEp[0]-0.03<z<=KNEEp[0]+0.05]
def make_rib(z):
    g=RAW[z]; e=ENV[z]; add=[]
    if z in LAP:
        rings=thigh_rings(z,g)
        for cen,r in rings: add.append(Point(*cen).buffer(r+0.006).difference(Point(*cen).buffer(r-0.006)))
        if len(rings)==2:
            (c1,r1),(c2,r2)=sorted(rings,key=lambda t:t[0][0]); top=ys(z)
            add.append(bez((c1[0]+r1*0.4,top-0.012),(0,top-0.004),(c2[0]-r2*0.4,top-0.012),0.016))
    if z in KNEE:
        ya=thigh_axis(z); add.append(bez((-0.14*s,ya-0.03),(0,ya-0.09),(0.14*s,ya-0.03),0.018))
    if z in ARMS[1:3]:
        ky=thigh_axis(z)-r_th-0.012-KH*0.5; add.append(bez((-ARMX,ky-0.03),(0,ky+0.06),(ARMX,ky-0.03),0.022))
    g=U([g]+add).difference(e)
    hulls=[]
    for zone in (box(-0.3,0.0,0.3,env.YSR+0.06),box(-0.26,env.YSR+0.08,0.26,env.SH+0.02),box(-0.26,env.SH+0.02,0.26,2)):
        part=U([c for c in geoms(g.intersection(zone)) if c.area>1.5e-4])
        if not part.is_empty and part.intersects(e.buffer(0.004)) and (part.bounds[2]-part.bounds[0])>0.06:
            if zone.bounds[3]<env.YSR+0.1: hulls+=[c.convex_hull for c in geoms(close(part,0.02)) if (c.bounds[2]-c.bounds[0])>0.06]
            else: hulls.append(part.convex_hull)
    if hulls and not e.is_empty: g=U([g,e.buffer(0.014,join_style=1).difference(e).intersection(U(hulls).buffer(0.003))])
    pads=[];cuts=[];holes=[]
    for c in combs:
        if any(abs(z-h)<1e-6 for h in c.hosts):
            pads.append(c.host_pad(z,g)); cuts.append(c.host_cut(z))
            if c.has_bolt: holes.append(c.host_hole())
    g=U([g]+pads); g=connect(g,pads,e)
    g=close(g,0.008).difference(e); g=U([g]+[p.difference(e) for p in pads])
    g=g.difference(U(cuts)).difference(U(holes))
    g=opn(g,0.0035).union(g.intersection(U(pads)))
    g=U([c for c in geoms(g) if c.area>8e-5 and c.intersects(U(pads).buffer(0.002))])
    g=close(g,0.0015).intersection(box(-1,0,1,2)).simplify(0.0002,preserve_topology=True)
    return g.difference(U(cuts)).difference(U(holes))
plates=[];bolts=[];joints=[]
LIFT=0.30; ZC=0.5*(Z[0]+Z[-1])
for i,z in enumerate(Z):
    g=make_rib(z)
    if g.is_empty: continue
    plates.append(dict(name=f'R{i+1:02d}',plane='z',pos=z,poly=g,moves=[dict(v=[0,LIFT,0],t0=0.40,t1=0.72),dict(v=[0,0,0.55*(z-ZC)],t0=0.72,t1=1.0)]))
rib_of={p['pos']:p['name'] for p in plates if p['plane']=='z'}
for c in combs:
    if c.side=='above': mv=[dict(v=[0,LIFT+0.30,0],t0=0.22,t1=0.48)]
    else: mv=[dict(v=[0,-0.04 if not c.floor else 0,0],t0=0.40,t1=0.72)]
    plates.append(dict(name=c.name,plane='x',pos=c.pos,poly=c.outline(),kind='comb',moves=mv))
    for h in c.hosts:
        if h in rib_of: joints.append(dict(a=rib_of[h],b=c.name,p=[c.pos,c.ymid(h),h],k='slot'))
    if c.has_bolt:
        b=c.bolt('z'); b['moves']=[dict(v=[0,0,0.55],t0=0.0,t1=0.25),dict(v=[0,LIFT,0],t0=0.40,t1=0.72)]; bolts.append(b)
        bu,bv=c.bolt_uv()
        for h in c.hosts:
            if h in rib_of: joints.append(dict(a=rib_of[h],b=b['name'],p=[bu,bv,h],k='axis'))
steps=['Omurga laması K01, el plakaları HL/HR ve bacak plakaları FL/FR (baldır kemiği + ayak) dik konur, yuvalar yukarı.',
       'Kaburgalar numara sırasıyla yukarıdan yuvalarına indirilir; tarak dişleri kaburgaların arasına girer.']
if any(c.name.startswith('A') for c in combs): steps.append('Üst kol kemik plakaları AL/AR üst kol kesitlerinin üst yuvalarına oturtulur.')
steps+=['Omuz tarakları YL/YR omuz dilleri üzerine oturtulur.',
        'Her tarağın yanındaki M16 cıvata deliklerden sürülür; torkla sıkılınca kaburgalar tarak dişlerini sıkıştırıp kilitler.']
if __name__=='__main__':
    info=export(NAME,plates,bolts,joints,dict(env.pose(),footY=0.0 if NAME!='V1' else 0.10,s=s),steps)
    print(info,'ribs',len([p for p in plates if p['plane']=='z']),'combs',[c.name for c in combs])
