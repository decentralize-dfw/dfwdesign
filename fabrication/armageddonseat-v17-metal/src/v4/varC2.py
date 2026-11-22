# Variant C "Kabuk" (based on A): original figure as frame + continuous seat-and-back sagittal shell plates: original frontal slices, original proportions (1:1), upper body leaned back 13 deg.
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
SH_T=0.030
ENV0={z:slice_env(env,'z',z,(-0.42,0.42)) for z in Z}
def slice_env_off(z,d):
    from wf import GU,GV,_poly
    gu=GU[(GU>=-0.42)&(GU<=0.42)]; uu,vv=np.meshgrid(gu,GV); x,y=uu.ravel(),vv.ravel(); zz=np.full_like(x,z)
    I=np.zeros_like(x,bool)
    for dy in np.linspace(0,d,4):
        for dz in np.linspace(0,d,4):
            I|=env.inside(x,y+dy,zz+dz)
    I=I.reshape(uu.shape).astype(np.float32)
    if I.max()<0.5: return Polygon()
    return opn(_poly(gu,GV,-I,-0.5),0.006).buffer(0.003)
SHELL_TOP=env.YSR+0.52
ENV={z:U([slice_env_off(z,SH_T+0.024).intersection(box(-1,-1,1,SHELL_TOP+0.03)),ENV0[z]]) for z in Z}
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
LAP=[z for z in Z if -0.175-1e-6<=z<=0.15+1e-6]
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
    lo=lambda s,m=max(t for _,t in tops):m+0.004
    combs.append(Comb('Y'+n,xc,'above',zs,lo,lambda s,lo=lo:lo(s)+0.05,bolt_off=-0.048*sgn))
    # hand plate: side profile of the hand on the floor, under the arm ribs
    hx=0.33*sgn; hp=slice_fig(Figure(s=1.0,phi=13,kx=1.08,mode='filled'),'x',hx,(-0.45,-0.26)).intersection(box(-0.45,0,-0.26,0.085))
    combs.append(Comb('H'+n,hx,'below',ARMS,lambda s:0.012,lambda s:0.056,bolt_off=-0.045*sgn,floor=True,extra=close(U([hp,box(-0.42,0,-0.30,0.03)]),0.01)))
    # foot plate: side profile of the foot (toes forward), under the shin ribs
    fx=0.142*sgn; fp=slice_fig(Figure(s=1.0,phi=13,kx=1.08,mode='filled'),'x',fx,(0.16,0.45)).intersection(box(0.16,0,0.45,0.10))
    combs.append(Comb('F'+n,fx,'below',KNEE+[z for z in Z if 0.25<z<0.29],lambda s:0.012,lambda s:0.056,bolt_off=-0.045*sgn,floor=True,extra=close(U([fp,box(0.17,0,0.42,0.03)]),0.01)))
# ---------------- shell plates (seat + back in one sagittal profile) ----------------
SX=[float(x) for x in np.round(np.arange(-0.2125,0.2126,0.025),4)]
PELVIS=-0.2; KNEEP=0.15; BACKZ=-0.35
def env_y_at(z,x):   # lowest sitter-envelope height on the vertical line (x,z)
    ys_=np.arange(0.08,1.6,0.002); ins=env.inside(np.full_like(ys_,x),ys_,np.full_like(ys_,z))
    return float(ys_[np.argmax(ins)]) if ins.any() else None
def mk_band(z,dy_top,depth):
    return (lambda s,z=z:env_y_at(z,s)-dy_top-depth, lambda s,z=z:env_y_at(z,s)-dy_top)
_lo,_hi=mk_band(PELVIS,0.004+SH_T/1.0+0.004,0.065)
_lo2,_hi2=mk_band(KNEEP,0.004+SH_T+0.004,0.065)
_lo3,_hi3=mk_band(BACKZ,0.004+SH_T/np.tan(env.REC)*0.9+0.004,0.065)
BX=[x for x in SX if env_y_at(BACKZ,x) is not None and env_y_at(BACKZ,x)<env.YSR+0.47]
shcombs=[Comb('CP',PELVIS,'below',SX,_lo,_hi,bolt=False,pad_w=0.016),
         Comb('CK',KNEEP,'below',SX,_lo2,_hi2,bolt=False,pad_w=0.016),
         Comb('CB',BACKZ,'below',BX,_lo3,_hi3,bolt=False,pad_w=0.016)]
# shell bolts run through the shell band itself (where the ribs are carved away), between two rib planes
def band_center(z,x=0.0):
    y=env_y_at(z,x); return y-SH_T/2 if y is not None else None
SBOLTS=[dict(name='BOLT_CP',z=PELVIS+0.0125,y=band_center(PELVIS+0.0125),hosts=SX),
        dict(name='BOLT_CK',z=KNEEP-0.0125,y=band_center(KNEEP-0.0125),hosts=SX),
        dict(name='BOLT_CB',z=BACKZ+0.0125,y=None,hosts=[x for x in BX if abs(x)<0.09])]
def _back_y(z):   # height on the centre line where the back band crosses z
    ys_=np.arange(0.5,1.4,0.002); ins=env.inside(np.zeros_like(ys_),ys_,np.full_like(ys_,z))
    ins2=env.inside(np.zeros_like(ys_),ys_,np.full_like(ys_,z+SH_T))
    band=(~ins)&ins2
    return float(ys_[band].mean()) if band.any() else None
SBOLTS[2]['y']=_back_y(BACKZ+0.0125)
def shell_plate(x):
    e=ENV0_x(x)
    band=e.buffer(SH_T,join_style=1).difference(e)
    top=env.YSR+0.52
    g=band.intersection(box(-0.62,0.36,env.ZF+0.035,top))
    g=U([c for c in geoms(g) if c.area>2e-4])
    mine=[c for c in shcombs if any(abs(x-h)<1e-6 for h in c.hosts)]
    pads=[c.host_pad(x,g) for c in mine]; cuts=[c.host_cut(x) for c in mine]
    holes=[Point(b['z'],b['y']).buffer(BOLT[16]['hole']/2,quad_segs=16) for b in SBOLTS if b['y'] is not None and any(abs(x-h)<1e-6 for h in b['hosts'])]
    pads+=[Point(b['z'],b['y']).buffer(0.0165) for b in SBOLTS if b['y'] is not None and any(abs(x-h)<1e-6 for h in b['hosts'])]
    g=U([g]+pads).difference(e); g=connect(g,pads,e,maxlen=0.2)
    g=close(g,0.006).difference(e); g=U([g]+[p.difference(e) for p in pads])
    g=U([c for c in geoms(g) if c.area>8e-5 and c.intersects(U(pads).buffer(0.002))])
    g=close(g,0.0015).intersection(box(-1,0,1,2)).simplify(0.0002,preserve_topology=True)
    return g.difference(U(cuts)).difference(U(holes))
def ENV0_x(x): return slice_env(env,'x',x,(-0.6,0.46))
def shell_comb_apply(z,g):
    for c in shcombs:
        if abs(c.pos-z)<1e-6:
            # clear the rib above the band where the shell plates come down, add the band, cut the slots
            clear=U([box(xs-0.0125,c.yhi(xs),xs+0.0125,1.6) for xs in c.hosts])
            g=g.difference(clear)
            band=Polygon([(xs,c.yhi(xs)) for xs in np.linspace(c.sa-T/2-0.014,c.sb+T/2+0.014,40)]+[(xs,c.ylo(xs)) for xs in np.linspace(c.sb+T/2+0.014,c.sa-T/2-0.014,40)])
            g=U([g,close(opn(band,0.005),0.005)])
            h=SLOT/2; cuts=[]
            for xs in c.hosts:
                hi,mid=c.yhi(xs),c.ymid(xs)
                cuts.append(Polygon([(xs-h,mid),(xs+h,mid),(xs+h,hi-CH),(xs+h+CH,hi),(xs+h+CH,hi+0.03),(xs-h-CH,hi+0.03),(xs-h-CH,hi),(xs-h,hi-CH)]))
            g=close(g,0.0015).difference(U(cuts))
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
    g=close(g,0.0015).intersection(box(-1,0,1,2)).simplify(0.0002,preserve_topology=True)
    g=shell_comb_apply(z,g)
    return g.difference(U(cuts)).difference(U(holes))
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
for x in SX:
    g=shell_plate(x)
    if g.is_empty: continue
    plates.append(dict(name=f'S{int(round((x+0.2125)/0.025))+1:02d}',plane='x',pos=x,poly=g,moves=[dict(v=[0,0.75,0],t0=0.22,t1=0.45),dict(v=[0.9*x,0,0],t0=0.72,t1=1.0)]))
rib_of={p['pos']:p['name'] for p in plates if p['plane']=='z'}
sh_of={p['pos']:p['name'] for p in plates if p['plane']=='x' and p['name'].startswith('S')}
for c in shcombs:
    for x in c.hosts:
        if x in sh_of and c.pos in rib_of:
            joints.append(dict(a=sh_of[x],b=rib_of[c.pos],p=[x,c.ymid(x),c.pos],k='slot'))
for sb in SBOLTS:
    if sb['y'] is None: continue
    hs=[x for x in sb['hosts'] if x in sh_of]
    b=dict(name=sb['name'],d=16,axis='x',u=sb['z'],v=sb['y'],s0=min(hs)-T/2,s1=max(hs)+T/2,nutside=1,
           moves=[dict(v=[0.55,0,0],t0=0.0,t1=0.2),dict(v=[0,0.75,0],t0=0.22,t1=0.45)])
    bolts.append(b)
    for x in hs: joints.append(dict(a=sh_of[x],b=b['name'],p=[x,sb['y'],sb['z']],k='axis'))
steps=['Omurga laması K01, el plakaları HL/HR ve ayak plakaları FL/FR yere/sehpaya dik konur (yuvalar yukarı).',
       'Figürün kaburgaları numara sırasıyla yukarıdan yuvalarına indirilir.',
       'Omuz tarakları YL/YR omuz dilleri üzerine oturtulur.',
       'Kabuk plakaları S01–S18 (oturak + sırt tek parça) yukarıdan indirilir; pelvis, diz ve sırt kaburgalarının üst yuvalarına geçer.',
       'M16 cıvatalar tarakların yanındaki deliklerden sürülür; torkla sıkılınca plakalar tarak dişlerini sıkıştırıp kilitler.']
if __name__=='__main__':
    info=export('C',plates,bolts,joints,dict(env.pose(),footY=0.06,s=1.0),steps)
    print(info, 'ribs',len([p for p in plates if p['plane']=='z']))
