import pickle,json,numpy as np,ezdxf,warnings,os,shutil;warnings.filterwarnings("ignore")
from shapely.geometry import Polygon,Point,box,LineString
from shapely.ops import unary_union,polylabel
from shapely import affinity
from ribs import geoms,ys,zb,ZF,ZBS,YSR
import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
d=pickle.load(open('ribs_final.pkl','rb'));ribs=d['ribs'];R=d['R'];Z=d['Z'];T=d['T'];K=d['KEEL'];keel=d['keel']
OUT='out';shutil.rmtree(OUT,ignore_errors=True);os.makedirs(OUT+'/dxf',exist_ok=True)
RHO={'al':2700,'st':7850}
# ---------- parts ----------
parts=[]
rodmap0={r['name']:r for r in R}
nz=[z for z in Z if not ribs[z][0].is_empty]
for i,z in enumerate(nz):
    comps=sorted(geoms(ribs[z][0]),key=lambda c:(-c.area))
    comps=sorted(comps[:1],key=lambda c:c.centroid.x)+sorted(comps[1:],key=lambda c:(c.centroid.x,c.centroid.y))
    for k,c in enumerate(comps):
        rn=[n for n in ribs[z][1] if c.buffer(0.0005).contains(Point(rodmap0[n]['x'],rodmap0[n]['y']))]
        pid=f'R{i+1:02d}'+('' if len(comps)==1 else 'abcdefgh'[k])
        parts.append(dict(id=pid,z=z,g=c,rods=rn,rib=f'R{i+1:02d}'))
parts.append(dict(id='K01',z=None,g=keel,rods=[],rib='K01'))
def hole_d(dn): return dn+0.8
rodmap={r['name']:r for r in R}
# ---------- DXF per part (mm) ----------
def to_mm(g): return affinity.scale(g,1000,1000,origin=(0,0))
def write_dxf(path,items,label_h=6):
    doc=ezdxf.new('R2010',setup=True);doc.units=ezdxf.units.MM
    for n,c in (('CUT',1),('HOLES',5),('ETCH',3)): doc.layers.add(n,color=c)
    msp=doc.modelspace()
    for (pid,g,circles,dx,dy,rot) in items:
        gg=affinity.rotate(g,rot,origin=(0,0)) if rot else g
        gg=affinity.translate(gg,dx,dy)
        for p in geoms(gg):
            msp.add_lwpolyline(list(p.exterior.coords),close=True,dxfattribs={'layer':'CUT'})
            for h in p.interiors:
                hp=Polygon(h)
                if any(hp.centroid.distance(Point(c))<0.5 for c,_ in circles_t(circles,rot,dx,dy)) and hp.area<200: continue
                msp.add_lwpolyline(list(h.coords),close=True,dxfattribs={'layer':'CUT'})
        for (c,dia) in circles_t(circles,rot,dx,dy):
            msp.add_circle(c,dia/2,dxfattribs={'layer':'HOLES'})
        big=max(geoms(gg),key=lambda p:p.area)
        lp=polylabel(big,tolerance=0.5)
        msp.add_text(pid,height=label_h,dxfattribs={'layer':'ETCH'}).set_placement((lp.x,lp.y),align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER)
    doc.saveas(path)
def circles_t(circles,rot,dx,dy):
    out=[]
    for (x,y),dia in circles:
        p=affinity.rotate(Point(x,y),rot,origin=(0,0)) if rot else Point(x,y)
        out.append(((p.x+dx,p.y+dy),dia))
    return out
for p in parts:
    g=to_mm(p['g'])
    if p['id']=='K01':
        circ=[((zc*1000,(K['y0']+K['y1'])/2*1000),6.6) for zc in (keel.bounds[0]+0.010,keel.bounds[2]-0.010)]
    else:
        circ=[((rodmap[n]['x']*1000,rodmap[n]['y']*1000),hole_d(rodmap[n]['d'])) for n in p['rods']]
    p['g_mm']=g;p['circ']=circ
    write_dxf(f"{OUT}/dxf/{p['id']}.dxf",[(p['id'],g,circ,0,0,0)])
# ---------- nesting (shelf packing on 3000x1500) ----------
SW,SH,GAP=3000,1500,8
items=[]
for p in parts:
    b=p['g_mm'].bounds; w,h=b[2]-b[0],b[3]-b[1]
    rot=90 if h>w and h<=SH-2*GAP else 0
    if rot: w,h=h,w
    items.append((p,rot,w,h))
items.sort(key=lambda t:-t[3])
sheets=[];cur=None
def newsheet():
    s=dict(items=[],x=GAP,y=GAP,rowh=0);sheets.append(s);return s
cur=newsheet()
for p,rot,w,h in items:
    if cur['x']+w+GAP>SW: cur['x']=GAP;cur['y']+=cur['rowh']+GAP;cur['rowh']=0
    if cur['y']+h+GAP>SH: cur=newsheet()
    g=affinity.rotate(p['g_mm'],rot,origin=(0,0)) if rot else p['g_mm']
    b=g.bounds; dx=cur['x']-b[0]; dy=cur['y']-b[1]
    cur['items'].append((p['id'],p['g_mm'],p['circ'],dx,dy,rot))
    cur['x']+=w+GAP;cur['rowh']=max(cur['rowh'],h)
fig,axs=plt.subplots(len(sheets),1,figsize=(14,7*len(sheets)))
axs=np.atleast_1d(axs)
for k,s in enumerate(sheets):
    write_dxf(f"{OUT}/dxf/NEST_sheet{k+1}_3000x1500.dxf",s['items'],label_h=10)
    ax=axs[k];ax.add_patch(plt.Rectangle((0,0),SW,SH,fc='#f4f4f2',ec='k'))
    for pid,g,circ,dx,dy,rot in s['items']:
        gg=affinity.translate(affinity.rotate(g,rot,origin=(0,0)) if rot else g,dx,dy)
        for q in geoms(gg):
            x,y=q.exterior.xy;ax.fill(x,y,color='#5b6b7a')
            for h in q.interiors: x,y=h.xy;ax.fill(x,y,color='#f4f4f2')
        c=gg.centroid;ax.text(c.x,c.y,pid,fontsize=7,color='#c0392b',ha='center',va='center',weight='bold')
    ax.set_xlim(-20,SW+20);ax.set_ylim(-20,SH+20);ax.set_aspect('equal');ax.set_title(f'Plaka {k+1}/{len(sheets)} — 3000×1500×8 mm, Al 6082-T651');ax.axis('off')
plt.tight_layout();plt.savefig(f'{OUT}/nesting.png',dpi=70)
# ---------- BOM ----------
area=sum(p['g'].area for p in parts)
used=sum(sum(q.area for q in geoms(p['g'])) for p in parts)
m_al=area*T*RHO['al']; m_st=area*T*RHO['st']
rods=[];spacers={};nuts={}
SP={12:('20×3',0.020,0.014),10:('16×2,5',0.016,0.011),8:('14×2,5',0.014,0.009)}
for r in R:
    zs=[z for z in Z if r['z0']-1e-6<=z<=r['z1']+1e-6]
    L=(max(zs)-min(zs))+T+2*(0.002+0.0008*r['d']*1000/1000*0+0.8*r['d']/1000)+0.004
    Lmm=int(np.ceil(L*1000/5)*5)
    rods.append(dict(name=r['name'],M=r['d'],L=Lmm,n_ribs=len(zs)))
    spacers[r['d']]=spacers.get(r['d'],0)+len(zs)-1
    nuts[r['d']]=nuts.get(r['d'],0)+2
def rodmass(dn,L): return {12:0.69,10:0.48,8:0.30}[dn]*L/1000*1.0
def tubem(dn): od,i=SP[dn][1],SP[dn][2]; return np.pi/4*(od**2-i**2)*(P-T)*7900
P=0.025
hw=sum(rodmass(r['M'],r['L']) for r in rods)+sum(tubem(k)*v for k,v in spacers.items())+sum(v*{12:0.035,10:0.022,8:0.012}[k] for k,v in nuts.items())
# group rods by M & length
from collections import Counter
rodgrp=Counter((r['M'],r['L']) for r in rods)
# ---------- statics ----------
E_al=70e9; fy=250e6
# chair CoG (plates + hardware approximated)
cz=sum(p['g'].area*p['z'] for p in parts if p['z'] is not None)/sum(p['g'].area for p in parts if p['z'] is not None)
cy=sum(p['g'].area*p['g'].centroid.y for p in parts if p['z'] is not None)/sum(p['g'].area for p in parts if p['z'] is not None)
Wc=(m_al+hw)*9.81
# floor contacts
cont=[]
for p in parts:
    if p['z'] is None: continue
    for q in geoms(p['g']):
        if q.bounds[1]<0.002:
            seg=q.intersection(box(-1,0,1,0.002))
            if not seg.is_empty: cont.append((p['z'],seg.bounds[0],seg.bounds[2]))
z_rear=min(c[0] for c in cont)-T/2; z_front=max(c[0] for c in cont)+T/2
# backward overturning: 600N on seat at 175mm from front edge; horizontal at back top
F_v=600; z_load=ZF-0.175; h_back=0.95
M_res=F_v*(z_load-z_rear)+Wc*(cz-z_rear)
F_tip_back=M_res/h_back
# occupant leaning: 100 kg person, CoG ~ H-point region shifted back
z_occ=ZBS+0.10; F_tip_back_occ=(981*(z_occ-z_rear)+Wc*(cz-z_rear))/h_back
# sideways: line between rear-outer contact and front-outer contact
rear_out=max(c[2] for c in cont if c[0]<0); front_out=max(c[2] for c in cont if c[0]>0)
A=np.array([rear_out,z_rear]);B=np.array([front_out,z_front])
def dist_line(pt):
    v=B-A;w=np.array(pt)-A; return (v[0]*w[1]-v[1]*w[0])/np.linalg.norm(v)
x_load=0.215-0.02; z_l=ZBS+0.15
d_load=abs(dist_line((x_load,z_l))); d_cg=abs(dist_line((0,cz)))
# EN1022-style lateral: 600N vertical at seat edge + horizontal 20N.. check sign: positive d = inside polygon
F_side_max=(Wc*d_cg+F_v*d_load)/ (ys(z_l))  # horizontal force at seat height causing tipping
# keel beam
L_k=(K['z1']-K['z0']); P_seat=2000
M_k=P_seat*L_k/4; W_k=T*(K['ymid']-K['y0'])**2/6; s_k=M_k/W_k
I_k=T*(K['ymid']-K['y0'])**3/12; dlt_k=P_seat*L_k**3/(48*E_al*I_k)
# composite keel + 5 rods (steel n=2.86)
A_k=T*(K['ymid']-K['y0']); y_k=(K['y0']+K['ymid'])/2
A_r=5*84.3e-6; y_r=(4*0.362+0.385)/5; n=200/70
yc=(A_k*y_k+n*A_r*y_r)/(A_k+n*A_r); I_c=I_k+A_k*(y_k-yc)**2+n*A_r*(y_r-yc)**2
dlt_c=1000*L_k**3/(48*E_al*I_c); s_c=M_k*(yc-K['y0'])/I_c
# rear legs: 3 arm plates per side, narrowest width near floor
legw=0.040; F_leg=0.6*P_seat/2
s_leg=F_leg/(3*legw*T)
Fh=560; M_leg=Fh/4*0.33; s_leg_b=M_leg/(3*legw*T**2/6)
stat=dict(m_al=m_al,m_st=m_st,m_st5=area*0.005*7850,hw=hw,area=area,cz=cz,cy=cy,z_rear=z_rear,z_front=z_front,
  F_tip_back=F_tip_back,F_tip_back_occ=F_tip_back_occ,d_load=d_load,d_cg=d_cg,F_side_max=F_side_max,
  M_k=M_k,s_k=s_k/1e6,dlt_k=dlt_k*1000,dlt_c=dlt_c*1000,s_c=s_c/1e6,I_ratio=I_c/I_k,
  s_leg=s_leg/1e6,s_leg_b=s_leg_b/1e6,n_parts=len(parts),n_sheets=len(sheets),
  rodgrp=[dict(M=k[0],L=k[1],n=v) for k,v in sorted(rodgrp.items())],spacers=spacers,nuts=nuts,rods=rods,parts=[dict(id=p['id'],z=p['z'],rods=p['rods'],w=round((p['g'].bounds[2]-p['g'].bounds[0])*1000),h=round((p['g'].bounds[3]-p['g'].bounds[1])*1000)) for p in parts],
  seat_front=float(ys(ZF-0.03)),seat_rear=float(YSR),seat_depth=ZF-ZBS,
  extents=[0.776,1.164,0.999])
json.dump(stat,open(f'{OUT}/summary.json','w'),indent=1,default=float)
# CSV BOM
with open(f'{OUT}/BOM.csv','w') as f:
    f.write('poz;tanım;malzeme;adet;not\n')
    for p in parts:
        b=p['g_mm'].bounds
        f.write(f"{p['id']};{'Kaburga' if p['id'][0]=='R' else 'Omurga laması (geçmeli salma)'}{' parçası' if len(p['id'])>3 else ''} {b[2]-b[0]:.0f}×{b[3]-b[1]:.0f} mm;Al 6082-T651 8 mm;1;{'z='+format(p['z']*1000,'.0f')+' mm' if p['z'] is not None else 'yarım-geçme yuvalı'}\n")
    for (M,L),c in sorted(rodgrp.items()): f.write(f"ROD-M{M}x{L};Dişli çubuk DIN 976-1 M{M}×{L};A2-70 paslanmaz;{c};\n")
    for k,v in sorted(spacers.items()): f.write(f"SP-{k};Mesafe borusu Ø{SP[k][0]} × 17,0 mm (±0,05);AISI 304;{v};uçlar alınmış, çapaksız\n")
    for k,v in sorted(nuts.items()): f.write(f"NUT-{k};Kep somun DIN 1587 M{k};A2;{v};+ pul DIN 125 A2 ({v} ad.) + Loctite 243\n")
    f.write("BOLT-K;Altıgen cıvata ISO 4017 M6×30 + DIN 985 fiberli somun + 2 pul;A2;2;omurga kilit pimleri\n")
    f.write("FELT;Yapışkanlı keçe/EPDM şerit 10×3 mm;—;1 m;zemine basan kenarlar\n")
print(json.dumps({k:v for k,v in stat.items() if k not in('rods',)},indent=0,default=float))
