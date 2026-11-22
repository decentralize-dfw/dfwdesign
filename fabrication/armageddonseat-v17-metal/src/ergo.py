import pickle,numpy as np,matplotlib,warnings;warnings.filterwarnings("ignore")
matplotlib.use('Agg');import matplotlib.pyplot as plt
from shapely.geometry import box
from ribs import geoms, ys, zb, ZF, ZBS, YSR, RECLINE, TILT
import trimesh
d=pickle.load(open('ribs_final.pkl','rb'));ribs=d['ribs'];Z=d['Z'];T=d['T']
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9})
fig,(ax,ax2)=plt.subplots(1,2,figsize=(15,8.2),gridspec_kw=dict(width_ratios=[1.25,1]))
# original side silhouette (ghost)
o=trimesh.load('seat.stl')
from matplotlib.collections import PolyCollection
ax.add_collection(PolyCollection(o.triangles[:,:,[2,1]],facecolor='#d9534f',alpha=.035,edgecolor='none'))
contact=[]
SW=0
for z in Z:
    g,_=ribs[z]
    if g.is_empty: continue
    y0,y1=g.bounds[1],g.bounds[3]
    # vertical extents in side view (draw each rib as its y-intervals at |x|<0.4)
    for c in geoms(g):
        ax.add_patch(plt.Rectangle((z-T/2,c.bounds[1]),T,c.bounds[3]-c.bounds[1],color='#3d4a57',lw=0))
    if -0.1<z<0.1: SW=max(SW,g.bounds[2]-g.bounds[0])
    cen=g.intersection(box(-0.2,0,0.2,2))
    if not cen.is_empty: contact.append((z,cen.bounds[3]))
kp=d['keel']; x,y=kp.exterior.xy; ax.fill(x,y,color='#7d8a96')
# manikin (50th pct male, ~175 cm) - segment capsules
H=np.array([ZBS-0.09*np.tan(RECLINE)+0.125, YSR+0.095])
th=RECLINE
sh=H+0.48*np.array([-np.sin(th),np.cos(th)])
neck=sh+0.07*np.array([-np.sin(th),np.cos(th)])
head=neck+np.array([-0.005,0.13])
seat_ang=np.arctan2(ys(ZF)-ys(ZBS),ZF-ZBS)
knee=H+0.45*np.array([np.cos(seat_ang+0.05),np.sin(seat_ang+0.05)])
ankle=np.array([knee[0]+0.10,0.09]); toe=ankle+np.array([0.20,-0.07])
elbow=sh+np.array([0.05,-0.28]); hand=elbow+np.array([0.26,-0.03])
def cap(a,b,w,**k):
    a,b=np.array(a),np.array(b);v=b-a;n=np.array([-v[1],v[0]])/np.linalg.norm(v)*w/2
    t=np.linspace(0,np.pi,12)
    pts=[a+n, b+n]+[b+ n*np.cos(s) + (v/np.linalg.norm(v))*w/2*np.sin(s) for s in t]+[b-n,a-n]+[a- n*np.cos(s) - (v/np.linalg.norm(v))*w/2*np.sin(s) for s in t]
    ax.add_patch(plt.Polygon(pts,closed=True,**k))
mk=dict(fc='#e8a87c',ec='#b5733f',alpha=.55,lw=1)
cap(H,sh,0.215,**mk); cap(H,knee,0.15,**mk); cap(knee,ankle,0.10,**mk); cap(ankle,toe,0.06,**mk)
cap(neck,head,0.07,**mk); ax.add_patch(plt.Circle(head+np.array([0.01,0.03]),0.095,**mk)); cap(sh,elbow,0.08,**mk); cap(elbow,hand,0.065,**mk)
ax.plot(*H,'ko',ms=4); ax.annotate('H-noktası',H,xytext=(H[0]+0.06,H[1]+0.07),fontsize=8,arrowprops=dict(arrowstyle='-',lw=.6))
# annotate
zz=np.linspace(ZBS,ZF+0.04,50); ax.plot(zz,[ys(z) for z in zz],'--',color='#2a9d8f',lw=1.2)
yy=np.linspace(YSR,1.15,50); ax.plot([zb(y) for y in yy],yy,'--',color='#2a9d8f',lw=1.2)
def dim(p,q,txt,off=(0,0),**k):
    ax.annotate('',q,p,arrowprops=dict(arrowstyle='<->',lw=.8,color='#264653'))
    m=(np.array(p)+np.array(q))/2+np.array(off); ax.text(*m,txt,fontsize=8.5,color='#264653',ha='center',va='center',bbox=dict(fc='white',ec='none',alpha=.8,pad=1))
dim((ZF-0.03,0),(ZF-0.03,ys(ZF-0.03)),f'{ys(ZF-0.03)*1000:.0f}',off=(0.035,0))
dim((ZBS+0.01,0),(ZBS+0.01,YSR),f'{YSR*1000:.0f}',off=(-0.035,-0.1))
dim((ZBS,0.30),(ZF,0.30),f'oturma derinliği {(ZF-ZBS)*1000:.0f}',off=(0,-0.02))
ax.text(-0.665,0.70,f'sırt eğimi\n{np.degrees(RECLINE):.0f}° (dikeyden)\noturak–sırt ≈{90+np.degrees(RECLINE)-np.degrees(TILT):.0f}°',fontsize=8.5,color='#2a9d8f')
ax.text(0.02,ys(0)+0.03,f'oturak eğimi {np.degrees(TILT):.0f}°',fontsize=8.5,color='#2a9d8f')
ax.annotate('bel desteği\n(lumbar) +20 mm',(zb(YSR+0.21),YSR+0.21),xytext=(-0.66,YSR+0.24),fontsize=8,arrowprops=dict(arrowstyle='->',lw=.7))
ax.annotate('sakrum boşluğu',(zb(YSR+0.06),YSR+0.06),xytext=(-0.66,YSR+0.10),fontsize=8,arrowprops=dict(arrowstyle='->',lw=.7))
ax.annotate('şelale ön kenar\nR40',(ZF+0.02,ys(ZF+0.02)),xytext=(ZF+0.06,ys(ZF)+0.12),fontsize=8,arrowprops=dict(arrowstyle='->',lw=.7))
ax.annotate('baş: geriye yaslanınca destek',(-0.44,1.04),xytext=(-0.62,1.22),fontsize=8,arrowprops=dict(arrowstyle='->',lw=.7))
ax.annotate('omurga laması\n(geçmeli salma)',(0.0,0.26),xytext=(-0.07,0.12),fontsize=8,arrowprops=dict(arrowstyle='->',lw=.7))
ax.set_xlim(-0.68,0.82);ax.set_ylim(-0.02,1.32);ax.set_aspect('equal');ax.set_xlabel('derinlik z (m) — ön →');ax.set_ylabel('yükseklik y (m)')
ax.set_title('Yan kesit: yeni geometri + %50 erkek manken (soluk kırmızı: orijinal model)',fontsize=10)
ax.grid(alpha=.25)
# front view with seat width/hip check
for z in Z:
    g,_=ribs[z]
    for c in geoms(g):
        x,y=c.exterior.xy; ax2.fill(x,y,color='#3d4a57',alpha=.08,lw=0)
# 95th pct female hip breadth seated ~ 0.44, 95th male ~0.41
ax2.add_patch(plt.Rectangle((-0.22,ys(0)),0.44,0.16,fc='#e8a87c',alpha=.4,ec='#b5733f'))
ax2.text(0,ys(0)+0.08,'kalça genişliği\n%95 kadın 440 mm',ha='center',fontsize=8)
ax2.annotate('',(SW/2,0.47),(-SW/2,0.47),arrowprops=dict(arrowstyle='<->',lw=.8))
ax2.text(0,0.485,f'oturak {SW*1000:.0f} mm',ha='center',fontsize=8.5)
ax2.set_xlim(-0.45,0.45);ax2.set_ylim(-0.02,1.32);ax2.set_aspect('equal');ax2.set_title('Ön görünüş (tüm kaburgalar üst üste)',fontsize=10);ax2.grid(alpha=.25)
plt.tight_layout();plt.savefig('ergonomi.png',dpi=130)
print('contact',[(round(z,3),round(y,3)) for z,y in contact])
print('H',H,'knee',knee,'seat front',ys(ZF-0.03),'rear',YSR)
