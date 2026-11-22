from content import *
from shapely.ops import nearest_points
from shapely import affinity
R=[]
def rod(n,x,y,z0,z1,d): R.append(dict(name=n,x=x,y=y,z0=round(z0,4),z1=round(z1,4),d=d))
ZR=-0.425; ZK=0.20
KEEL=dict(x=0.0,y0=0.250,y1=0.340,ymid=0.300,z0=ZR,z1=ZK,tab=0.020,slot=T+0.0006,chamfer=0.0015)
for s in (-1,1):
    rod(f'L{"1" if s<0 else "2"}',s*0.097,0.362,ZR,ZK,12); rod(f'L{"3" if s<0 else "4"}',s*0.167,0.362,ZR,ZK,12)
rod('L6',0.0,0.385,-0.175,0.15,12)
def ellipse(cx,cy,a,b): return affinity.scale(Point(cx,cy).buffer(1,quad_segs=32),a,b)
def curve_band(f,x0,x1,wd,n=40):
    xs=np.linspace(x0,x1,n); return LineString([(x,f(x)) for x in xs]).buffer(wd/2,cap_style=1,join_style=1)
def bez(p0,p1,p2,n=24):
    t=np.linspace(0,1,n)[:,None]; return (1-t)**2*np.array(p0)+2*(1-t)*t*np.array(p1)+t**2*np.array(p2)
def structure(z,g0):
    add=[]
    ysz=ys0(z)
    # rear girdle: arched band joining the arms (rear seat frame), ribs carrying the arms
    if ZR-1e-6<=z<=-0.375+1e-6:
        add.append(curve_band(lambda x:0.335+0.030*(1-(x/0.33)**2),-0.33,0.33,0.044))
    # pelvis bone (ellipse) through which the seat rods pass
    if -0.40<=z<=-0.20:
        add.append(ellipse(0,0.360,0.215,0.040))
    # lap: bridge between thigh rings + curvy keel fin
    if -0.20<=z<=0.155:
        add.append(ellipse(0,ysz-0.028,0.115,0.030))
    if ZR-1e-6<=z<=ZK+1e-6:
        top=min(ysz,0.40)-0.03; yb=KEEL['y0']-(0.022 if abs(z-ZR)<1e-6 or abs(z-ZK)<1e-6 else 0)
        L=bez((-0.055,top),(-0.018,(top+yb)/2+0.01),(-0.026,yb)); Rr=bez((0.026,yb),(0.018,(top+yb)/2+0.01),(0.055,top))
        add.append(Polygon(np.vstack([L,Rr])).buffer(0.003))
    # knees: sagging arch joining both knees (front seat frame)
    if 0.175-1e-6<=z<=ZK+1e-6:
        add.append(curve_band(lambda x:0.300+0.022*(x/0.16)**2,-0.16,0.16,0.040))
    # arms reach the floor (rear legs): smooth taper of the forearm into a foot
    if ZR-1e-6<=z<=-0.375+1e-6:
        for s in (-1,1):
            arm=U([c for c in geoms(g0) if abs(c.centroid.x)>0.24 and c.centroid.y<0.6 and c.centroid.x*s>0])
            if arm.is_empty: continue
            yb=arm.bounds[1]; low=arm.intersection(box(-1,yb,1,yb+0.07)); xc=low.centroid.x
            add.append(U([low,ellipse(xc,0.010,0.024,0.010)]).convex_hull)
    return U(add)
def hollow(g,band=0.020):
    out=[]
    for c in geoms(g):
        inner=opn(c.buffer(-band,join_style=1),0.010)
        wins=[x for x in geoms(inner) if x.area>0.0006]
        out.append(c.difference(U(wins)) if wins else c)
    return U(out)
# ---- rods placed after geometry is known ----
for k,(x,y) in enumerate([(0.299,0.24),(0.297,0.32),(0.288,0.40),(0.274,0.48)]):
    for s in (-1,1): rod(f'A{"L" if s<0 else "R"}{k+1}',s*x,y,-0.425,-0.375,8)
rod('B1',-0.19,0.72,-0.40,-0.275,10); rod('B2',0.19,0.72,-0.40,-0.275,10)
rod('B3',-0.11,0.84,-0.40,-0.325,10); rod('B4',0.11,0.84,-0.40,-0.325,10)
rod('B5',0.0,0.905,-0.425,-0.40,10)
rod('H1',0.0,0.975,-0.475,-0.40,10); rod('H2',0.0,1.045,-0.475,-0.40,10)
for s in (-1,1):
    n='L' if s<0 else 'R'
    rod(f'E{n}a',s*0.318,0.022,-0.425,-0.30,8); rod(f'E{n}b',s*0.348,0.022,-0.425,-0.30,8)
    rod(f'F{n}a',s*0.120,0.022,0.20,0.40,8); rod(f'F{n}b',s*0.155,0.022,0.20,0.375,8)
    for k,y in enumerate((0.12,0.21,0.29)): rod(f'S{n}{k+1}',s*0.138,y,0.20,0.25,8)
def rods_at(z): return [r for r in R if r['z0']-1e-6<=z<=r['z1']+1e-6]
def boss_r(d): return {12:0.019,10:0.016,8:0.012}[d]
def hole_d(d): return d/1000+0.0008
NUT_R={12:0.0105,10:0.009,8:0.0075}
def keel_cut(z):
    k=KEEL; h=k['slot']/2
    if abs(z-k['z0'])<1e-6 or abs(z-k['z1'])<1e-6: return box(-h,k['y0']-0.0005,h,k['y1']+0.0005)
    if k['z0']<z<k['z1']:
        c=k['chamfer']
        return Polygon([(-h-c,k['y0']-0.02),(-h-c,k['y0']),(-h,k['y0']+c),(-h,k['ymid']),(h,k['ymid']),(h,k['y0']+c),(h+c,k['y0']),(h+c,k['y0']-0.02)])
    return None
def keel_outline():
    k=KEEL; h=k['slot']/2; c=k['chamfer']
    za=k['z0']-T/2-k['tab']; zb_=k['z1']+T/2+k['tab']
    # curvy keel: lower edge sags like a spine between the legs
    xs=np.linspace(za,zb_,60); mid=(za+zb_)/2; half=(zb_-za)/2
    bot=[(x,k['y0']-0.0*0) for x in xs]
    g=Polygon([(za,k['y1'])]+[(x,k['y0']+0.012*((x-mid)/half)**2) for x in xs]+[(zb_,k['y1'])])
    g=close(opn(g,0.006),0.006)
    cuts=[]
    for z in Z:
        if k['z0']<z<k['z1']:
            cuts.append(Polygon([(z-h,k['ymid']),(z+h,k['ymid']),(z+h,k['y1']-c),(z+h+c,k['y1']),(z+h+c,k['y1']+0.01),(z-h-c,k['y1']+0.01),(z-h-c,k['y1']),(z-h,k['y1']-c)]))
    for zc in (za+0.010,zb_-0.010): cuts.append(Point(zc,(k['y0']+k['y1'])/2).buffer(0.0033,quad_segs=16))
    return g.difference(U(cuts))
pending=[]
def make_rib(z):
    d=base.get(z,dict(detail=[],mass=[]))
    env=envelope(z); envb=env.buffer(0.004)
    det=U(d['detail']); mass=U(d['mass'])
    g0=U([det,mass])
    st=structure(z,g0)
    massc=hollow(U([mass,st.intersection(mass.buffer(0.01)) if False else Polygon()]).difference(env))
    g=U([massc,det.difference(env),st.difference(env)])
    # contact skin: a curved rib-like band following the carved (bucket) edge wherever a ring is cut,
    # so back / headrest contact is continuous across the ring (lumbar support)
    if z<=-0.20 and not env.is_empty:
        hulls=[]
        for zone in (box(-0.26,YSR+0.08,0.26,SH_Y+0.02),box(-0.26,SH_Y+0.02,0.26,2)):
            part=U([c for c in geoms(det.intersection(zone)) if c.area>2e-4])
            if not part.is_empty and part.intersects(env) and (part.bounds[2]-part.bounds[0])>0.08: hulls.append(part.convex_hull)
        if hulls:
            skin=env.buffer(0.020,join_style=1).difference(env).intersection(U(hulls).buffer(0.004)).intersection(box(-1,YSR+0.08,1,2))
            g=U([g,skin])
    rs=rods_at(z)
    bosses=[Point(r['x'],r['y']).buffer(boss_r(r['d'])) for r in rs]
    bz=U(bosses)
    g=U([g,bz])
    g=U([c for c in geoms(g) if c.area>1.2e-4 or c.bounds[1]<0.01 or c.intersects(bz)])
    # connectivity: short links then MST; components holding >=2 rods are anchored
    def link(a,b):
        pa,pb=nearest_points(a,b); va=np.array(pb.coords[0])-np.array(pa.coords[0]); n=np.linalg.norm(va)
        if n<1e-6: return None
        va/=n; return LineString([np.array(pa.coords[0])-va*0.012,np.array(pb.coords[0])+va*0.012]).buffer(0.009,cap_style=1)
    comps=geoms(g); add=[]
    for i in range(len(comps)):
        for j in range(i+1,len(comps)):
            if comps[i].distance(comps[j])<0.03:
                s=link(comps[i],comps[j])
                if s is not None and not s.intersects(envb): add.append(s)
    g=U([g]+add); comps=geoms(g)
    if len(comps)>1:
        n=len(comps); par=list(range(n))
        cnt=[sum(comps[i].buffer(1e-4).contains(Point(r['x'],r['y'])) for r in rs) for i in range(n)]
        def f(a):
            while par[a]!=a: par[a]=par[par[a]]; a=par[a]
            return a
        E=[]
        for i in range(n):
            for j in range(i+1,n):
                dd=comps[i].distance(comps[j]); s=link(comps[i],comps[j])
                if s is None: continue
                if s.intersects(envb): dd+=10
                E.append((dd,i,j,s))
        E.sort(key=lambda e:e[0]); st2=[]; drop=set()
        for dd,i,j,s in E:
            a,b2=f(i),f(j)
            if a==b2 or (cnt[a]>=2 and cnt[b2]>=2): continue
            small=[k for k,gk in ((i,a),(j,b2)) if cnt[gk]==0 and comps[k].area<0.003]
            if (dd>0.07) and small: drop.update(small); continue
            if dd>=10: pending.append((float(z),i,j)); continue
            st2.append(s); par[a]=b2; cnt[b2]+=cnt[a]
        drop.update(k for k in range(n) if cnt[f(k)]==0)
        g=U([c for k,c in enumerate(comps) if k not in drop]+st2)
    # organic fillets at every junction, then re-carve the sitter
    g=close(g,0.010).difference(env)
    g=U([g,bz])
    holes=[Point(r['x'],r['y']).buffer(hole_d(r['d'])/2,quad_segs=16) for r in rs]
    if holes: g=g.difference(U(holes))
    kc=keel_cut(z)
    if kc is not None: g=g.difference(kc)
    for r in R:
        if r in rs: continue
        if (abs(r['z1']-round(z-P,4))<1e-6 and 'pk1' in r) or (abs(r['z0']-round(z+P,4))<1e-6 and 'pk0' in r):
            g=g.difference(Point(r['x'],r['y']).buffer(NUT_R[r['d']]+0.0025))
    g2=opn(g,0.0035).union(g.intersection(bz))
    g=U([c for c in geoms(g2) if c.area>6e-5 or c.intersects(bz)])
    if holes: g=g.difference(U(holes))
    if kc is not None: g=g.difference(kc)
    g=close(g,0.0015).intersection(box(-1,0,1,2)).simplify(0.0002,preserve_topology=True)
    return g,rs
def build_all(maxit=10):
    for it in range(maxit):
        ribs={}
        for z in Z:
            g,rs=make_rib(z); ribs[z]=(g,[r['name'] for r in rs])
        ch=False
        for r in R:
            for side in (-1,1):
                end=r['z0'] if side<0 else r['z1']; nb=round(end+side*P,4)
                if ('pk0' if side<0 else 'pk1') in r: continue
                while nb in ribs and not ribs[nb][0].is_empty and ribs[nb][0].intersects(Point(r['x'],r['y']).buffer(NUT_R[r['d']]+0.002)):
                    ch=True
                    if envelope(nb).intersects(Point(r['x'],r['y']).buffer(boss_r(r['d'])+0.003)): r['pk0' if side<0 else 'pk1']=True; break
                    elif side<0: r['z0']=nb
                    else: r['z1']=nb
                    nb=round(nb+side*P,4)
        print('iter',it,ch)
        if not ch: return ribs
    return ribs
def check_rods():
    return [(r['name'],z) for r in R for z in Z if r['z0']-1e-6<=z<=r['z1']+1e-6 and envelope(z).intersects(Point(r['x'],r['y']).buffer(boss_r(r['d'])+0.003))]
if __name__=='__main__':
    print('rod/envelope conflicts:',check_rods())
    ribs=build_all()
    pending.clear(); [make_rib(z) for z in Z]
    print('pending',pending)
    pickle.dump(dict(ribs=ribs,R=R,Z=Z,T=T,KEEL=KEEL,keel=keel_outline()),open('ribs_final.pkl','wb'))
    print('area',sum(ribs[z][0].area for z in Z))
