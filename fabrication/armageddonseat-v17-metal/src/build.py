import numpy as np, pickle, warnings; warnings.filterwarnings("ignore")
from shapely.geometry import Polygon, MultiPolygon, box, Point, LineString
from shapely.ops import unary_union, nearest_points
from ribs import raw, Z, envelope, ys, zb, geoms, P, ZF, ZBS, YSR, SH_Y
T=0.008                       # plate thickness
SPACER=P-T                    # 17 mm
# ---------------- rods / bolts -----------------
# (name, x, y, z0, z1, d_nominal)
R=[]
def rod(n,x,y,z0,z1,d): R.append(dict(name=n,x=x,y=y,z0=round(z0,4),z1=round(z1,4),d=d))
ZR=-0.425; ZK=0.30      # rear girdle rib / front knee rib
rod('L1',-0.10,0.350,ZR,ZK,12); rod('L2',0.10,0.350,ZR,ZK,12)
rod('L3',-0.175,0.350,ZR,ZK,12); rod('L4',0.175,0.350,ZR,ZK,12)
rod('L6',0.0,0.372,-0.175,0.225,12)
rod('B1',-0.165,0.62,ZR,-0.225,10); rod('B2',0.165,0.62,ZR,-0.225,10)
rod('B3',-0.16,0.86,ZR,-0.325,10); rod('B4',0.16,0.86,ZR,-0.325,10)
rod('B5',0.0,0.93,ZR,-0.35,10)
rod('H1',0.0,1.035,-0.475,-0.40,10); rod('H2',0.0,1.10,-0.475,-0.425,10)
# feet / hands
for s in (-1,1):
    rod(f'F{"L" if s<0 else "R"}a',s*0.118,0.022,0.30,0.475,8); rod(f'F{"L" if s<0 else "R"}b',s*0.152,0.022,0.30,0.45,8)
    rod(f'E{"L" if s<0 else "R"}a',s*0.318,0.024,-0.425,-0.30,8); rod(f'E{"L" if s<0 else "R"}b',s*0.350,0.024,-0.425,-0.30,8)
# shin lacing bolts (front legs) and arm lacing bolts (rear legs) - visible bolt rows
for s in (-1,1):
    for k,y in enumerate((0.13,0.22,0.30)):
        rod(f'S{"L" if s<0 else "R"}{k+1}',s*0.135,y,0.275,0.35 if y<0.3 else 0.325,8)
ARM_PTS=[]
def _arm_pts():
    from ribs import raw as _raw
    parts=[p for p,t in _raw[-0.4]]
    from shapely.ops import unary_union as uu
    g=uu(parts)
    out=[]
    for y in (0.16,0.27,0.48,0.58,0.68):
        L=g.intersection(box(0.2,y-0.001,0.45,y+0.001))
        if L.is_empty: continue
        xs=[c.centroid.x for c in geoms(L)]; out.append((float(np.mean(xs)),y))
    return out
for k,(x,y) in enumerate(_arm_pts()):
    for s in (-1,1): rod(f'A{"L" if s<0 else "R"}{k+1}',s*x,y,-0.425,-0.375,8)
# ------------- keel (interlocking spine under the seat) -------------
KEEL=dict(x=0.0,y0=0.240,y1=0.330,ymid=0.295,z0=ZR,z1=ZK,tab=0.020,slot=T+0.0006,chamfer=0.0015)
def keel_cut(z):
    k=KEEL; h=k['slot']/2
    if abs(z-k['z0'])<1e-6 or abs(z-k['z1'])<1e-6:      # end ribs: closed window, keel tab passes through
        return box(-h,k['y0']-0.0005,h,k['y1']+0.0005)
    if k['z0']<z<k['z1']:                                  # interior ribs: open slot from fin bottom (half-lap)
        c=k['chamfer']
        return Polygon([(-h-c,k['y0']-0.02),(-h-c,k['y0']),(-h,k['y0']+c),(-h,k['ymid']),(h,k['ymid']),(h,k['y0']+c),(h+c,k['y0']),(h+c,k['y0']-0.02)])
    return None
def keel_outline():
    k=KEEL; h=k['slot']/2; c=k['chamfer']
    za=k['z0']-T/2-k['tab']; zb_=k['z1']+T/2+k['tab']
    g=box(za,k['y0'],zb_,k['y1']).buffer(-0.006,join_style=1).buffer(0.006,join_style=1)
    cuts=[]
    from ribs import Z as _Z
    for z in _Z:
        if k['z0']<z<k['z1']:
            cuts.append(Polygon([(z-h,k['ymid']),(z+h,k['ymid']),(z+h,k['y1']-c),(z+h+c,k['y1']),(z+h+c,k['y1']+0.01),(z-h-c,k['y1']+0.01),(z-h-c,k['y1']),(z-h,k['y1']-c)]))
    for zc in (za+0.010,zb_-0.010): cuts.append(Point(zc,(k['y0']+k['y1'])/2).buffer(0.0033,quad_segs=16))  # M6 lock-bolt holes
    return g.difference(unary_union(cuts))
# ------------- base content -------------
def arm_x(rawparts,y):
    pass
def _cage(z):
    parts=[p for p,t in raw.get(z,[])]
    if not parts: return None
    cage=unary_union(parts).intersection(box(-0.24,0.52,0.24,0.98))
    big=[c for c in geoms(cage) if (c.area>0.0005 or c.length>0.4) and abs(c.centroid.x)<0.12 and (c.bounds[2]-c.bounds[0])>0.12]
    return unary_union(big).convex_hull if big else None
def cage_hull(z):
    h=_cage(z); a=round(z-P,4); c=round(z+P,4)
    nb=[_cage(a),_cage(c)]
    if all(n is not None for n in nb) and (h is None or h.area<0.6*np.mean([n.area for n in nb])):
        return nb[0].intersection(nb[1])
    return h
def content(z):
    parts=[p for p,t in raw.get(z,[])]
    if z>-0.29 and z<-0.2: parts=[p for p in parts if not (p.bounds[3]<0.12 and abs(p.centroid.x)>0.25)]
    g=unary_union(parts) if parts else Polygon()
    add=[]
    # torso ribcage -> solid hull (will be hollowed) for torso ribs
    if -0.40<=z<=-0.20:
        h=cage_hull(z)
        if h is not None: add.append(h)
    # arms reach the floor (rear legs)
    if -0.425<=z<=-0.375:
        for s in (-1,1):
            arm=g.intersection(box(min(0,s*1),0,max(0,s*1),0.6)).intersection(box(-1,0,1,0.6))
            arm=unary_union([a for a in geoms(arm) if abs(a.centroid.x)>0.24])
            if arm.is_empty: continue
            low=arm.intersection(box(-1,0,1,arm.bounds[1]+0.06))
            xc=low.centroid.x
            add.append(unary_union([low,box(xc-0.022,0,xc+0.022,0.03)]).convex_hull)
            add.append(arm.buffer(0.004))
        # rear girdle: arm-to-arm band under the seat (rear seat frame)
        add.append(box(-0.33,0.326,0.33,0.394).buffer(0.012).buffer(-0.012))
    # pelvis band for torso ribs between girdle and lap
    if -0.375<=z<=-0.20:
        from shapely import affinity
        add.append(affinity.scale(Point(0,0.358).buffer(1.0,quad_segs=32),0.225,0.046))
    # seat pan: bridge between thighs + central keel fin
    if -0.425<=z<=0.30:
        top=ys(z)
        yb=KEEL['y0']-(0.025 if abs(z-KEEL['z0'])<1e-6 or abs(z-KEEL['z1'])<1e-6 else 0.0)
        fin=Polygon([(-0.05,min(top,0.40)-0.03),(0.05,min(top,0.40)-0.03),(0.026,yb),(-0.026,yb)])
        add.append(fin.buffer(0.004,join_style=1).buffer(-0.004,join_style=1))
    if -0.20<=z<=0.25:
        top=ys(z)
        add.append(box(-0.15,top-0.05,0.15,top-0.007))
    # knees: inner bar connecting both knees (front seat frame)
    if 0.275<=z<=0.30:
        add.append(box(-0.17,0.262,0.17,0.31).buffer(0.01).buffer(-0.01))
    return unary_union([g]+add)
def hollow(g,band=0.026):
    out=[]
    for c in geoms(g):
        inner=c.buffer(-band,join_style=1)
        inner=inner.buffer(-0.012,join_style=1).buffer(0.012,join_style=1)
        wins=[w for w in geoms(inner) if w.area>0.0008]
        out.append(c.difference(unary_union(wins)) if wins else c)
    return unary_union(out)
def rods_at(z): return [r for r in R if r['z0']-1e-6<=z<=r['z1']+1e-6]
def boss_r(d): return {12:0.019,10:0.017,8:0.012}[d]
def hole_d(d): return d/1000+0.0008
pending=[]
def make_rib(z):
    g=content(z)
    env=envelope(z)
    g=g.difference(env)
    g=hollow(g)
    rs=rods_at(z)
    bosses=[Point(r['x'],r['y']).buffer(boss_r(r['d'])) for r in rs]
    g=unary_union([g]+bosses)
    g=g.buffer(-0.003,join_style=1).buffer(0.003,join_style=1)   # remove slivers
    g=unary_union([c for c in geoms(g) if c.area>4e-5]+bosses)
    # connectivity (struts) - components holding >=2 rods are anchored
    envb=env.buffer(0.006)
    comps=geoms(g); short=[]
    for i in range(len(comps)):
        for j in range(i+1,len(comps)):
            a,b_=nearest_points(comps[i],comps[j])
            if a.distance(b_)<0.035:
                L=LineString([a,b_]) if a.distance(b_)>1e-6 else None
                if L is None or not L.intersects(envb):
                    va=np.array(b_.coords[0])-np.array(a.coords[0]); n_=np.linalg.norm(va)
                    if n_<1e-6: continue
                    va/=n_; short.append(LineString([np.array(a.coords[0])-va*0.01,np.array(b_.coords[0])+va*0.01]).buffer(0.012,cap_style=1))
    if short:
        g=unary_union([g]+short).difference(env); g=unary_union([g]+bosses)
    comps=geoms(g)
    if len(comps)>1:
        n=len(comps); anchored=[sum(c.buffer(1e-4).contains(Point(r['x'],r['y'])) for r in rs)>=2 for c in comps]
        parent=list(range(n))
        def f(a):
            while parent[a]!=a: parent[a]=parent[parent[a]]; a=parent[a]
            return a
        an=[i for i in range(n) if anchored[i]]
        for i in an[1:]: parent[f(i)]=f(an[0])
        edges=[]
        envb=env.buffer(0.006)
        for i in range(n):
            for j in range(i+1,n):
                a,b=nearest_points(comps[i],comps[j]); d=a.distance(b)
                L=LineString([a,b])
                if L.length>0 and L.intersects(envb): d+=10
                edges.append((d,i,j,a,b))
        edges.sort(key=lambda e:e[0])
        struts=[]
        for d,i,j,a,b in edges:
            if f(i)!=f(j):
                parent[f(i)]=f(j)
                if d>=10: pending.append((z,i,j)); continue
                # strut from slightly inside each component
                va=np.array(b.coords[0])-np.array(a.coords[0]); L=np.linalg.norm(va)+1e-9; va/=L
                pa=np.array(a.coords[0])-va*0.01; pb=np.array(b.coords[0])+va*0.01
                struts.append(LineString([pa,pb]).buffer(0.012,cap_style=1))
        g=unary_union([g]+struts).difference(env)
        g=unary_union([g]+bosses)
    holes=[Point(r['x'],r['y']).buffer(hole_d(r['d'])/2,quad_segs=16) for r in rs]
    g=g.difference(unary_union(holes)) if holes else g
    kc=keel_cut(z)
    if kc is not None: g=g.difference(kc)
    # nut clearance pockets for rods that end at the neighbouring rib
    for r in R:
        if r in rs: continue
        if (abs(r['z1']-round(z-P,4))<1e-6 and 'pk1' in r) or (abs(r['z0']-round(z+P,4))<1e-6 and 'pk0' in r):
            g=g.difference(Point(r['x'],r['y']).buffer({12:0.0105,10:0.009,8:0.0075}[r['d']]+0.0025))
    # remove features thinner than ~7 mm (keep bosses intact)
    bz=unary_union(bosses) if bosses else Polygon()
    g2=g.buffer(-0.0035,join_style=1).buffer(0.0035,join_style=1).union(g.intersection(bz))
    keep=[c for c in geoms(g2) if c.area>6e-5 or any(c.buffer(1e-4).contains(Point(r['x'],r['y']).buffer(0.001)) for r in rs)]
    g=unary_union(keep)
    if holes: g=g.difference(unary_union(holes))
    # final edge smoothing (no sharp corners; kerf friendly)
    g=g.buffer(0.0015,join_style=1).buffer(-0.0015,join_style=1)
    g=g.intersection(box(-1,0,1,2)).simplify(0.00025,preserve_topology=True)
    return g, rs
def check_rods():
    bad=[]
    for r in R:
        for z in Z:
            if r['z0']-1e-6<=z<=r['z1']+1e-6:
                if envelope(z).intersects(Point(r['x'],r['y']).buffer(boss_r(r['d'])+0.003)): bad.append((r['name'],z))
    return bad
NUT_R={12:0.0105,10:0.009,8:0.0075}
def build_all(maxit=6):
    for it in range(maxit):
        ribs={}
        for z in Z:
            g,rs=make_rib(z); ribs[z]=(g,[r['name'] for r in rs])
        changed=False
        for r in R:
            for side in (-1,1):
                end=r['z0'] if side<0 else r['z1']
                nb=round(end+side*P,4)
                if nb in ribs and not ribs[nb][0].is_empty and ribs[nb][0].intersects(Point(r['x'],r['y']).buffer(NUT_R[r['d']]+0.002)):
                    if envelope(nb).intersects(Point(r['x'],r['y']).buffer(boss_r(r['d'])+0.003)):
                        r['pk0' if side<0 else 'pk1']=True; changed=True; continue
                    if side<0: r['z0']=nb
                    else: r['z1']=nb
                    changed=True
        print('iteration',it,'changed',changed)
        if not changed: return ribs
    return ribs
if __name__=='__main__':
    print('rod conflicts:',check_rods())
    ribs=build_all()
    print('pending (could not strut):',pending)
    pickle.dump(dict(ribs=ribs,R=R,Z=Z,T=T,KEEL=KEEL,keel=keel_outline()),open('ribs_final.pkl','wb'))
    for z in Z: print(z, round(ribs[z][0].area,4), len(geoms(ribs[z][0])), ribs[z][1])
