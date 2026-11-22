from base import *
# ---------- assemble base (curvy original) shapes per new z ----------
base={}   # z -> dict(detail=, mass=)   mass: regions filled then hollowed after carving
def put(z,kind,g):
    z=round(z,4); d=base.setdefault(z,dict(detail=[],mass=[]))
    if not g.is_empty: d[kind].append(g)
for zo in sorted(S):
    u=S[zo]
    if u is None or u.is_empty: continue
    if zo<=-0.175+1e-6:      # upper body slices
        head=u.intersection(box(-1,HEAD_Y,1,2)); body=u.difference(box(-1,HEAD_Y,1,2))
        zh=zo-M_HEAD*P
        if not head.is_empty:
            put(zh,'detail',tr(thick(head),1.0))
            put(zh,'mass',tr(fill(close(head.buffer(0.004),0.02)),1.0))
        put(zo,'detail',tr(thick(body),SX_UP))
        cage=body.intersection(box(-0.24,0.52,0.24,1.0))
        big=[c for c in geoms(close(cage.buffer(0.003),0.02)) if (c.bounds[2]-c.bounds[0])>0.12 and abs(c.centroid.x)<0.12]
        if big and zo>=-0.40: put(zo,'mass',tr(fill(U(big)),SX_UP))
        if zo>=-0.25:   # hip balls
            hip=body.intersection(box(-0.25,0.4,0.25,0.6))
            for s in (box(-1,0,0,2),box(0,0,1,2)):
                hh=hip.intersection(s)
                if hh.area>2e-4: put(zo,'mass',tr(hh.convex_hull.buffer(0.004),SX_UP))
    else:
        put(zo,'detail',tr(thick(u),SX_LOW))
        if zo<=0.15:   # thigh rod-rings -> filled discs (hollowed after carving => ring), beads kept as detail
            for s in (box(-1,0,0,2),box(0,0,1,2)):
                hh=u.intersection(s).intersection(box(-1,0.3,1,1))
                if not hh.is_empty:
                    c=hh.convex_hull; put(zo,'mass',tr(c.buffer(0.006),SX_LOW))
        elif zo<=0.25:  # knees
            kn=u.intersection(box(-1,0.46,1,1))
            for s in (box(-1,0,0,2),box(0,0,1,2)):
                hh=kn.intersection(s)
                if hh.area>1e-4: put(zo,'mass',tr(hh.convex_hull.buffer(0.004),SX_LOW))
Z=sorted(base)
def thigh_centers(z):
    out=[]
    for m in base[z]['mass']:
        for p in geoms(m):
            if p.centroid.y>0.3 and p.centroid.y<0.5: out.append((p.centroid.x,p.centroid.y,np.sqrt(p.area/np.pi)))
    return sorted(out)
if __name__=='__main__':
    print(len(Z),Z[0],Z[-1],'ZBS',ZBS,'YSR',YSR,'SH',SH_Y)
    for z in (-0.1,0.0,0.1): print(z,thigh_centers(z))
