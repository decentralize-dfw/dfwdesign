# check every bolt against every plate it is not meant to pass through
import json,sys,trimesh,numpy as np
v=sys.argv[1]; m=json.load(open(f'out/{v}/meta.json'))
sc=trimesh.load(f'out/{v}/model.glb')
geo={n:sc.geometry[n] for n in sc.geometry}
hosts={}
for j in m['joints']:
    if j['k']=='axis': hosts.setdefault(j['b'],set()).add(j['a'])
bad=[]
for b in [n for n in geo if n.startswith('BOLT')]:
    bm=geo[b]
    # shank only: thin cylinder samples along the bolt axis
    ext=bm.bounds; ax=np.argmax(ext[1]-ext[0]); c=(ext[0]+ext[1])/2
    pts=[]
    for t in np.linspace(ext[0][ax]+0.03,ext[1][ax]-0.03,200):
        p=c.copy(); p[ax]=t; pts.append(p)
    pts=np.array(pts)
    for n,g in geo.items():
        if n==b or n.startswith('BOLT') or n in hosts.get(b,()): continue
        if not g.is_watertight: continue
        inside=g.contains(pts)
        if inside.any(): bad.append((b,n,int(inside.sum())))
print(v,'clashes:',bad)
