import trimesh,numpy as np,matplotlib,pickle
matplotlib.use('Agg');import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from shapely.ops import unary_union
m=trimesh.load('seat.stl');m.merge_vertices()
n=m.face_normals;tc=m.triangles_center;T=m.triangles
zs=np.round(np.arange(-0.425,0.43,0.025),4)
mask=np.abs(n[:,2])>0.95
fig,axs=plt.subplots(5,7,figsize=(28,26));out={}
other=~mask
print('non-z faces z-range hist', np.histogram(tc[other][:,2],bins=20))
for i,z in enumerate(zs):
    sel=mask&(np.abs(tc[:,2]-z)<0.0125)
    tris=[Polygon(t[:,:2]) for t in T[sel]]
    tris=[t.buffer(1e-5) for t in tris if t.area>1e-12]
    u=unary_union(tris).buffer(-1e-5) if tris else None
    out[z]=u;ax=axs.flat[i]
    if u is not None and not u.is_empty:
        for g in getattr(u,'geoms',[u]):
            x,y=g.exterior.xy;ax.fill(x,y,alpha=.6)
            for h in g.interiors: x,y=h.xy;ax.fill(x,y,color='w')
    ax.set_xlim(-.4,.4);ax.set_ylim(0,1.32);ax.set_aspect('equal');ax.set_title(f'z={z} A={u.area if u else 0:.4f} n={sel.sum()}')
plt.tight_layout();plt.savefig('sections.png',dpi=40)
pickle.dump(out,open('sections.pkl','wb'))
