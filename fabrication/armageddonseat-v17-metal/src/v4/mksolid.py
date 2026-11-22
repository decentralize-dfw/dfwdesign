import numpy as np, sys
sys.path.insert(0,'../v3')
from vol import raster, sdf, geoms, U, close, fill, S, ZS
from shapely.geometry import box
F=np.zeros((len(ZS),)+raster(S[ZS[0]]).shape,np.float32)
for k,zo in enumerate(ZS):
    u=S[zo]
    if u is None or u.is_empty: F[k]=0.5; continue
    parts=[]
    # torso/head: big closing so open rib rings become solid sections
    core=u.intersection(box(-0.245,0.40,0.245,1.40))
    parts.append(fill(close(core.buffer(0.004),0.045)))
    rest=u.difference(box(-0.245,0.40,0.245,1.40))
    parts.append(fill(close(rest.buffer(0.004),0.02)))
    F[k]=sdf(raster(U(parts)))
np.savez_compressed('solid.npz',F=F)
print('ok')
