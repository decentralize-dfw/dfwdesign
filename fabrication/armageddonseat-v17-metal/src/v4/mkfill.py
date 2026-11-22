# filled-slice SDF volume (each original slice with its holes closed) for solid-based variants
import numpy as np, pickle, sys
sys.path.insert(0,'../v3')
from vol import raster, sdf, geoms, U, close, fill, S, ZS
F=np.zeros((len(ZS),)+raster(S[ZS[0]]).shape,np.float32)
for k,zo in enumerate(ZS):
    u=S[zo]
    F[k]=sdf(raster(fill(close(u.buffer(0.004),0.014)))) if u is not None and not u.is_empty else 0.5
np.savez_compressed('fill.npz',F=F)
print(F.shape)
