# Orijinal (Draco sıkıştırmalı) GLB'yi çözüp seat.stl üretir.
# Gerekenler: pip install trimesh DracoPy numpy shapely ezdxf matplotlib mapbox-earcut networkx
import trimesh, os
here = os.path.dirname(os.path.abspath(__file__))
m = trimesh.load(os.path.join(here, '..', '..', '..', 'armageddonseat-v17.glb')).to_geometry()
m.export(os.path.join(here, 'seat.stl'))
print(m.extents)
