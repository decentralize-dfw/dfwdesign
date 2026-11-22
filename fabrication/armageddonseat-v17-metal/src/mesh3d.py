import pickle,numpy as np,trimesh,warnings,os,sys;warnings.filterwarnings("ignore")
EX=float(os.environ.get('EXPLODE','0'))
from shapely.geometry import Polygon
from ribs import geoms
d=pickle.load(open('ribs_final.pkl','rb'));ribs=d['ribs'];R=d['R'];Z=d['Z'];T=d['T']
scene=trimesh.Scene()
plate=[];rods=[];spac=[];nuts=[]
def place(m,z0): m.apply_translation([0,0,z0]); return m
for z in Z:
    g,_=ribs[z]
    for p in geoms(g):
        m=trimesh.creation.extrude_polygon(p,T)
        plate.append(place(m,z*(1+EX)-T/2))
for r in (R if EX==0 else []):
    zs=[z for z in Z if r['z0']-1e-6<=z<=r['z1']+1e-6]
    if not zs: continue
    d_=r['d']/1000
    za,zb_=min(zs)-T/2-0.002-0.6*d_, max(zs)+T/2+0.002+0.6*d_   # rod incl. washer+nut length
    c=trimesh.creation.cylinder(radius=d_/2,height=zb_-za,sections=20); c.apply_translation([r['x'],r['y'],(za+zb_)/2]); rods.append(c)
    od={12:0.020,10:0.017,8:0.014}[r['d']]
    for a,b in zip(zs[:-1],zs[1:]):
        s=trimesh.creation.annulus(r_min=d_/2+0.0003,r_max=od/2,height=(b-a)-T,sections=24); s.apply_translation([r['x'],r['y'],(a+b)/2]); spac.append(s)
    # acorn nuts: hex + dome
    for zz,sg in ((min(zs)-T/2,-1),(max(zs)+T/2,1)):
        w=trimesh.creation.cylinder(radius=d_*1.0,height=0.002,sections=24); w.apply_translation([r['x'],r['y'],zz+sg*0.001])
        h=trimesh.creation.cylinder(radius=d_*0.95,height=0.8*d_,sections=6); h.apply_translation([r['x'],r['y'],zz+sg*(0.002+0.4*d_)])
        dome=trimesh.creation.icosphere(subdivisions=2,radius=d_*0.8); dome.vertices[:,2]*=0.8; dome.apply_translation([r['x'],r['y'],zz+sg*(0.002+0.8*d_)])
        nuts+= [w,h,dome]
kp=d['keel']
km=trimesh.creation.extrude_polygon(kp,T)   # polygon in (z,y) -> extrude along +Z, then map
v=km.vertices.copy(); km.vertices=np.c_[v[:,2]-T/2, v[:,1], v[:,0]]; km.fix_normals(); km.apply_translation([0,-1.6*EX,0]); plate.append(km)
for zc in (kp.bounds[0]+0.010,kp.bounds[2]-0.010):   # M6 lock bolts across keel tabs
    bb=trimesh.creation.cylinder(radius=0.003,height=0.03,sections=16); bb.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[0,1,0])); bb.apply_translation([0,0.285,zc]); rods.append(bb)
    for sx in (-1,1):
        hh=trimesh.creation.cylinder(radius=0.0055,height=0.005,sections=6); hh.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[0,1,0])); hh.apply_translation([sx*0.0065,0.285,zc]); nuts.append(hh)
def col(ms,rgba,name):
    m=trimesh.util.concatenate(ms); m.visual=trimesh.visual.ColorVisuals(m,face_colors=rgba)
    # PBR material
    m.visual=trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(name=name,baseColorFactor=rgba,metallicFactor=0.85,roughnessFactor=0.38))
    return m
# glTF: y-up; our coords already y-up, z front
scene.add_geometry(col(plate,[150,156,162,255],'plate'),geom_name='ribs')
if rods: scene.add_geometry(col(rods,[200,200,205,255],'rod'),geom_name='rods')
if spac: scene.add_geometry(col(spac,[110,112,118,255],'spacer'),geom_name='spacers')
if nuts: scene.add_geometry(col(nuts,[215,215,220,255],'nut'),geom_name='nuts')
scene.export('chair_exploded.glb' if EX else 'chair.glb')
allm=trimesh.util.concatenate(plate+rods+spac+nuts)
if EX: sys.exit()
print('bounds',allm.bounds.round(3),'extents',allm.extents.round(3))
print('plates',len(plate),'spacers',len(spac),'nuts',len(nuts)//3)
