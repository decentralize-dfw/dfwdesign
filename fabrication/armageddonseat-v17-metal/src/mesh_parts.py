import pickle,json,numpy as np,trimesh,warnings;warnings.filterwarnings("ignore")
from base import geoms
d=pickle.load(open('ribs_final.pkl','rb'));ribs=d['ribs'];R=d['R'];Z=d['Z'];T=d['T'];K=d['KEEL'];kp=d['keel']
P=0.025
scene=trimesh.Scene(); meta=dict(parts=[],rods=[],keel={})
def mat(name,rgba,metal=0.85,rough=0.38):
    return trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(name=name,baseColorFactor=rgba,metallicFactor=metal,roughnessFactor=rough))
M_PLATE=[158,164,170,255]; M_KEEL=[120,150,190,255]; M_ROD=[205,205,210,255]; M_SP=[120,122,128,255]; M_NUT=[220,220,225,255]
def add(name,mesh,rgba,info):
    mesh.visual=mat(name.split('_')[0],rgba); scene.add_geometry(mesh,geom_name=name,node_name=name); info['name']=name; meta['parts'].append(info)
nz=[z for z in Z if not ribs[z][0].is_empty]
for i,z in enumerate(nz):
    comps=sorted(geoms(ribs[z][0]),key=lambda c:-c.area)
    for k,c in enumerate(comps):
        m=trimesh.creation.extrude_polygon(c,T); m.apply_translation([0,0,z-T/2])
        add(f'R{i+1:02d}'+('' if len(comps)==1 else 'abcdefgh'[k]),m,M_PLATE,dict(type='rib',z=float(z)))
km=trimesh.creation.extrude_polygon(kp,T); v=km.vertices.copy(); km.vertices=np.c_[v[:,2]-T/2,v[:,1],v[:,0]]; km.fix_normals()
add('K01',km,M_KEEL,dict(type='keel'))
yk=(K['y0']+K['y1'])/2
for j,zc in enumerate((kp.bounds[0]+0.010,kp.bounds[2]-0.010)):
    parts=[trimesh.creation.cylinder(radius=0.003,height=0.03,sections=16)]
    parts[0].apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[0,1,0]))
    for sx in (-1,1):
        h=trimesh.creation.cylinder(radius=0.0055,height=0.005,sections=6); h.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[0,1,0])); h.apply_translation([sx*0.0065,0,0]); parts.append(h)
    b=trimesh.util.concatenate(parts); b.apply_translation([0,yk,zc])
    add(f'KB{j+1}',b,M_NUT,dict(type='keelbolt',z=float(zc),side=-1 if j==0 else 1))
meta['keel']=dict(y0=K['y0'],y1=K['y1'],ymid=K['ymid'],z0=K['z0'],z1=K['z1'],slots=[float(z) for z in Z if K['z0']<z<K['z1']])
OD={12:0.020,10:0.017,8:0.014}
for r in R:
    zs=[z for z in Z if r['z0']-1e-6<=z<=r['z1']+1e-6]
    if not zs: continue
    dd=r['d']/1000; za,zb=min(zs)-T/2-0.002-0.8*dd, max(zs)+T/2+0.002+0.8*dd
    cyl=trimesh.creation.cylinder(radius=dd/2,height=zb-za,sections=20); cyl.apply_translation([r['x'],r['y'],(za+zb)/2])
    direction=1 if r['name'][0] in 'LSF' else -1
    add(f"ROD_{r['name']}",cyl,M_ROD,dict(type='rod',rod=r['name']))
    meta['rods'].append(dict(name=r['name'],x=r['x'],y=r['y'],z0=float(min(zs)),z1=float(max(zs)),za=float(za),zb=float(zb),d=r['d'],dir=direction))
    for a,b in zip(zs[:-1],zs[1:]):
        sp=trimesh.creation.annulus(r_min=dd/2+0.0003,r_max=OD[r['d']]/2,height=(b-a)-T,sections=20); sp.apply_translation([r['x'],r['y'],(a+b)/2])
        add(f"SP_{r['name']}_{int(round((a+0.7)*1000))}",sp,M_SP,dict(type='spacer',rod=r['name'],za=float(a),zb=float(b)))
    for zz,sg in ((min(zs)-T/2,-1),(max(zs)+T/2,1)):
        w=trimesh.creation.cylinder(radius=dd*1.0,height=0.002,sections=24); w.apply_translation([0,0,sg*0.001])
        h=trimesh.creation.cylinder(radius=dd*0.95,height=0.8*dd,sections=6); h.apply_translation([0,0,sg*(0.002+0.4*dd)])
        dome=trimesh.creation.icosphere(subdivisions=2,radius=dd*0.8); dome.vertices[:,2]*=0.8; dome.apply_translation([0,0,sg*(0.002+0.8*dd)])
        nut=trimesh.util.concatenate([w,h,dome]); nut.apply_translation([r['x'],r['y'],zz])
        add(f"NUT_{r['name']}_{'a' if sg<0 else 'b'}",nut,M_NUT,dict(type='nut',rod=r['name'],end=sg,z=float(zz)))
scene.export('chair_parts.glb')
json.dump(meta,open('chair_parts.json','w'))
print(len(meta['parts']),'parts', len(meta['rods']),'rods')
