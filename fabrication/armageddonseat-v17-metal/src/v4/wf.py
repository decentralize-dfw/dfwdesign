# Shared framework for the laser-cut-only variants (plates + big bolts, no tubes / extra hardware).
import numpy as np, json, warnings; warnings.filterwarnings("ignore")
from shapely.geometry import Polygon, MultiPolygon, box, Point, LineString
from shapely.ops import unary_union, nearest_points
from shapely import affinity
from scipy.ndimage import map_coordinates
import contourpy, trimesh, ezdxf

T=0.008; P0=0.025
_V=np.load('../v3/vol.npz'); D=_V['D']; M=_V['M']; ZS=_V['ZS']; RES,X0,X1,Y0,Y1=_V['grid']
FD=np.load('fill.npz')['F']
FS=np.load('solid.npz')['F']

def geoms(g):
    if g is None or g.is_empty: return []
    return [x for x in getattr(g,'geoms',[g]) if x.geom_type=='Polygon' and not x.is_empty]
def U(l): return unary_union([x for x in l if x is not None and not x.is_empty]) if l else Polygon()
def close(g,r): return g.buffer(r,join_style=1).buffer(-r,join_style=1)
def opn(g,r): return g.buffer(-r,join_style=1).buffer(r,join_style=1)

# ---------------- volume sampling ----------------
def _samp(F,x,y,z,blend):
    k=(z-ZS[0])/P0; i=(x-X0)/RES; j=(y-Y0)/RES
    k0=np.clip(np.floor(k),0,len(ZS)-1); k1=np.clip(k0+1,0,len(ZS)-1); t=np.clip(k-k0,0,1)
    if blend=='near':
        t=np.clip((t-0.35)/0.30,0,1); t=t*t*(3-2*t)
    v0=map_coordinates(F,[k0,j,i],order=1,mode='nearest'); v1=map_coordinates(F,[k1,j,i],order=1,mode='nearest')
    v=(1-t)*v0+t*v1
    out=(k<-0.45)|(k>len(ZS)-0.55)|(i<0)|(i>F.shape[2]-1)|(j<0)|(j>F.shape[1]-1)
    v[out]=0.5
    return v

class Figure:
    """Original figure, uniformly scaled by s, upper body reclined by phi about the hip, lap widened by kx."""
    def __init__(self,s=1.0,phi=12,kx=1.08,mode='detail',thick=0.005):
        self.s=s; self.phi=np.radians(phi); self.kx=kx; self.mode=mode; self.thick=thick
        self.YH,self.ZH=0.50,-0.25; self.ARM_TOP,self.ARM_Z=0.80,-0.38
        y1,z1=self.up_fwd(self.ARM_TOP,self.ARM_Z); self.ARM_DZ=z1-self.ARM_Z; self.ARM_SY=y1/self.ARM_TOP
    def up_fwd(self,y,z):
        c,s=np.cos(self.phi),np.sin(self.phi); yb=y-self.YH; zb=z-self.ZH
        return self.YH+zb*s+yb*c, self.ZH+zb*c-yb*s
    def up_inv(self,y,z):
        c,s=np.cos(self.phi),np.sin(self.phi); dz=z-self.ZH; dy=y-self.YH
        return self.YH+(-dz*s+dy*c), self.ZH+(dz*c+dy*s)
    def value(self,x,y,z):
        x=np.asarray(x,float)/self.s; y=np.asarray(y,float)/self.s; z=np.asarray(z,float)/self.s
        best=np.full(x.shape,0.5,np.float32)
        def samp(xo,yo,zo):
            if self.mode=='detail': return _samp(D,xo,yo,zo,'near')
            return np.minimum(_samp(FS,xo,yo,zo,'lin'),_samp(M,xo,yo,zo,'lin'))
        # lower body (thighs, shins, feet): lap widened
        xo=x/self.kx; ok=(y<self.YH)|(z>-0.19)
        xo=np.where((z>-0.19)&(y<0.62),x/self.kx,x)
        best=np.where(ok,np.minimum(best,samp(xo,y,z)),best)
        # upper body
        yo,zo=self.up_inv(y,z)
        ok=(yo>=self.YH)&(zo<=-0.19)&((np.abs(x)<0.245)|(yo>=self.ARM_TOP))
        best=np.where(ok,np.minimum(best,samp(x,yo,zo)),best)
        # arms (rear legs) follow the shoulder
        yo=y/self.ARM_SY; zo=z-self.ARM_DZ
        ok=(np.abs(x)>=0.245)&(yo<self.ARM_TOP)&(zo<=-0.19)
        best=np.where(ok,np.minimum(best,samp(x,yo,zo)),best)
        return best*self.s

# ---------------- sitter envelope (3D bucket) ----------------
class Envelope:
    def __init__(self,seat_y,zf,depth=0.40,tilt=4,recline=15,ks=0.30,kb=1.6,kh=4.0,lumbar=0.020,sacral=0.035,head_w=0.115):
        self.SY=seat_y; self.ZF=zf; self.ZBS=zf-depth; self.TILT=np.radians(tilt); self.REC=np.radians(recline)
        self.KS=ks; self.KB=kb; self.KH=kh; self.LUM=lumbar; self.SAC=sacral; self.HW=head_w
        self.YSR=float(self.ys(np.array([self.ZBS]))[0]); self.SH=self.YSR+0.58
    def ys(self,z):
        z=np.asarray(z,float); R=0.04
        y=self.SY-(self.ZF-np.minimum(z,self.ZF))*np.tan(self.TILT)
        d=np.clip(z-(self.ZF-R),0,R); y=y-np.where(z>self.ZF-R,R-np.sqrt(R*R-d*d),0)
        y=y-np.where(z>self.ZF,(z-self.ZF)*0.5,0)-np.where(z>self.ZF+0.13,(z-self.ZF-0.13)*6,0)
        return y
    def zb(self,y):
        y=np.asarray(y,float)
        z=self.ZBS-(y-self.YSR)*np.tan(self.REC)
        z=z+self.LUM*np.exp(-((y-(self.YSR+0.20))/0.07)**2)-self.SAC*np.exp(-((y-(self.YSR+0.03))/0.05)**2)
        return z
    def w(self,y):
        t=np.clip((y-(self.SH-0.04))/0.12,0,1); return 0.25+(self.HW-0.25)*(0.5-0.5*np.cos(np.pi*t))
    def kb(self,y): return self.KB+(self.KH-self.KB)*np.clip((y-self.SH)/0.1,0,1)
    def inside(self,x,y,z,margin=0.0):
        x=np.abs(np.asarray(x,float)); y=np.asarray(y,float); z=np.asarray(z,float)
        return (y>=0.08)&(y+margin>self.ys(z)+self.KS*x*x)&(z+margin>self.zb(y)+self.kb(y)*x*x)&(x<self.w(y)+margin)
    def pose(self):
        th=self.REC; H=[self.ZBS+0.125-0.09*np.tan(th),self.YSR+0.095]
        return dict(H=H,theta=float(np.degrees(th)),seatFront=[self.ZF,float(self.ys(np.array([self.ZF-0.03]))[0])],ysr=self.YSR,zbs=self.ZBS)

# ---------------- plane slicing ----------------
GU=np.arange(-0.62,0.62+1e-9,0.002); GV=np.arange(-0.004,1.45,0.002)
def _poly(U2,V2,F,level):
    cg=contourpy.contour_generator(U2,V2,F,fill_type=contourpy.FillType.OuterOffset)
    pts,offs=cg.filled(-10,level); out=[]
    for p,o in zip(pts,offs):
        rings=[p[o[k]:o[k+1]] for k in range(len(o)-1)]
        if len(rings[0])<4: continue
        out.append(Polygon(rings[0],[r for r in rings[1:] if len(r)>=4]).buffer(0))
    return U(out)
def plane_xyz(plane,pos,uu,vv):
    if plane=='z': return uu,vv,np.full_like(uu,pos)       # 2D coords (x,y)
    return np.full_like(uu,pos),vv,uu                        # plane 'x': 2D coords (z,y)
def slice_fig(fig,plane,pos,urange=(-0.62,0.62)):
    gu=GU[(GU>=urange[0])&(GU<=urange[1])]
    uu,vv=np.meshgrid(gu,GV); x,y,z=plane_xyz(plane,pos,uu.ravel(),vv.ravel())
    F=fig.value(x,y,z).reshape(uu.shape)
    g=_poly(gu,GV,F,fig.thick if fig.mode=='detail' else 0.0)
    return close(g,0.003).simplify(0.0006)
def slice_env(env,plane,pos,urange=(-0.62,0.62)):
    gu=GU[(GU>=urange[0])&(GU<=urange[1])]
    uu,vv=np.meshgrid(gu,GV); x,y,z=plane_xyz(plane,pos,uu.ravel(),vv.ravel())
    I=env.inside(x,y,z).reshape(uu.shape).astype(np.float32)
    if I.max()<0.5: return Polygon()
    g=_poly(gu,GV,-I,-0.5)
    return opn(g,0.008)
def hollow(g,band=0.022,keep=None,minwin=0.0007):
    out=[]
    for c in geoms(g):
        inner=opn(c.buffer(-band,join_style=1),0.010)
        if keep is not None: inner=inner.difference(keep)
        inner=opn(inner,0.008)
        wins=[w for w in geoms(inner) if w.area>minwin]
        out.append(c.difference(U(wins)) if wins else c)
    return U(out)

# ---------------- bolts ----------------
BOLT={16:dict(hole=0.0175,head_af=0.024,head_h=0.010,washer=0.030,washer_t=0.003,nut_h=0.013,boss=0.022)}
def bolt_hole(d=16): return Point(0,0).buffer(BOLT[d]['hole']/2,quad_segs=16)

# ---------------- connectivity helper ----------------
def connect(g,anchors,env2d,maxlen=0.09):
    """anchors: list of shapely geoms (slot zones / bolt bosses) that fix a component.
       Components touching an anchor are fixed; others are linked to the nearest fixed group or dropped."""
    comps=geoms(g)
    if len(comps)<=1: return g
    A=U(anchors) if anchors else Polygon()
    fixed=[c.intersects(A) for c in comps]
    envb=env2d.buffer(0.004) if not env2d.is_empty else Polygon()
    struts=[]; changed=True
    while changed:
        changed=False
        for i,c in enumerate(comps):
            if fixed[i]: continue
            best=None
            for j,d in enumerate(comps):
                if i==j or not fixed[j]: continue
                a,b=nearest_points(c,d); dist=a.distance(b)
                if dist>maxlen: continue
                v=np.array(b.coords[0])-np.array(a.coords[0]); n=np.linalg.norm(v)
                if n<1e-6: best=(0,None,j); break
                v/=n; s=LineString([np.array(a.coords[0])-v*0.012,np.array(b.coords[0])+v*0.012]).buffer(0.009,cap_style=1)
                if s.intersects(envb): continue
                if best is None or dist<best[0]: best=(dist,s,j)
            if best is not None:
                if best[1] is not None: struts.append(best[1])
                fixed[i]=True; changed=True
    keep=[c for i,c in enumerate(comps) if fixed[i]]
    return U(keep+struts)

# ---------------- export ----------------
def plate_mesh(p):
    ms=[]
    for c in geoms(p['poly']):
        m=trimesh.creation.extrude_polygon(c,T); m.apply_translation([0,0,-T/2])
        if p['plane']=='x':
            v=m.vertices.copy(); m.vertices=np.c_[v[:,2]+p['pos'],v[:,1],v[:,0]]; m.fix_normals()
        else: m.apply_translation([0,0,p['pos']])
        ms.append(m)
    return trimesh.util.concatenate(ms)
def bolt_mesh(b):
    d=b['d']; B=BOLT[d]; L=b['s1']-b['s0']
    parts=[]
    sh=trimesh.creation.cylinder(radius=d/2000,height=L+B['washer_t']*2+B['nut_h']+0.006,sections=24); sh.apply_translation([0,0,(b['s0']+b['s1'])/2+ (B['nut_h']+0.006)/2*b['nutside']]); parts.append(sh)
    def hexp(z,h): m=trimesh.creation.cylinder(radius=B['head_af']/np.sqrt(3),height=h,sections=6); m.apply_translation([0,0,z]); return m
    def wash(z): m=trimesh.creation.annulus(r_min=B['hole']/2,r_max=B['washer']/2,height=B['washer_t'],sections=32); m.apply_translation([0,0,z]); return m
    head_end=b['s0'] if b['nutside']>0 else b['s1']; nut_end=b['s1'] if b['nutside']>0 else b['s0']; sg=b['nutside']
    parts.append(wash(head_end-sg*B['washer_t']/2)); parts.append(hexp(head_end-sg*(B['washer_t']+B['head_h']/2),B['head_h']))
    parts.append(wash(nut_end+sg*B['washer_t']/2)); parts.append(hexp(nut_end+sg*(B['washer_t']+B['nut_h']/2),B['nut_h']))
    m=trimesh.util.concatenate(parts)
    if b['axis']=='x':
        v=m.vertices.copy(); m.vertices=np.c_[v[:,2],v[:,1]+b['v'],v[:,0]+b['u']]
    else:
        m.apply_translation([b['u'],b['v'],0])
    return m
def mat(name,rgba,metal=0.85,rough=0.38):
    return trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(name=name,baseColorFactor=rgba,metallicFactor=metal,roughnessFactor=rough))
def export(name,plates,bolts,joints,pose,steps,out_dir='out'):
    import os; os.makedirs(f'{out_dir}/{name}/dxf',exist_ok=True)
    sc=trimesh.Scene(); meta=dict(parts=[],joints=joints,pose=pose,steps=steps)
    for p in plates:
        m=plate_mesh(p); m.visual=mat('comb' if p.get('kind')=='comb' else 'plate',[128,136,146,255] if p.get('kind')=='comb' else [158,164,170,255])
        sc.add_geometry(m,geom_name=p['name'],node_name=p['name']); meta['parts'].append(dict(name=p['name'],moves=p.get('moves',[])))
    for b in bolts:
        m=bolt_mesh(b); m.visual=mat('bolt',[214,214,220,255],0.9,0.3)
        sc.add_geometry(m,geom_name=b['name'],node_name=b['name']); meta['parts'].append(dict(name=b['name'],moves=b.get('moves',[])))
    sc.export(f'{out_dir}/{name}/model.glb'); json.dump(meta,open(f'{out_dir}/{name}/meta.json','w'))
    import pickle; pickle.dump(dict(plates=[{k:p[k] for k in ('name','plane','pos','poly')} for p in plates],bolts=bolts),open(f'{out_dir}/{name}/plates.pkl','wb'))
    # DXF per plate + BOM
    area=0
    for p in plates:
        doc=ezdxf.new('R2010',setup=True); doc.units=ezdxf.units.MM
        for n,cl in (('CUT',1),('ETCH',3)): doc.layers.add(n,color=cl)
        msp=doc.modelspace(); g=affinity.scale(p['poly'],1000,1000,origin=(0,0))
        for c in geoms(g):
            msp.add_lwpolyline(list(c.exterior.coords),close=True,dxfattribs={'layer':'CUT'})
            for h in c.interiors: msp.add_lwpolyline(list(h.coords),close=True,dxfattribs={'layer':'CUT'})
            area+=c.area/1e6
        big=max(geoms(g),key=lambda q:q.area)
        from shapely.ops import polylabel
        lp=polylabel(big,tolerance=0.5)
        msp.add_text(p['name'],height=6,dxfattribs={'layer':'ETCH'}).set_placement((lp.x,lp.y),align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER)
        doc.saveas(f"{out_dir}/{name}/dxf/{p['name']}.dxf")
    from collections import Counter
    bc=Counter((b['d'],int(np.ceil((b['s1']-b['s0']+0.045)*1000/10)*10)) for b in bolts)
    with open(f'{out_dir}/{name}/BOM.csv','w') as f:
        f.write('poz;tanım;adet\n')
        for p in plates: f.write(f"{p['name']};{'Tarak / kilit plakası' if p.get('kind')=='comb' else 'Plaka'} 8 mm Al 6082-T651 lazer;1\n")
        for (d,L),n in sorted(bc.items()): f.write(f"M{d}x{L};Cıvata ISO 4014 M{d}×{L} + 2 pul + somun, A2/8.8 galvaniz;{n}\n")
    return dict(plates=len(plates),bolts=len(bolts),area=area,mass=area*T*2700)
