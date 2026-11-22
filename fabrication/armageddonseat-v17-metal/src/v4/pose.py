# Re-posing the original figure: rigid body segments rotated about anatomical joints (side plane),
# limb lengths unchanged, optional uniform scale. The posed figure is sampled from the slice SDF volume.
import numpy as np
from wf import _samp, D, FS, M, Figure
SEGS=['head','torso','uarm','farm','hand','thigh','shin','foot']
J=dict(HIP=(-0.20,0.505),KNEE=(0.21,0.505),ANK=(0.235,0.08),HEEL=(0.20,0.0),SH=(-0.36,0.93),EL=(-0.40,0.55),WR=(-0.39,0.12),NECK=(-0.27,1.06))
def seg_of(x,y,z):
    ax=np.abs(x)
    out=np.full(x.shape,-1)
    lower=z>-0.19
    out=np.where(~lower&(y>=1.075),0,out)
    out=np.where(~lower&(y>=0.40)&(y<1.075)&((ax<0.212)|(y>=0.86)),1,out)
    arm=~lower&(ax>=0.212)&(y<0.86)
    out=np.where(arm&(y>=0.55),2,out); out=np.where(arm&(y>=0.12)&(y<0.55),3,out); out=np.where(arm&(y<0.12),4,out)
    out=np.where(lower&(z<=0.165)&(y>=0.40),5,out)
    out=np.where(lower&(z>0.165)&(y>=0.10),6,out); out=np.where(lower&(z>0.165)&(y<0.10),7,out)
    return out
def T(dz,dy): return np.array([[1,0,dz],[0,1,dy],[0,0,1.]])
def R(theta):
    c,s=np.cos(np.radians(theta)),np.sin(np.radians(theta)); return np.array([[c,-s,0],[s,c,0],[0,0,1.]])
def Rabout(p,theta): return T(*p)@R(theta)@T(-p[0],-p[1])
class Pose:
    def __init__(self,s=1.0,root=(0,0),t=0,n=0,sh=0,el=0,wr=0,th=0,kn=0,ft=0,foot_pivot='ANK',
                 leg_kx=1.12,leg_dx=0.029,arm_dx=0.0,mode='detail',thick=0.005):
        self.s=s; self.mode=mode; self.thick=thick; self.leg_kx=leg_kx; self.leg_dx=leg_dx; self.arm_dx=arm_dx
        Mr=T(*root)
        Mt=Mr@Rabout(J['HIP'],t); Mh=Mt@Rabout(J['NECK'],n)
        Mua=Mt@Rabout(J['SH'],sh); Mfa=Mua@Rabout(J['EL'],el); Mhd=Mfa@Rabout(J['WR'],wr)
        Mth=Mr@Rabout(J['HIP'],th); Msh=Mth@Rabout(J['KNEE'],kn); Mft=Msh@Rabout(J[foot_pivot],ft)
        self.M=[Mh,Mt,Mua,Mfa,Mhd,Mth,Msh,Mft]; self.Mi=[np.linalg.inv(m) for m in self.M]
    def fx(self,k,x):        # forward x map
        if k in (5,6,7): return np.sign(x)*(np.abs(x)*self.leg_kx-self.leg_dx)
        if k in (3,4): return np.sign(x)*(np.abs(x)-self.arm_dx)
        return x
    def fx_inv(self,k,x):
        if k in (5,6,7): return np.sign(x)*(np.abs(x)+self.leg_dx)/self.leg_kx
        if k in (3,4): return np.sign(x)*(np.abs(x)+self.arm_dx)
        return x
    def point(self,k,z,y):   # forward map of an original (z,y) point of segment k (scaled)
        p=self.M[k]@np.array([z,y,1.]); return p[0]*self.s,p[1]*self.s
    def value(self,x,y,z):
        x=np.asarray(x,float)/self.s; y=np.asarray(y,float)/self.s; z=np.asarray(z,float)/self.s
        best=np.full(x.shape,0.5,np.float32)
        for k in range(8):
            Mi=self.Mi[k]; zo=Mi[0,0]*z+Mi[0,1]*y+Mi[0,2]; yo=Mi[1,0]*z+Mi[1,1]*y+Mi[1,2]; xo=self.fx_inv(k,x)
            ok=seg_of(xo,yo,zo)==k
            if not ok.any(): continue
            if self.mode=='detail': v=_samp(D,xo[ok],yo[ok],zo[ok],'near')
            else: v=np.minimum(_samp(FS,xo[ok],yo[ok],zo[ok],'lin'),_samp(M,xo[ok],yo[ok],zo[ok],'lin'))
            b=best[ok]; best[ok]=np.minimum(b,v)
        return best*self.s
