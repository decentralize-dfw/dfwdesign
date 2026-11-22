# Interlocking combs: a comb plate crosses a stack of host plates; hosts get pads + half-lap slots,
# the two end hosts get closed windows (comb tabs pass through -> comb cannot drop out),
# one big bolt runs beside the comb through all hosts and clamps the stack against the comb teeth.
from wf import *
SLOT=T+0.0006; CH=0.0015
def band_poly(sa,sb,ylo,yhi,tab=0.020,n=60):
    ss=np.linspace(sa,sb,n)
    top=[(s,yhi(s)) for s in ss]; bot=[(s,ylo(s)) for s in ss[::-1]]
    g=Polygon(top+bot)
    return close(opn(g,0.006),0.006)
class Comb:
    def __init__(self,name,pos,side,hosts,ylo,yhi,bolt_off=0.045,bolt=True,extra=None,floor=False,pad_w=0.022):
        self.name=name; self.pos=pos; self.side=side; self.hosts=sorted(hosts)
        self.ylo=ylo; self.yhi=yhi; self.bolt_off=bolt_off; self.has_bolt=bolt; self.extra=extra; self.floor=floor; self.pad_w=pad_w
        self.sa=self.hosts[0]; self.sb=self.hosts[-1]
    def ymid(self,s): return 0.5*(self.ylo(s)+self.yhi(s))
    def bolt_uv(self):
        s=(self.sa+self.sb)/2; return (self.pos+self.bolt_off, self.ymid(s))
    def is_end(self,s): return abs(s-self.sa)<1e-6 or abs(s-self.sb)<1e-6
    # ---- what each host receives ----
    def host_pad(self,s,host_mat):
        u=self.pos; lo,hi=self.ylo(s),self.yhi(s); pw=self.pad_w
        pad=box(u-pw,lo,u+pw,hi)
        if self.has_bolt:
            bu,bv=self.bolt_uv(); pad=U([pad,Point(bu,bv).buffer(BOLT[16]['boss']),box(min(u,bu),lo,max(u,bu),hi)])
        if self.is_end(s):   # closed window needs material on the open side too
            if self.side=='below' and not self.floor: pad=U([pad,box(u-pw,lo-0.022,u+pw,hi)])
            if self.side=='below' and self.floor: pass
            if self.side=='above': pad=U([pad,box(u-pw,lo,u+pw,hi+0.022)])
        # connector strip from the band to the host's own material
        strip=box(u-0.013,hi,u+0.013,hi+0.35) if self.side=='below' else box(u-0.013,lo-0.35,u+0.013,lo)
        hit=host_mat.intersection(strip)
        if not hit.is_empty:
            if self.side=='below': ytop=min(c.bounds[1] for c in geoms(hit))+0.01; pad=U([pad,box(u-0.012,hi-0.005,u+0.012,ytop)])
            else: yb=max(c.bounds[3] for c in geoms(hit))-0.01; pad=U([pad,box(u-0.012,yb,u+0.012,lo+0.005)])
        return close(pad,0.008)
    def host_cut(self,s):
        u=self.pos; h=SLOT/2; lo,hi,mid=self.ylo(s),self.yhi(s),self.ymid(s)
        if self.is_end(s): return box(u-h,lo-0.0005,u+h,hi+0.0005)
        if self.side=='below':
            return Polygon([(u-h-CH,lo-0.03),(u-h-CH,lo),(u-h,lo+CH),(u-h,mid),(u+h,mid),(u+h,lo+CH),(u+h+CH,lo),(u+h+CH,lo-0.03)])
        return Polygon([(u-h-CH,hi+0.03),(u-h-CH,hi),(u-h,hi-CH),(u-h,mid),(u+h,mid),(u+h,hi-CH),(u+h+CH,hi),(u+h+CH,hi+0.03)])
    def host_hole(self):
        if not self.has_bolt: return None
        bu,bv=self.bolt_uv(); return Point(bu,bv).buffer(BOLT[16]['hole']/2,quad_segs=16)
    # ---- the comb plate itself (2D coords: s along the stack, y) ----
    def outline(self):
        h=SLOT/2; tab=0.020
        g=band_poly(self.sa-T/2-tab,self.sb+T/2+tab,self.ylo,self.yhi)
        if self.extra is not None: g=U([g,self.extra])
        cuts=[]
        for s in self.hosts[1:-1]:
            lo,hi,mid=self.ylo(s),self.yhi(s),self.ymid(s)
            if self.side=='below':
                cuts.append(Polygon([(s-h,mid),(s+h,mid),(s+h,hi-CH),(s+h+CH,hi),(s+h+CH,hi+0.03),(s-h-CH,hi+0.03),(s-h-CH,hi),(s-h,hi-CH)]))
            else:
                cuts.append(Polygon([(s-h,mid),(s+h,mid),(s+h,lo+CH),(s+h+CH,lo),(s+h+CH,lo-0.03),(s-h-CH,lo-0.03),(s-h-CH,lo),(s-h,lo+CH)]))
        return close(g.difference(U(cuts)),0.0015) if cuts else g
    def bolt(self,host_plane):
        bu,bv=self.bolt_uv()
        return dict(name='BOLT_'+self.name,d=16,axis='z' if host_plane=='z' else 'x',u=bu,v=bv,s0=self.sa-T/2,s1=self.sb+T/2,nutside=1)
