import json, os, shutil
from wf import Envelope
m=json.load(open('../v3/chair_parts.json'))
E=1.0; ZC=-0.1; LIFT=0.12; DROP=0.22
T1=(0.0,0.22); T2=(0.18,0.48); T3=(0.44,0.72); T4=(0.68,1.0)
mv=lambda v,t: dict(v=v,t0=t[0],t1=t[1])
rods={r['name']:r for r in m['rods']}
parts=[]; ribz={}
for p in m['parts']:
    t=p['type']; ms=[]
    if t=='rib': ms=[mv([0,LIFT,0],T3),mv([0,0,E*(p['z']-ZC)],T4)]; ribz.setdefault(round(p['z'],4),p['name'])
    elif t=='spacer': zm=(p['za']+p['zb'])/2; ms=[mv([0,LIFT,0],T3),mv([0,0,E*(zm-ZC)],T4)]
    elif t=='keel': ms=[mv([0,-DROP,0],T3)]
    elif t=='keelbolt': ms=[mv([0.06,0,0],T1),mv([0,-DROP,0],T3)]
    elif t=='rod':
        q=rods[p['rod']]; zc=(q['z0']+q['z1'])/2
        out0=(q['z1']+0.12-q['za']) if q['dir']>0 else (q['z0']-0.12-q['zb'])
        zend=q['z1'] if q['dir']>0 else q['z0']
        ms=[mv([0,0,out0],T2),mv([0,LIFT,0],T3),mv([0,0,E*(zend-ZC)],T4)]
    elif t=='nut':
        q=rods[p['rod']]; e=p['end']; zend=q['z0'] if e<0 else q['z1']
        ms=[mv([0,0,e*0.05],T1),mv([0,LIFT,0],T3),mv([0,0,E*(zend-ZC)],T4)]
        if e==q['dir']:
            out0=(q['z1']+0.12-q['za']) if q['dir']>0 else (q['z0']-0.12-q['zb'])
            tip=(q['zb'] if q['dir']>0 else q['za'])+out0+q['dir']*0.035
            ms.insert(1,mv([0,0,tip-p['z']-e*0.05],T2))
    parts.append(dict(name=p['name'],moves=ms))
joints=[]
Zs=sorted(ribz)
for q in m['rods']:
    for z in Zs:
        if q['z0']-1e-6<=z<=q['z1']+1e-6: joints.append(dict(a=ribz[z],b='ROD_'+q['name'],p=[q['x'],q['y'],z],k='axis'))
K=m['keel']
for z in K['slots']:
    z=round(z,4)
    if z in ribz: joints.append(dict(a=ribz[z],b='K01',p=[0,K['ymid'],z],k='slot'))
env=Envelope(seat_y=0.44,zf=0.17,depth=0.385,tilt=4,recline=18)
steps=['Omurga laması K01 iki sehpaya dik konur.','Kaburgalar sırayla yuvalara indirilir; uç kaburgalar omurganın uç dillerine geçer.',
       'Gergi çubukları deliklerden sürülür; her aralığa bir mesafe borusu takılır.','Pul + kep somun ile sıkılır.']
os.makedirs('out/RC',exist_ok=True)
json.dump(dict(parts=parts,joints=joints,pose=dict(env.pose(),footY=0.0,s=1.0),steps=steps),open('out/RC/meta.json','w'))
shutil.copy('../v3/chair_parts.glb','out/RC/model.glb')
print(len(parts),len(joints))
