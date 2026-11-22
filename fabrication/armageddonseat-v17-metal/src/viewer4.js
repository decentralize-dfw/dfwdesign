import * as THREE from 'three';
import {GLTFLoader} from 'three/examples/jsm/loaders/GLTFLoader.js';
import {OrbitControls} from 'three/examples/jsm/controls/OrbitControls.js';
import {RoomEnvironment} from 'three/examples/jsm/environments/RoomEnvironment.js';
import {RoundedBoxGeometry} from 'three/examples/jsm/geometries/RoundedBoxGeometry.js';
import {LineSegments2} from 'three/examples/jsm/lines/LineSegments2.js';
import {LineSegmentsGeometry} from 'three/examples/jsm/lines/LineSegmentsGeometry.js';
import {LineMaterial} from 'three/examples/jsm/lines/LineMaterial.js';

function b64(s){const bin=atob(s);const u=new Uint8Array(bin.length);for(let i=0;i<bin.length;i++)u[i]=bin.charCodeAt(i);return u.buffer;}
const ss=x=>{x=Math.min(1,Math.max(0,x));return x*x*(3-2*x);};

// ---------- a generic seated person (clay mannequin, ~175 cm) ----------
function makeHuman(pose,opt={}){
  const g=new THREE.Group();
  const mat=new THREE.MeshStandardMaterial({color:opt.color||0xd8b49a,roughness:0.75,metalness:0.0,transparent:true,opacity:opt.opacity||0.42,depthWrite:false});
  const th=THREE.MathUtils.degToRad(pose.theta);
  const v=(z,y)=>new THREE.Vector2(z,y);
  const H=v(pose.H[0],pose.H[1]);
  const up=v(-Math.sin(th),Math.cos(th));
  const sh=H.clone().add(up.clone().multiplyScalar(0.47));
  const neck=H.clone().add(up.clone().multiplyScalar(0.55));
  const head=neck.clone().add(v(-Math.sin(th*0.4),Math.cos(th*0.4)).multiplyScalar(0.12));
  const kneeY=pose.seatFront[1]+0.075;const dy=kneeY-H.y;const knee=v(H.x+Math.sqrt(Math.max(0.01,0.44*0.44-dy*dy)),kneeY);
  const footY=pose.footY||0;const ankY=footY+0.075;const dk=knee.y-ankY;
  let ank; if(dk>=0.43){ank=v(knee.x+0.06,knee.y-0.425);} else ank=v(knee.x+Math.sqrt(0.43*0.43-dk*dk)*0.55,ankY);
  const toe=ank.clone().add(v(0.17,-0.05));
  const hand=H.clone().lerp(knee,0.62).add(v(0,0.09));
  const elbow=sh.clone().add(v(0.03,-0.27));
  const P=(p,x)=>new THREE.Vector3(x,p.y,p.x);
  function limb(a,b,r){const A=a,B=b;const d=B.clone().sub(A);const L=d.length();
    const m=new THREE.Mesh(new THREE.CapsuleGeometry(r,Math.max(0.001,L),6,14),mat);
    m.position.copy(A.clone().add(B).multiplyScalar(0.5));m.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),d.normalize());m.castShadow=true;m.renderOrder=5;g.add(m);}
  // torso (flattened capsule)
  const tor=new THREE.Mesh(new THREE.CapsuleGeometry(0.135,0.30,8,18),mat);
  const tc=P(H.clone().lerp(sh,0.55),0);tor.position.copy(tc);tor.scale.set(1.22,1,0.78);tor.rotation.x=-th;tor.castShadow=true;g.add(tor);
  const pel=new THREE.Mesh(new THREE.SphereGeometry(0.15,20,14),mat);pel.position.copy(P(H,0).add(new THREE.Vector3(0,0.02,0.01)));pel.scale.set(1.15,0.8,0.95);pel.castShadow=true;g.add(pel);
  const hd=new THREE.Mesh(new THREE.SphereGeometry(0.095,24,18),mat);hd.position.copy(P(head,0));hd.scale.set(0.9,1.12,1.0);hd.castShadow=true;g.add(hd);
  limb(P(neck,0),P(head,0),0.05);
  for(const s of [-1,1]){
    limb(P(H,0.095*s),P(knee,0.10*s),0.075);
    limb(P(knee,0.10*s),P(ank,0.10*s),0.055);
    limb(P(ank,0.10*s),P(toe,0.105*s),0.04);
    limb(P(sh,0.20*s),P(elbow,0.22*s),0.045);
    limb(P(elbow,0.22*s),P(hand,0.16*s),0.038);
  }
  return g;
}
// ---------- a generic plain plastic chair ----------
function makePlasticChair(){
  const g=new THREE.Group();
  const m=new THREE.MeshStandardMaterial({color:0xf1f1ec,roughness:0.55,metalness:0});
  const seat=new THREE.Mesh(new RoundedBoxGeometry(0.44,0.035,0.42,4,0.015),m);seat.position.set(0,0.445,0.02);g.add(seat);
  const back=new THREE.Mesh(new RoundedBoxGeometry(0.42,0.36,0.025,4,0.012),m);back.position.set(0,0.69,-0.19);back.rotation.x=-0.17;g.add(back);
  for(const sx of [-1,1]){const post=new THREE.Mesh(new RoundedBoxGeometry(0.03,0.42,0.025,3,0.01),m);post.position.set(0.19*sx,0.62,-0.18);post.rotation.x=-0.17;g.add(post);}
  for(const sx of [-1,1])for(const sz of [-1,1]){
    const leg=new THREE.Mesh(new THREE.CylinderGeometry(0.016,0.012,0.44,16),m);
    leg.position.set(0.19*sx,0.215,0.02+0.18*sz);leg.rotation.z=-0.06*sx;leg.rotation.x=0.06*sz;g.add(leg);}
  g.traverse(o=>{if(o.isMesh){o.castShadow=true;o.receiveShadow=true;}});
  return g;
}
const PLASTIC_POSE={H:[-0.085,0.54],theta:11,seatFront:[0.22,0.45],footY:0};

window.initViewer4=function(el,opt){
  let r;try{r=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:!!opt.still});}catch(e){const f=el.querySelector('.vfallback');if(f)f.hidden=false;return null;}
  r.setPixelRatio(opt.still?1:Math.min(devicePixelRatio,2));r.toneMapping=THREE.ACESFilmicToneMapping;r.shadowMap.enabled=true;r.shadowMap.type=THREE.PCFSoftShadowMap;
  el.prepend(r.domElement);
  const s=new THREE.Scene();
  s.environment=new THREE.PMREMGenerator(r).fromScene(new RoomEnvironment(),0.04).texture;
  const L=new THREE.DirectionalLight(0xffffff,1.7);L.position.set(1.6,3.2,2.2);L.castShadow=true;L.shadow.mapSize.set(2048,2048);
  Object.assign(L.shadow.camera,{left:-2,right:2,top:2,bottom:-2});s.add(L);
  s.add(new THREE.HemisphereLight(0xffffff,0x888888,0.9));const L2=new THREE.DirectionalLight(0xffffff,0.6);L2.position.set(-2,1.5,-2);s.add(L2);
  const ground=new THREE.Mesh(new THREE.PlaneGeometry(16,16),new THREE.ShadowMaterial({opacity:.16}));ground.rotation.x=-Math.PI/2;ground.receiveShadow=true;s.add(ground);
  const c=new THREE.PerspectiveCamera(30,1,0.01,60);c.position.set(...(opt.cam||[2.5,1.5,2.7]));
  const ctl=new OrbitControls(c,r.domElement);ctl.target.set(0,0.6,-0.05);ctl.enableDamping=true;ctl.minDistance=0.6;ctl.maxDistance=8;ctl.maxPolarAngle=Math.PI*0.53;ctl.update();
  const matAxis=new LineMaterial({color:0x26313c,linewidth:1.6,dashed:true,dashSize:0.016,gapSize:0.010,transparent:true,opacity:0,depthTest:false});
  const matSlot=new LineMaterial({color:0x1f5fa8,linewidth:1.9,dashed:true,dashSize:0.012,gapSize:0.008,transparent:true,opacity:0,depthTest:false});
  const loader=new GLTFLoader();
  const models={}; let cur=null, curId=null;
  const human=new THREE.Group(); s.add(human);
  const compare=new THREE.Group(); s.add(compare);
  const plastic=makePlasticChair(); compare.add(plastic); const plasticHuman=makeHuman(PLASTIC_POSE,{color:0xcfb09a}); compare.add(plasticHuman);
  compare.position.set(0.98,0,0.05); compare.visible=false;
  const state={t:0,target:0,speed:1/6,human:false,compare:false};
  function moveOf(p,t){let x=0,y=0,z=0;for(const m of p.moves||[]){const k=ss((t-m.t0)/(m.t1-m.t0));x+=m.v[0]*k;y+=m.v[1]*k;z+=m.v[2]*k;}return [x,y,z];}
  function pose(t){
    const M=cur; if(!M)return;
    const off={};
    M.meta.parts.forEach(p=>{const o=M.nodes[p.name];const d=moveOf(p,t);off[p.name]=d;if(o)o.position.set(d[0],d[1],d[2]);});
    const fill=(segs,list)=>{const arr=new Float32Array(list.length*6);let i=0;for(const j of list){const a=off[j.a]||[0,0,0],b=off[j.b]||[0,0,0];
      arr[i++]=j.p[0]+a[0];arr[i++]=j.p[1]+a[1];arr[i++]=j.p[2]+a[2];arr[i++]=j.p[0]+b[0];arr[i++]=j.p[1]+b[1];arr[i++]=j.p[2]+b[2];}
      segs.geometry.setPositions(arr);segs.computeLineDistances();};
    if(M.axisList.length)fill(M.axisLines,M.axisList);
    if(M.slotList.length)fill(M.slotLines,M.slotList);
    const vis=ss(t*6);matAxis.opacity=0.6*vis;matSlot.opacity=0.85*vis;
    M.axisLines.visible=M.slotLines.visible=t>0.002;
    human.visible=state.human&&t<0.02; compare.visible=state.compare&&t<0.02;
    if(!opt.still||opt.dolly){const k=1+0.4*ss(t);const dir=c.position.clone().sub(ctl.target);dir.setLength(baseDist*k);c.position.copy(ctl.target).add(dir);}
  }
  let baseDist=c.position.distanceTo(ctl.target);
  ctl.addEventListener('end',()=>{baseDist=c.position.distanceTo(ctl.target)/(1+0.4*ss(state.t));});
  function frame(){const tx=state.compare?0.49:0;ctl.target.set(tx,0.6,-0.05);}
  async function getBuf(v){ if(v.b64&&window[v.b64])return b64(window[v.b64]); const res=await fetch(v.url); if(!res.ok)throw new Error('load'); const txt=await res.text(); return b64(txt.trim()); }
  async function getMeta(v){ if(v.metaVar&&window[v.metaVar])return window[v.metaVar]; const res=await fetch(v.metaUrl);return await res.json(); }
  async function load(v){
    if(models[v.id])return models[v.id];
    const [buf,meta]=await Promise.all([getBuf(v),getMeta(v)]);
    const gl=await new Promise((res,rej)=>loader.parse(buf,'',res,rej));
    gl.scene.traverse(o=>{if(o.isMesh){o.castShadow=true;o.receiveShadow=true;if(v.recolor){o.geometry.computeVertexNormals();o.material=new THREE.MeshStandardMaterial({color:0xa9503f,metalness:.2,roughness:.6,side:THREE.DoubleSide});}}});
    const nodes={};(meta.parts||[]).forEach(p=>{const o=gl.scene.getObjectByName(p.name);if(o)nodes[p.name]=o;});
    const axisList=(meta.joints||[]).filter(j=>j.k==='axis'), slotList=(meta.joints||[]).filter(j=>j.k!=='axis');
    const mk=(m)=>{const l=new LineSegments2(new LineSegmentsGeometry(),m);l.frustumCulled=false;l.renderOrder=10;l.visible=false;return l;};
    const M={id:v.id,root:gl.scene,meta,nodes,axisList,slotList,axisLines:mk(matAxis),slotLines:mk(matSlot)};
    gl.scene.add(M.axisLines);gl.scene.add(M.slotLines);
    models[v.id]=M;return M;
  }
  async function show(v){
    const M=await load(v);
    if(cur)s.remove(cur.root);
    cur=M;curId=v.id;s.add(M.root);
    human.clear(); if(M.meta.pose&&M.meta.pose.H)human.add(makeHuman(M.meta.pose));
    pose(state.t); if(opt.onshow)opt.onshow(M.meta);
    return M;
  }
  function size(){const w=el.clientWidth,h=el.clientHeight;r.setSize(w,h);c.aspect=w/h;c.updateProjectionMatrix();matAxis.resolution.set(w,h);matSlot.resolution.set(w,h);}
  size(); if(!opt.still)new ResizeObserver(size).observe(el);
  const bg=()=>{const v=getComputedStyle(el).getPropertyValue('--viewer-bg').trim();s.background=new THREE.Color(v||'#e9ecee');};
  bg(); if(!opt.still){new MutationObserver(bg).observe(document.documentElement,{attributes:true});matchMedia('(prefers-color-scheme: dark)').addEventListener('change',bg);}
  let last=performance.now();
  if(!opt.still)(function loop(now){requestAnimationFrame(loop);const dt=Math.min(0.25,(now-last)/1000);last=now;
    if(state.t!==state.target){const step=state.speed*dt*(opt.reduce?20:1);state.t=state.t<state.target?Math.min(state.target,state.t+step):Math.max(state.target,state.t-step);pose(state.t);if(opt.onprogress)opt.onprogress(state.t);}
    ctl.update();r.render(s,c);})(last);
  return {
    show, explode(on){state.target=on?1:0;}, scrub(v){state.t=state.target=v;pose(v);},
    setHuman(on){state.human=on;pose(state.t);}, setCompare(on){state.compare=on;compare.visible=on;frame();pose(state.t);},
    still(t,camPos){if(camPos){c.position.set(...camPos);baseDist=c.position.distanceTo(ctl.target);}state.t=t;ctl.update();pose(t);r.render(s,c);},
    get t(){return state.t;}
  };
};
