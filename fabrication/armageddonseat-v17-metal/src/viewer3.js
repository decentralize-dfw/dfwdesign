import * as THREE from 'three';
import {GLTFLoader} from 'three/examples/jsm/loaders/GLTFLoader.js';
import {OrbitControls} from 'three/examples/jsm/controls/OrbitControls.js';
import {RoomEnvironment} from 'three/examples/jsm/environments/RoomEnvironment.js';
import {Line2} from 'three/examples/jsm/lines/Line2.js';
import {LineMaterial} from 'three/examples/jsm/lines/LineMaterial.js';
import {LineGeometry} from 'three/examples/jsm/lines/LineGeometry.js';
function b64(s){const bin=atob(s);const u=new Uint8Array(bin.length);for(let i=0;i<bin.length;i++)u[i]=bin.charCodeAt(i);return u.buffer;}
const ss=(x)=>{x=Math.min(1,Math.max(0,x));return x*x*(3-2*x);};
const seg=(t,a,b)=>ss((t-a)/(b-a));
window.initViewer3=function(el,opt){
  let r;try{r=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:!!opt.still});}catch(e){const f=el.querySelector('.vfallback');if(f)f.hidden=false;return null;}
  r.setPixelRatio(opt.still?1:Math.min(devicePixelRatio,2));r.toneMapping=THREE.ACESFilmicToneMapping;r.shadowMap.enabled=true;r.shadowMap.type=THREE.PCFSoftShadowMap;
  el.prepend(r.domElement);
  const s=new THREE.Scene();
  s.environment=new THREE.PMREMGenerator(r).fromScene(new RoomEnvironment(),0.04).texture;
  const L=new THREE.DirectionalLight(0xffffff,1.7);L.position.set(1.5,3,2);L.castShadow=true;L.shadow.mapSize.set(2048,2048);
  Object.assign(L.shadow.camera,{left:-1.6,right:1.6,top:1.6,bottom:-1.6});s.add(L);
  s.add(new THREE.HemisphereLight(0xffffff,0x888888,0.9));const L2=new THREE.DirectionalLight(0xffffff,0.7);L2.position.set(-2,1.5,-2);s.add(L2);
  const ground=new THREE.Mesh(new THREE.PlaneGeometry(14,14),new THREE.ShadowMaterial({opacity:.16}));ground.rotation.x=-Math.PI/2;ground.receiveShadow=true;s.add(ground);
  const c=new THREE.PerspectiveCamera(30,1,0.01,60);c.position.set(...(opt.cam||[2.3,1.45,2.5]));
  const ctl=new OrbitControls(c,r.domElement);ctl.target.set(...(opt.target||[0,0.6,-0.05]));ctl.enableDamping=true;ctl.minDistance=0.6;ctl.maxDistance=7;ctl.maxPolarAngle=Math.PI*0.53;ctl.update();
  const meta=opt.meta, parts={}, rodsBy={};
  meta.rods.forEach(q=>rodsBy[q.name]=q);
  const K=meta.keel, E=1.0, ZC=-0.1, LIFT=0.12, DROP=0.22;
  const Zs=(z,t4)=>z+t4*E*(z-ZC);
  const rodOff=(q,t2,t4)=>{const zc=(q.z0+q.z1)/2;const follow=Zs(zc,t4)-zc;
    const out=q.dir>0?(Zs(q.z1,t4)+0.12-q.za):(Zs(q.z0,t4)-0.12-q.zb);return (1-t2)*follow+t2*out;};
  let model=null, orig=null, cur=null;
  // dashed guide lines
  const lineMatRod=new LineMaterial({color:0x26313c,linewidth:1.5,dashed:true,dashSize:0.016,gapSize:0.010,transparent:true,opacity:0,depthTest:false});
  const lineMatKeel=new LineMaterial({color:0x1f5fa8,linewidth:1.8,dashed:true,dashSize:0.012,gapSize:0.008,transparent:true,opacity:0,depthTest:false});
  const mkLine=(m)=>{const g=new LineGeometry();g.setPositions([0,0,0,0,0,0.001]);const l=new Line2(g,m);l.frustumCulled=false;l.renderOrder=10;return l;};
  const guides=new THREE.Group();s.add(guides);
  const rodLines=meta.rods.map(q=>{const l=mkLine(lineMatRod);guides.add(l);return [q,l];});
  const keelLines=K.slots.map(z=>{const l=mkLine(lineMatKeel);guides.add(l);return [z,l];});
  function setLine(l,a,b){l.geometry.setPositions([...a,...b]);l.computeLineDistances();}
  function pose(t){
    if(!model)return;
    const t1=seg(t,0.0,0.22), t2=seg(t,0.18,0.48), t3=seg(t,0.44,0.72), t4=seg(t,0.68,1.0);
    const up=LIFT*t3, kd=-DROP*t3;
    meta.parts.forEach(p=>{
      const o=parts[p.name]; if(!o)return; let dx=0,dy=up,dz=0;
      if(p.type==='rib'){dz=Zs(p.z,t4)-p.z;}
      else if(p.type==='spacer'){const zm=(p.za+p.zb)/2;dz=Zs(zm,t4)-zm;}
      else if(p.type==='keel'){dy=kd;}
      else if(p.type==='keelbolt'){dy=kd;dx=0.06*t1;}
      else if(p.type==='rod'){dz=rodOff(rodsBy[p.rod],t2,t4);}
      else if(p.type==='nut'){const q=rodsBy[p.rod];const zend=p.end<0?q.z0:q.z1;const base=(Zs(zend,t4)-zend)+p.end*0.05*t1;
        if(p.end===q.dir){const off=rodOff(q,t2,t4);const tip=(q.dir>0?q.zb:q.za)+off+q.dir*0.035;dz=(1-t2)*base+t2*(tip-p.z);} else dz=base;}
      o.position.set(dx,dy,dz);
    });
    rodLines.forEach(([q,l])=>{
      const off=rodOff(q,t2,t4);
      const a=Math.min(Zs(q.z0,t4)-0.08,q.za+off-0.04), b=Math.max(Zs(q.z1,t4)+0.08,q.zb+off+0.04);
      setLine(l,[q.x,q.y+up,a],[q.x,q.y+up,b]);
    });
    lineMatRod.opacity=0.55*t1;
    keelLines.forEach(([z,l])=>{setLine(l,[0,K.ymid+up,Zs(z,t4)],[0,K.ymid+kd,z]);});
    lineMatKeel.opacity=0.8*t3;
    guides.visible=t>0.001&&cur===model;
    const k=1+0.45*ss(t);  // pull the camera back as the parts spread
    if(!opt.still||opt.dolly){const dir=c.position.clone().sub(ctl.target);const want=baseDist*k;dir.setLength(want);c.position.copy(ctl.target).add(dir);}
  }
  let baseDist=0;
  baseDist=c.position.distanceTo(ctl.target);
  ctl.addEventListener('end',()=>{baseDist=c.position.distanceTo(ctl.target)/(1+0.45*ss(state.t));});
  const ld=new GLTFLoader();
  ld.parse(b64(window[opt.data]),'',gl=>{
    gl.scene.traverse(o=>{if(o.isMesh){o.castShadow=true;o.receiveShadow=true;}});
    meta.parts.forEach(p=>{const o=gl.scene.getObjectByName(p.name);if(o)parts[p.name]=o;});
    model=gl.scene;cur=model;s.add(model);pose(state.t);if(opt.still)render();
    if(opt.onload)opt.onload();
  });
  function showOrig(on){
    if(on){ if(!orig){ld.parse(b64(window[opt.orig]),'',gl=>{gl.scene.traverse(o=>{if(o.isMesh){o.geometry.computeVertexNormals();o.material=new THREE.MeshStandardMaterial({color:0xa9503f,metalness:.2,roughness:.6,side:THREE.DoubleSide});o.castShadow=true;}});orig=gl.scene;showOrig(true);});return;}
      if(model)s.remove(model);s.add(orig);cur=orig;guides.visible=false;}
    else{ if(orig)s.remove(orig); if(model){s.add(model);cur=model;pose(state.t);} }
  }
  const state={t:opt.t||0,target:opt.t||0,speed:1/6};
  function size(){const w=el.clientWidth,h=el.clientHeight;r.setSize(w,h);c.aspect=w/h;c.updateProjectionMatrix();lineMatRod.resolution.set(w,h);lineMatKeel.resolution.set(w,h);}
  size(); if(!opt.still)new ResizeObserver(size).observe(el);
  const bg=()=>{const v=getComputedStyle(el).getPropertyValue('--viewer-bg').trim();s.background=new THREE.Color(v||'#e9ecee');};
  bg(); if(!opt.still){new MutationObserver(bg).observe(document.documentElement,{attributes:true});matchMedia('(prefers-color-scheme: dark)').addEventListener('change',bg);}
  function render(){r.render(s,c);}
  let last=performance.now();
  if(!opt.still)(function loop(now){requestAnimationFrame(loop);const dt=Math.min(0.25,(now-last)/1000);last=now;
    if(state.t!==state.target){const step=state.speed*dt*(opt.reduce?20:1);state.t=state.t<state.target?Math.min(state.target,state.t+step):Math.max(state.target,state.t-step);pose(state.t);if(opt.onprogress)opt.onprogress(state.t);}
    ctl.update();render();})(last);
  return {
    explode(on){state.target=on?1:0;},
    scrub(v){state.t=state.target=v;pose(v);},
    get t(){return state.t;},
    showOrig,
    still(t){state.t=t;pose(t);render();}
  };
};
