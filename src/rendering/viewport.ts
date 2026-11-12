import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import type { Simulation } from '../physics/simulation';
import type { FlightFrame } from './flight-recording';
import { landscape } from './landscape';
import { length, type V3 } from '../physics/math';
import { materials } from '../physics/materials';
import { airVelocity } from '../physics/weather';
import { aerodynamicLoads, centerOfLift } from '../physics/aerodynamics';
import { liftSummary, stallSummary, type LiftSummary, type StallSummary } from '../physics/flight-loads';
export interface ViewOptions { heat:boolean; wire:boolean; forces:boolean; velocity:boolean; follow:boolean; flex:boolean; flexScale:number; layers:boolean; massCenter:boolean; liftCenter:boolean; liftMeter:boolean; stallMeter:boolean }
class PlaneView {
 environment=landscape(); scene=new THREE.Scene(); geometry=new THREE.BufferGeometry(); paper:THREE.Mesh;
 wire:THREE.LineSegments; folds:THREE.LineSegments; path:THREE.Line;
 flexArrows:THREE.ArrowHelper[]=[];referenceWire:THREE.LineSegments;flexRms=0;
 arrows:THREE.ArrowHelper[]=[]; velocity=new THREE.ArrowHelper();
 massMarker=this.makeCenterMarker('MASS','#2879bd',1);liftMarker=this.makeCenterMarker('LIFT','#a640a5',-1);
 liftCenter?:V3;liftReading?:LiftSummary;stallReading?:StallSummary;private analyzedFrame?:FlightFrame;
 private sun=new THREE.DirectionalLight(0xffffff,3.2);
 constructor(public simulation:Simulation,public accent:number){
  this.scene.background=new THREE.Color('#c5dfdb');this.scene.fog=new THREE.Fog('#c5dfdb',35,110);this.scene.add(this.environment);
  this.scene.add(new THREE.HemisphereLight(0xffffff,0x7d8d80,2.7));
  const key=this.sun;key.castShadow=true;key.shadow.mapSize.set(1024,1024);
  // Keep a small shadow camera around the aircraft for a crisp contact cue,
  // even when the flight travels far from the launch point.
  Object.assign(key.shadow.camera,{left:-.8,right:.8,top:.8,bottom:-.8,near:.1,far:160});key.shadow.camera.updateProjectionMatrix();
  key.shadow.bias=-.00002;key.shadow.normalBias=.002;this.scene.add(key,key.target);
  const m=simulation.mesh,surface=materials[simulation.parameters.material];
  this.geometry.setAttribute('position',new THREE.Float32BufferAttribute(new Float32Array(m.rest.length*3),3));
  this.geometry.setAttribute('color',new THREE.Float32BufferAttribute(new Float32Array(m.rest.length*3),3));
  this.geometry.setIndex(m.faces.flat());
  this.paper=new THREE.Mesh(this.geometry,new THREE.MeshStandardMaterial({color:0xffffff,vertexColors:true,side:THREE.DoubleSide,roughness:surface.roughness,metalness:surface.metalness,opacity:surface.opacity,transparent:surface.opacity<1,depthWrite:surface.opacity===1,forceSinglePass:true,flatShading:true,polygonOffset:true,polygonOffsetFactor:1,polygonOffsetUnits:1}));
  this.paper.castShadow=true;this.scene.add(this.paper);
  this.referenceWire=this.makeLines(m.edges.length,0x6c8ace,0.35);this.referenceWire.visible=false;this.scene.add(this.referenceWire);
  for(let i=0;i<Math.min(64,m.rest.length);i++){const arrow=new THREE.ArrowHelper(new THREE.Vector3(0,1,0),new THREE.Vector3(),.01,0xf27642,.004,.002);arrow.visible=false;this.flexArrows.push(arrow);this.scene.add(arrow);}
  this.wire=this.makeLines(m.edges.length,0x687c78,0.25);this.scene.add(this.wire);
  this.folds=this.makeLines(m.hinges.filter(h=>h.crease).length,0xb4793c,0.9);this.scene.add(this.folds);
  const pathGeo=new THREE.BufferGeometry();pathGeo.setAttribute('position',new THREE.Float32BufferAttribute(new Float32Array(1600*3),3));pathGeo.setDrawRange(0,0);
  this.path=new THREE.Line(pathGeo,new THREE.LineBasicMaterial({color:accent,transparent:true,opacity:0.6}));this.path.frustumCulled=false;this.scene.add(this.path);
  for(let i=0;i<16;i++){const a=new THREE.ArrowHelper(new THREE.Vector3(0,1,0),new THREE.Vector3(),0.05,0x118b78,0.01,0.005);this.arrows.push(a);this.scene.add(a);}
  this.velocity.setColor(0x3971bd);this.scene.add(this.velocity);
  this.scene.add(this.massMarker,this.liftMarker);
 }
 makeCenterMarker(label:string,color:string,side:number){
  const group=new THREE.Group();group.visible=false;
  const dot=new THREE.Mesh(new THREE.SphereGeometry(.0035,12,8),new THREE.MeshBasicMaterial({color,depthTest:false,depthWrite:false}));dot.renderOrder=1002;group.add(dot);
  const stem=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3(0,side*.035,0)]),new THREE.LineBasicMaterial({color,depthTest:false,depthWrite:false}));stem.renderOrder=1001;group.add(stem);
  const canvas=document.createElement('canvas');canvas.width=192;canvas.height=72;const ctx=canvas.getContext('2d')!;
  ctx.fillStyle='#fff9e9';ctx.fillRect(0,0,192,72);ctx.strokeStyle=color;ctx.lineWidth=8;ctx.strokeRect(4,4,184,64);ctx.fillStyle=color;ctx.font='bold 36px sans-serif';ctx.textAlign='center';ctx.fillText(label,96,49);
  const sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:new THREE.CanvasTexture(canvas),depthTest:false,depthWrite:false,toneMapped:false}));sprite.position.y=side*.045;sprite.scale.set(.048,.018,1);sprite.renderOrder=1003;group.add(sprite);return group;
 }
 makeLines(count:number,color:number,opacity:number){const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(new Float32Array(count*6),3));return new THREE.LineSegments(g,new THREE.LineBasicMaterial({color,transparent:true,opacity}));}
 update(options:ViewOptions,frame?:FlightFrame){
  const s=this.simulation,p=this.geometry.getAttribute('position') as THREE.BufferAttribute,c=this.geometry.getAttribute('color') as THREE.BufferAttribute;
  const positions=frame?s.positions.map((_,i)=>[frame.positions[i*3],frame.positions[i*3+1],frame.positions[i*3+2]] as [number,number,number]):s.positions;
  const center=frame?.center??s.center(),height=Math.max(0,center[1]);
  this.sun.target.position.set(...center);this.sun.position.copy(this.sun.target.position).add(new THREE.Vector3(2,10,3));
  this.sun.shadow.radius=Math.min(8,1+height*1.8);
  this.sun.shadow.intensity=materials[s.parameters.material].opacity*.85/(1+height*.12);
  this.massMarker.visible=options.massCenter&&!!frame;this.massMarker.position.set(...(frame?.center??s.center()));
  if((options.liftCenter||options.liftMeter||options.stallMeter)&&frame&&this.analyzedFrame!==frame){
   const velocities=positions.map((_,i)=>[frame.velocities[i*3],frame.velocities[i*3+1],frame.velocities[i*3+2]] as V3);
   const panels=aerodynamicLoads(positions,velocities,s.mesh.faces,s.aeroSettings(frame.time),positions.map(()=>[0,0,0] as V3));
   this.stallReading=stallSummary(panels);this.liftCenter=centerOfLift(panels);this.liftReading=liftSummary(panels,s.mesh.masses.reduce((sum,mass)=>sum+mass,0),s.parameters.gravity);this.analyzedFrame=frame;
  }
  this.liftMarker.visible=options.liftCenter&&!!frame&&!!this.liftCenter;if(this.liftCenter)this.liftMarker.position.set(...this.liftCenter);
  const sock=this.environment.getObjectByName('windsock');if(sock){const wind=new THREE.Vector3(...airVelocity(s.parameters,frame?.time??s.time,[0,2.9,-3.3]));if(wind.length()<.05)wind.set(0,-1,0);wind.normalize();sock.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),wind);sock.position.set(0,2.9,-3.3).addScaledVector(wind,.4);}
  const field=s.deformation(positions),deformation=field.perVertex,color=new THREE.Color();this.flexRms=field.rms;
  const layers=s.mesh.rest.map(()=>0),counts=layers.map(()=>0);if(options.layers)s.mesh.faces.forEach((f,i)=>f.forEach(v=>{layers[v]+=s.mesh.layerCounts?.[i]??1;counts[v]++;}));
  this.flexArrows.forEach((arrow,j)=>{const i=Math.floor(j*positions.length/this.flexArrows.length),magnitude=deformation[i];arrow.visible=options.flex&&magnitude>1e-6;if(!arrow.visible)return;arrow.position.set(...field.reference[i]);arrow.setDirection(new THREE.Vector3(...field.vectors[i]).normalize());const size=magnitude*options.flexScale;arrow.setLength(size,Math.min(.012,size*.25),Math.min(.006,size*.12));arrow.setColor(new THREE.Color().setHSL(.06-.06*Math.min(magnitude/.01,1),.9,.5));});
  this.referenceWire.visible=options.flex;const reference=this.referenceWire.geometry.getAttribute('position') as THREE.BufferAttribute;s.mesh.edges.forEach((e,i)=>{reference.setXYZ(2*i,...field.reference[e.a]);reference.setXYZ(2*i+1,...field.reference[e.b]);});reference.needsUpdate=true;this.referenceWire.geometry.computeBoundingSphere();
  positions.forEach((v,i)=>{p.setXYZ(i,...v);if(options.layers)color.setHSL(.48-.45*Math.min((layers[i]/Math.max(1,counts[i])-1)/7,1),.65,.6);else if(options.heat)color.setHSL(0.46-0.43*Math.min(deformation[i]/0.012,1),0.52,0.66);else color.setHex(s.parameters.material==='paper'?(s.mesh.regions[i]===0?0xc5cfc7:0xfffcf1):materials[s.parameters.material].color);c.setXYZ(i,color.r,color.g,color.b);});
  p.needsUpdate=true;c.needsUpdate=true;this.geometry.computeVertexNormals();this.geometry.computeBoundingSphere();
  const wire=this.wire.geometry.getAttribute('position') as THREE.BufferAttribute;
  s.mesh.edges.forEach((e,i)=>{wire.setXYZ(i*2,...positions[e.a]);wire.setXYZ(i*2+1,...positions[e.b]);});wire.needsUpdate=true;this.wire.geometry.computeBoundingSphere();this.wire.visible=options.wire;
  const folds=this.folds.geometry.getAttribute('position') as THREE.BufferAttribute;
  s.mesh.hinges.filter(h=>h.crease).forEach((h,i)=>{folds.setXYZ(i*2,...positions[h.ids[2]]);folds.setXYZ(i*2+1,...positions[h.ids[3]]);});folds.needsUpdate=true;this.folds.geometry.computeBoundingSphere();
  const path=this.path.geometry.getAttribute('position') as THREE.BufferAttribute;s.trail.forEach((v,i)=>{if(i<1600)path.setXYZ(i,...v);});path.needsUpdate=true;this.path.geometry.setDrawRange(0,Math.min(s.trail.length,1600));
  this.arrows.forEach((a,i)=>{const panel=s.panels[Math.floor(i*s.panels.length/16)];a.visible=options.forces&&!!panel&&length(panel.force)>1e-8;if(!a.visible||!panel)return;a.position.set(...panel.center);a.setDirection(new THREE.Vector3(...panel.force).normalize());a.setLength(Math.min(length(panel.force)*15,0.15),0.009,0.004);});
  const v=s.center(s.velocities);this.velocity.visible=options.velocity&&length(v)>1e-5;this.velocity.position.set(...s.center());if(this.velocity.visible){this.velocity.setDirection(new THREE.Vector3(...v).normalize());this.velocity.setLength(length(v)*0.025,0.018,0.008);}
 }
 dispose(){this.sun.shadow.dispose();this.scene.traverse(o=>{const x=o as THREE.Mesh;if(x.geometry)x.geometry.dispose();if(x.material){const mats=Array.isArray(x.material)?x.material:[x.material];mats.forEach(m=>{const map=(m as THREE.MeshStandardMaterial).map;map?.dispose();m.dispose();});}});}
}
export class Viewport {
 renderer:THREE.WebGLRenderer;camera=new THREE.PerspectiveCamera(40,1,0.005,250);controls:OrbitControls;
 views:PlaneView[]=[];options:ViewOptions={heat:false,wire:false,forces:false,velocity:false,follow:true,flex:false,flexScale:10,layers:false,massCenter:false,liftCenter:false,liftMeter:false,stallMeter:false};
 mode:'workbench'|'throw'='workbench';
 reviewing=false;reviewFollow=false;reviewFrame?:FlightFrame;private reviewDecor?:THREE.Group;private reviewTrailMaterial?:THREE.ShaderMaterial;
 private actual=new THREE.PerspectiveCamera(40,1,0.005,250);private observer:ResizeObserver;
 constructor(public element:HTMLElement){
  this.renderer=new THREE.WebGLRenderer({antialias:true,alpha:false,powerPreference:'high-performance'});this.renderer.shadowMap.enabled=true;this.renderer.shadowMap.type=THREE.PCFShadowMap;this.renderer.setPixelRatio(Math.min(devicePixelRatio,2));this.renderer.setClearColor('#e9eeeb');this.renderer.outputColorSpace=THREE.SRGBColorSpace;this.renderer.toneMapping=THREE.ACESFilmicToneMapping;this.renderer.toneMappingExposure=1;
  this.element.append(this.renderer.domElement);this.renderer.domElement.setAttribute('aria-label','Interactive 3D paper aeroplane. Drag to rotate and scroll to zoom.');
  this.controls=new OrbitControls(this.camera,this.renderer.domElement);this.controls.enableDamping=true;this.controls.dampingFactor=0.12;this.controls.minDistance=0.24;this.controls.maxDistance=40;this.controls.enablePan=false;this.controls.maxPolarAngle=Math.PI*0.94;
  this.home();this.observer=new ResizeObserver(()=>this.resize());this.observer.observe(element);this.resize();
 }
 home(){this.camera.position.set(0.22,0.25,0.29);this.controls.target.set(0,0,0);this.controls.update();}
 setSimulations(simulations:Simulation[]){this.endReview();this.views.forEach(v=>v.dispose());this.views=simulations.map((s,i)=>new PlaneView(s,i?0xb47634:0x157d6d));this.resize();}
 resize(){const w=this.element.clientWidth,h=this.element.clientHeight;if(w&&h)this.renderer.setSize(w,h);}
 beginReview(){
  if(this.reviewing)return;this.reviewing=true;this.controls.enabled=true;this.controls.enablePan=true;this.controls.maxDistance=120;this.controls.maxPolarAngle=Math.PI*0.49;
  const view=this.views[0],trail=view.simulation.trail,decor=new THREE.Group();this.reviewDecor=decor;view.scene.add(decor);
  // A screen-thin line stays unobtrusive when the camera gets close. Fade a
  // clearance around the recorded aircraft, rather than drawing through it.
  const center=view.simulation.center(),radius=Math.max(...view.simulation.positions.map(p=>new THREE.Vector3(...p).distanceTo(new THREE.Vector3(...center))))*1.15;
  this.reviewTrailMaterial=new THREE.ShaderMaterial({
   transparent:true,depthWrite:false,
   uniforms:{planeCenter:{value:new THREE.Vector3(...center)},clearance:{value:radius},trailColor:{value:new THREE.Color('#cf5a24')}},
   vertexShader:`varying vec3 worldPoint;
    void main(){worldPoint=(modelMatrix*vec4(position,1.0)).xyz;gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.0);}`,
   fragmentShader:`uniform vec3 planeCenter;uniform float clearance;uniform vec3 trailColor;varying vec3 worldPoint;
    void main(){float alpha=0.92*smoothstep(clearance,clearance*2.2,distance(worldPoint,planeCenter));gl_FragColor=vec4(trailColor,alpha);
    #include <colorspace_fragment>
    }`
  });
  decor.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(trail.map(p=>new THREE.Vector3(...p))),this.reviewTrailMaterial));
  const label=(text:string,point:typeof trail[number],color:string)=>{const c=document.createElement('canvas');c.width=256;c.height=96;const ctx=c.getContext('2d')!;ctx.fillStyle=color;ctx.fillRect(0,0,256,96);ctx.fillStyle='#243b49';ctx.font='bold 35px sans-serif';ctx.textAlign='center';ctx.fillText(text,128,61);const sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:new THREE.CanvasTexture(c),depthTest:false}));sprite.position.set(point[0],point[1]+0.45,point[2]);sprite.scale.set(1.1,0.41,1);decor.add(sprite);};
  label('TAKEOFF',trail[0],'#ffda55');
  if(view.simulation.touchdown)label('TOUCHDOWN',view.simulation.touchdown.center,'#ffad84');
  if(!view.simulation.touchdown||view.simulation.groundDistance>.5)label(view.simulation.status==='landed'?'REST':'END',trail.at(-1)!,'#91c7ba');
  this.fitReview();
 }
 endReview(){this.reviewing=false;this.reviewFollow=false;this.options.flex=false;this.options.layers=false;this.options.massCenter=false;this.options.liftCenter=false;this.options.liftMeter=false;this.options.stallMeter=false;this.reviewFrame=undefined;this.reviewDecor=undefined;this.reviewTrailMaterial=undefined;this.controls.enablePan=false;this.controls.maxDistance=40;this.controls.maxPolarAngle=Math.PI*0.94;this.controls.enabled=this.mode==='workbench';this.home();}
 fitReview(){
  if(!this.reviewing)return;this.reviewFollow=false;const bounds=new THREE.Box3().setFromPoints(this.views[0].simulation.trail.map(p=>new THREE.Vector3(...p))),target=bounds.getCenter(new THREE.Vector3()),radius=Math.max(0.8,bounds.getSize(new THREE.Vector3()).length()/2);
  const aspect=this.element.clientWidth/this.element.clientHeight,angle=Math.min(THREE.MathUtils.degToRad(20),Math.atan(Math.tan(THREE.MathUtils.degToRad(20))*aspect));
  const distance=radius/Math.sin(angle)*1.3;this.controls.target.copy(target);this.camera.position.copy(target).add(new THREE.Vector3(-0.6,0.65,1).normalize().multiplyScalar(distance));this.controls.update();
 }
 focusReview(point:'start'|'end'|'plane',distance=2.5){
  if(!this.reviewing)return;if(point!=='plane')this.reviewFollow=false;const s=this.views[0].simulation,p=point==='start'?s.start:point==='end'?s.center():this.reviewFrame?.center??s.center();const portraitScale=Math.max(1,.8/(this.element.clientWidth/this.element.clientHeight));const offset=this.camera.position.clone().sub(this.controls.target).normalize().multiplyScalar(distance*portraitScale);this.controls.target.set(...p);this.camera.position.copy(this.controls.target).add(offset);this.controls.update();
 }
 setReviewFrame(frame:FlightFrame){if(this.reviewFollow&&this.reviewFrame){const delta=new THREE.Vector3(...frame.center).sub(new THREE.Vector3(...this.reviewFrame.center));this.controls.target.add(delta);this.camera.position.add(delta);}this.reviewFrame=frame;}
 moveReview(forward:number,right:number,up:number,dt:number){if(!this.reviewing)return;const direction=this.controls.target.clone().sub(this.camera.position);direction.y=0;direction.normalize();const side=new THREE.Vector3().crossVectors(direction,new THREE.Vector3(0,1,0));const distance=Math.max(1,this.camera.position.distanceTo(this.controls.target)*0.5)*Math.min(dt,0.05);const delta=direction.multiplyScalar(forward*distance).addScaledVector(side,right*distance).add(new THREE.Vector3(0,up*distance,0));this.camera.position.add(delta);this.controls.target.add(delta);}
 launcherPoint(){const s=this.views[0].simulation,p=new THREE.Vector3(...s.start).project(this.actual);return {x:(p.x+1)*this.element.clientWidth/2,y:(1-p.y)*this.element.clientHeight/2};}
 previewPull(dx:number,dy:number){const s=this.views[0].simulation,p=new THREE.Vector3(...s.start).project(this.actual);p.x+=2*dx/this.element.clientWidth;p.y-=2*dy/this.element.clientHeight;p.unproject(this.actual).sub(new THREE.Vector3(...s.start));for(const v of s.positions){v[0]+=p.x;v[1]+=p.y;v[2]+=p.z;}}
 render(){
  const w=this.element.clientWidth,h=this.element.clientHeight;if(!w||!h)return;this.controls.update();this.renderer.setScissorTest(true);
  const n=this.views.length;
  this.views.forEach((view,i)=>{
    view.environment.visible=this.mode==='throw';view.scene.background=new THREE.Color(this.mode==='throw'?'#c5dfdb':'#e7e8db');view.update(this.options,this.reviewing?this.reviewFrame:undefined);const width=w/n,center=view.simulation.center();
    if(this.reviewDecor)this.reviewDecor.visible=!(this.options.flex||this.options.layers);if(this.reviewTrailMaterial)this.reviewTrailMaterial.uniforms.planeCenter.value.set(...(this.reviewFrame?.center??center));view.path.visible=!this.reviewing;
    this.actual.aspect=width/h;
    if(this.reviewing){this.actual.position.copy(this.camera.position);this.actual.quaternion.copy(this.camera.quaternion);}
    else if(this.mode==='throw'){
      const target=new THREE.Vector3(...(view.simulation.status==='ready'?view.simulation.start:center));
      if(this.options.follow){this.actual.position.copy(target).add(new THREE.Vector3(-1.5,0.6,0));this.actual.lookAt(target.clone().add(new THREE.Vector3(0.5,0,0)));}
      else {this.actual.position.set(-2.6,2.6,1.2);this.actual.lookAt(target);}
    }
    else if(this.options.follow){this.actual.position.copy(this.camera.position).multiplyScalar(Math.max(1,1/this.actual.aspect)).add(new THREE.Vector3(...center));this.actual.lookAt(...center);}
    else {const target=new THREE.Vector3(Math.max(3,center[0]/2),0.8,0);const offset=this.camera.position.clone().normalize().multiplyScalar(Math.max(7,center[0]*0.8));this.actual.position.copy(target).add(offset);this.actual.lookAt(target);}
    this.actual.updateProjectionMatrix();this.renderer.setViewport(i*width,0,width,h);this.renderer.setScissor(i*width,0,width,h);this.renderer.render(view.scene,this.actual);
  });this.renderer.setScissorTest(false);
 }
}
