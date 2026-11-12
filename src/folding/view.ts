import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { materials, type MaterialId } from '../physics/materials';
import { FoldSequence } from './animation';
import { cross, sub, unit, type V3 } from '../physics/math';

/** Orbitable workbench paper. Only the active panels rotate; the camera stays put. */
export class FoldingView {
 readonly renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});
 readonly scene=new THREE.Scene();readonly camera=new THREE.PerspectiveCamera(38,1,.001,10);
 readonly controls:OrbitControls;
 private paper=new THREE.Group();private sequence?:FoldSequence;private radius=.25;
 private target=new THREE.Vector3();private moved=false;private pointers=new Map<number,{x:number;y:number}>();
 private observer:ResizeObserver;
 constructor(readonly element:HTMLElement){
  this.renderer.setPixelRatio(Math.min(devicePixelRatio,2));this.renderer.setClearColor(0,0);element.append(this.renderer.domElement);
  this.renderer.domElement.setAttribute('aria-label','3D folding paper. Drag to orbit, scroll or pinch to zoom. Tap to make the next fold.');
  this.scene.add(new THREE.HemisphereLight(0xffffff,0x65887e,2.4));const light=new THREE.DirectionalLight(0xffffff,3);light.position.set(-.5,.7,1);this.scene.add(light,this.paper);
  this.controls=new OrbitControls(this.camera,this.renderer.domElement);this.controls.enableDamping=true;this.controls.enablePan=false;this.controls.minDistance=.18;this.controls.maxDistance=2;
  this.camera.position.set(0,0,.75);
  // Pointer travel distinguishes inspecting the model from tapping to fold.
  element.addEventListener('pointerdown',e=>{if(!this.pointers.size)this.moved=false;this.pointers.set(e.pointerId,{x:e.clientX,y:e.clientY});if(this.pointers.size>1)this.moved=true;},true);
  element.addEventListener('pointermove',e=>{const start=this.pointers.get(e.pointerId);if(start&&Math.hypot(e.clientX-start.x,e.clientY-start.y)>5)this.moved=true;},true);
  element.addEventListener('pointerup',e=>this.pointers.delete(e.pointerId),true);
  element.addEventListener('pointercancel',e=>{this.pointers.delete(e.pointerId);this.moved=true;},true);
  element.addEventListener('click',e=>{if(this.moved){e.stopImmediatePropagation();e.preventDefault();}},true);
  this.observer=new ResizeObserver(()=>this.resize());this.observer.observe(element);this.resize();
 }
 // Match the flight model: nose-to-tail is X, vertical is Y, wingspan is Z.
 // OrbitControls turns around Y, so horizontal drags yaw rather than roll the dart.
 private point(p:V3){return new THREE.Vector3(this.sequence!.recipe.height/2-p[1],p[0],p[2]);}
 setSequence(sequence:FoldSequence){
  const first=!this.sequence;this.sequence=sequence;
  const box=new THREE.Box3();
  for(let progress=0;progress<=sequence.recipe.operations.length;progress+=.25)
   for(const panel of sequence.sample(progress).panels)for(const p of panel.points)box.expandByPoint(this.point(p.position));
  box.getCenter(this.target);this.radius=box.getSize(new THREE.Vector3()).length()/2*1.15;
  if(first)this.home();
 }
 home(){
  const halfFov=THREE.MathUtils.degToRad(this.camera.fov/2),angle=Math.min(halfFov,Math.atan(Math.tan(halfFov)*this.camera.aspect));
  // Clear any remaining orbit momentum before restoring the sheet-normal view.
  const damping=this.controls.enableDamping;this.controls.enableDamping=false;this.controls.update();
  this.controls.target.copy(this.target);this.camera.position.copy(this.target).add(new THREE.Vector3(0,0,this.radius/Math.sin(angle)*1.05));this.controls.update();this.controls.enableDamping=damping;
 }
 private resize(){const w=this.element.clientWidth,h=this.element.clientHeight;if(!w||!h)return;this.renderer.setSize(w,h);this.camera.aspect=w/h;this.camera.updateProjectionMatrix();}
 draw(progress:number,size:number,gsm:number,active?:number,materialId:MaterialId='paper'){
  if(!this.sequence)return;
  this.paper.traverse(o=>{const mesh=o as THREE.Mesh;mesh.geometry?.dispose();if(mesh.material)(Array.isArray(mesh.material)?mesh.material:[mesh.material]).forEach(m=>m.dispose());});this.paper.clear();this.paper.scale.setScalar(size);
  const state=this.sequence.sample(progress),surface=materials[materialId];
  state.panels.forEach((panel,i)=>{
   const points=panel.points.map(p=>this.point(p.position)),positions:number[]=[];
   for(let j=1;j<points.length-1;j++)positions.push(...points[0].toArray(),...points[j].toArray(),...points[j+1].toArray());
   const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));geometry.computeVertexNormals();
   const material=new THREE.MeshStandardMaterial({color:materialId==='paper'?(gsm===110?0xe5d3ae:gsm===60?0xfffef6:i%2?0xf5ead4:0xfff7e6):surface.color,side:THREE.DoubleSide,roughness:surface.roughness,metalness:surface.metalness,flatShading:true,polygonOffset:true,polygonOffsetFactor:1+i*.15,polygonOffsetUnits:1+i*.15});
   this.paper.add(new THREE.Mesh(geometry,material));
   this.paper.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([...points,points[0]]),new THREE.LineBasicMaterial({color:0x62796d,transparent:true,opacity:.55})));
  });
  if(active===undefined){
   // Keep the orbit pivot on the current paper after a step finishes, preserving
   // the viewing direction and zoom. During motion the pivot stays stationary.
   const center=new THREE.Box3().setFromObject(this.paper).getCenter(new THREE.Vector3());
   this.camera.position.add(center.clone().sub(this.controls.target));this.controls.target.copy(center);this.target.copy(center);
  }
  if(active!==undefined){
   const operation=this.sequence.recipe.operations[active],axis=unit(sub(operation.axis[1],operation.axis[0]));
   const crease=this.sequence.sample(active+.00001).panels.flatMap(p=>p.points).filter(p=>Math.hypot(...cross(sub(p.position,operation.axis[0]),axis))<1e-7).map(p=>this.point(p.position));
   if(crease.length>1){const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints(crease),new THREE.LineBasicMaterial({color:0xee7845,depthTest:false,depthWrite:false,toneMapped:false}));line.renderOrder=5;this.paper.add(line);}
  }
 }
 render(){this.controls.update();this.renderer.render(this.scene,this.camera);}
}
