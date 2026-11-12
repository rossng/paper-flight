import * as THREE from 'three';
import { GROUND_HEIGHT } from '../physics/ground-contact';

/** One metre per tile: small stationary details reveal motion even when the
 * follow camera keeps the aircraft in the same place on screen. */
function groundTexture(grass:boolean){
 const canvas=document.createElement('canvas');canvas.width=512;canvas.height=512;
 const ctx=canvas.getContext('2d')!;
 let seed=grass?419:811;
 const random=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296;};
 ctx.fillStyle=grass?'#86b572':'#d9bf8b';ctx.fillRect(0,0,512,512);
 for(let i=0;i<1600;i++){
  const x=random()*512,y=random()*512,size=.5+random()*1.5;
  ctx.fillStyle=grass?(i%3?'rgba(55,97,45,.3)':'rgba(221,231,160,.4)'):(i%3?'rgba(116,88,48,.3)':'rgba(255,242,208,.55)');
  if(grass)ctx.fillRect(x,y,size,2+random()*6);
  else {ctx.beginPath();ctx.ellipse(x,y,size,size*.65,random()*Math.PI,0,Math.PI*2);ctx.fill();}
 }
 // Occasional larger flecks remain legible from the live follow camera.
 for(let i=0;i<70;i++){
  ctx.fillStyle=grass?'rgba(67,107,46,.18)':'rgba(120,96,61,.22)';
  ctx.beginPath();ctx.ellipse(random()*512,random()*512,2+random()*3,1+random()*2,random()*Math.PI,0,Math.PI*2);ctx.fill();
 }
 if(!grass){
  // Faint metre seams and quarter-metre survey crosses provide a scale near
  // the plane; these are surface markings, not physical bumps or obstacles.
  ctx.fillStyle='rgba(120,96,61,.18)';ctx.fillRect(0,0,2,512);
  ctx.strokeStyle='rgba(120,96,61,.35)';ctx.lineWidth=1.5;ctx.beginPath();
  for(let x=64;x<512;x+=128)for(let y=64;y<512;y+=128){ctx.moveTo(x-4,y);ctx.lineTo(x+4,y);ctx.moveTo(x,y-4);ctx.lineTo(x,y+4);}
  ctx.stroke();
 }
 const texture=new THREE.CanvasTexture(canvas);texture.wrapS=texture.wrapT=THREE.RepeatWrapping;
 texture.colorSpace=THREE.SRGBColorSpace;texture.anisotropy=8;return texture;
}
function textureGround(surface:THREE.Mesh<THREE.BufferGeometry,THREE.MeshStandardMaterial>,grass:boolean){
 const positions=surface.geometry.getAttribute('position'),uv=new Float32Array(positions.count*2);
 // Both geometries lie in local XY before their -90 degree X rotation. Use
 // world X/Z coordinates so their very different extents cannot stretch grain.
 for(let i=0;i<positions.count;i++){uv[2*i]=positions.getX(i)+surface.position.x;uv[2*i+1]=-positions.getY(i)+surface.position.z;}
 surface.geometry.setAttribute('uv',new THREE.BufferAttribute(uv,2));
 surface.material.color.set(0xffffff);surface.material.map=groundTexture(grass);
}
/** Metre-scale landmarks make real displacement legible without scaling the aircraft. */
export function landscape(){
 const group=new THREE.Group();
 const mesh=(geometry:THREE.BufferGeometry,color:number,x:number,y:number,z:number)=>{const m=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color,flatShading:true,roughness:1}));m.position.set(x,y,z);group.add(m);return m;};
 // The grass has a runway-shaped opening: coplanar neighbours cannot
 // z-fight, even from distant or shallow replay camera angles.
 const ground=new THREE.Shape();ground.moveTo(-60,-90);ground.lineTo(120,-90);ground.lineTo(120,90);ground.lineTo(-60,90);ground.closePath();
 const opening=new THREE.Path();opening.moveTo(-10,-2.5);opening.lineTo(-10,2.5);opening.lineTo(80,2.5);opening.lineTo(80,-2.5);opening.closePath();ground.holes.push(opening);
 const grass=mesh(new THREE.ShapeGeometry(ground),0x86b572,0,GROUND_HEIGHT,0);grass.rotation.x=-Math.PI/2;grass.receiveShadow=true;
 const runway=mesh(new THREE.PlaneGeometry(90,5),0xd9bf8b,35,GROUND_HEIGHT,0);runway.rotation.x=-Math.PI/2;runway.receiveShadow=true;
 textureGround(grass,true);textureGround(runway,false);
 for(let i=0;i<56;i++){
  const x=(i*7.31)%100-8,z=(i%2?1:-1)*(4+(i*3.17)%25),h=1.4+(i*0.71)%2.3;
  mesh(new THREE.CylinderGeometry(0.08,0.13,h*0.5,5),0x8b7352,x,h*0.25,z);
  mesh(new THREE.ConeGeometry(h*0.45,h,5),i%3?0x6c966b:0x95ac71,x,h*0.85,z);
 }
 for(let i=0;i<12;i++)mesh(new THREE.IcosahedronGeometry(8+i%4,0),i%2?0x9cac8b:0xb0bc96,15+i*7,2, (i%2?1:-1)*(30+i%3*5)).scale.set(1,0.6,1.4);
 for(let i=1;i<=12;i++){
  const x=i*5;
  for(const z of [-2.7,2.7]){
   mesh(new THREE.CylinderGeometry(0.025,0.025,0.6,5),0xa88a63,x,0.3,z);
   const c=document.createElement('canvas');c.width=256;c.height=128;const ctx=c.getContext('2d')!;ctx.fillStyle='#ffe3a1';ctx.fillRect(0,0,256,128);ctx.fillStyle='#243b49';ctx.font='bold 58px sans-serif';ctx.textAlign='center';ctx.fillText(`${x} m`,128,85);
   const sign=new THREE.Sprite(new THREE.SpriteMaterial({map:new THREE.CanvasTexture(c)}));sign.position.set(x,0.8,z);sign.scale.set(0.8,0.4,1);group.add(sign);
   const flagGeometry=new THREE.BufferGeometry();flagGeometry.setAttribute('position',new THREE.Float32BufferAttribute([0,0,0,0,0.4,0,0.5,0.2,0],3));const flag=new THREE.Mesh(flagGeometry,new THREE.MeshBasicMaterial({color:i%2?0xf0825c:0xffce54,side:THREE.DoubleSide}));flag.position.set(x,1,z);group.add(flag);
  }
 }
 for(let i=0;i<12;i++){
  const cloud=new THREE.Group();for(let j=0;j<3;j++){const puff=new THREE.Mesh(new THREE.IcosahedronGeometry(1.4,1),new THREE.MeshBasicMaterial({color:0xfff8e7}));puff.position.set(j*1.4,Math.sin(j)*0.5,0);cloud.add(puff);}cloud.position.set(i*10-15,12+i%3*2,(i%2?1:-1)*15);group.add(cloud);
 }
 // A little windsock at the launch clearing.
 mesh(new THREE.CylinderGeometry(0.035,0.035,3,6),0xf3e8ce,0,1.5,-3.3);
 const sock=mesh(new THREE.CylinderGeometry(0.12,0.22,0.8,8),0xd88159,0.4,2.9,-3.3);sock.name='windsock';sock.rotation.z=Math.PI/2;
 return group;
}
