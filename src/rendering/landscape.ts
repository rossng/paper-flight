import * as THREE from 'three';
/** Metre-scale landmarks make real displacement legible without scaling the aircraft. */
export function landscape(){
 const group=new THREE.Group();
 const mesh=(geometry:THREE.BufferGeometry,color:number,x:number,y:number,z:number)=>{const m=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color,flatShading:true,roughness:1}));m.position.set(x,y,z);group.add(m);return m;};
 mesh(new THREE.PlaneGeometry(180,180),0x86b572,30,-0.02,0).rotation.x=-Math.PI/2;
 mesh(new THREE.PlaneGeometry(90,5),0xd9bf8b,35,-0.012,0).rotation.x=-Math.PI/2;
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
