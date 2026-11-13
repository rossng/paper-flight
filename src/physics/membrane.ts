import type { PaperMesh, Face } from '../geometry/dart';
import type { SheetMaterial } from './sheet-material';
import { sub, dot, cross, length, scale, type V3 } from './math';
export interface Membrane { face:Face; md:V3; cd:V3; area:number; lambda:V3 }
/** Constant reference shape gradients in orthogonal material directions. A
 * triangle-local metric preserves the exact folded rest shape, including the
 * effective dart, which has no literal flat-sheet parametrization. */
export function buildMembranes(mesh:PaperMesh):Membrane[]{
 return mesh.faces.map(face=>{
  const [a,b,c]=face.map(i=>mesh.rest[i]),ab=sub(b,a),ac=sub(c,a),l=length(ab),x=dot(ac,scale(ab,1/l)),y=length(cross(ab,ac))/l;
  const gx:V3=[-1/l,1/l,0],gy:V3=[(x-l)/(l*y),-x/(l*y),1/y];
  const uv=face.map(i=>mesh.materialUV?.[i]??[mesh.rest[i][2],mesh.rest[i][0]]);
  const du=uv[1][0]-uv[0][0],dv=uv[1][1]-uv[0][1],ul=Math.hypot(du,dv);
  const orientation=Math.sign(du*(uv[2][1]-uv[0][1])-dv*(uv[2][0]-uv[0][0]))||1;
  const cos=ul>1e-12?dv/ul:1,sin=ul>1e-12?du/ul:0;
  const md=gx.map((v,i)=>cos*v+orientation*sin*gy[i]) as V3,cd=gx.map((v,i)=>sin*v-orientation*cos*gy[i]) as V3;
  return {face,md,cd,area:l*y/2,lambda:[0,0,0]};
 });
}
/** Recomputed each outer step: dt and live material overrides may change. */
export function membraneAlphas(triangle:Membrane,m:SheetMaterial,dt:number):V3 {
 return [m.youngMD,m.youngCD,m.shear].map(modulus=>1/(modulus*m.thickness*triangle.area*dt*dt)) as V3;
}
export interface MembraneBatch { faces:Int32Array; md:Float64Array; cd:Float64Array; alpha:Float64Array; lambda:Float64Array }
export function membraneBatch(triangles:Membrane[]):MembraneBatch {
 return {faces:Int32Array.from(triangles.flatMap(t=>t.face)),md:Float64Array.from(triangles.flatMap(t=>t.md)),cd:Float64Array.from(triangles.flatMap(t=>t.cd)),
  alpha:new Float64Array(triangles.length*3),lambda:new Float64Array(triangles.length*3)};
}
/** Diagonal orthotropic membrane energy: Et A/2 times squared Green normal
 * strain, Gt A/2 times squared engineering shear. Poisson membrane coupling is
 * omitted; explicit shear prevents skinny triangles collapsing like bar nets.
 * One XPBD sweep over flat xyz coordinates; `alpha` is compliance/dt^2 per mode. */
export function solveMembranes(x:Float64Array,w:ArrayLike<number>,batch:MembraneBatch,forward:boolean){
 const {faces,md,cd,alpha,lambda}=batch,count=faces.length/3;
 for(let k=0;k<count;k++){
  const t=3*(forward?k:count-1-k),ia=faces[t],ib=faces[t+1],ic=faces[t+2],a=3*ia,b=3*ib,c=3*ic,wa=w[ia],wb=w[ib],wc=w[ic];
  let ax=x[a],ay=x[a+1],az=x[a+2],bx=x[b],by=x[b+1],bz=x[b+2],cx=x[c],cy=x[c+1],cz=x[c+2];
  const m0=md[t],m1=md[t+1],m2=md[t+2],c0=cd[t],c1=cd[t+1],c2=cd[t+2];
  // Each normal mode needs only its own material derivative. Recompute it
  // after the preceding Gauss-Seidel correction; shear still needs both.
  for(let mode=0;mode<2;mode++){
   const d0=mode?c0:m0,d1=mode?c1:m1,d2=mode?c2:m2;
   const ex=bx-ax,ey=by-ay,ez=bz-az,fx=cx-ax,fy=cy-ay,fz=cz-az;
   const ux=d1*ex+d2*fx,uy=d1*ey+d2*fy,uz=d1*ez+d2*fz;
   const C=(ux*ux+uy*uy+uz*uz-1)/2,stiffness=alpha[t+mode];
   const g0x=d0*ux,g0y=d0*uy,g0z=d0*uz,g1x=d1*ux,g1y=d1*uy,g1z=d1*uz,g2x=d2*ux,g2y=d2*uy,g2z=d2*uz;
   const den=stiffness+wa*(g0x*g0x+g0y*g0y+g0z*g0z)+wb*(g1x*g1x+g1y*g1y+g1z*g1z)+wc*(g2x*g2x+g2y*g2y+g2z*g2z);
   const delta=(-C-stiffness*lambda[t+mode])/den;lambda[t+mode]+=delta;
   const sa=wa*delta,sb=wb*delta,sc=wc*delta;
   ax+=sa*g0x;ay+=sa*g0y;az+=sa*g0z;bx+=sb*g1x;by+=sb*g1y;bz+=sb*g1z;cx+=sc*g2x;cy+=sc*g2y;cz+=sc*g2z;
  }
  const ex=bx-ax,ey=by-ay,ez=bz-az,fx=cx-ax,fy=cy-ay,fz=cz-az;
  const ux=m1*ex+m2*fx,uy=m1*ey+m2*fy,uz=m1*ez+m2*fz,vx=c1*ex+c2*fx,vy=c1*ey+c2*fy,vz=c1*ez+c2*fz;
  const C=ux*vx+uy*vy+uz*vz,stiffness=alpha[t+2];
  const g0x=m0*vx+c0*ux,g0y=m0*vy+c0*uy,g0z=m0*vz+c0*uz;
  const g1x=m1*vx+c1*ux,g1y=m1*vy+c1*uy,g1z=m1*vz+c1*uz;
  const g2x=m2*vx+c2*ux,g2y=m2*vy+c2*uy,g2z=m2*vz+c2*uz;
  const den=stiffness+wa*(g0x*g0x+g0y*g0y+g0z*g0z)+wb*(g1x*g1x+g1y*g1y+g1z*g1z)+wc*(g2x*g2x+g2y*g2y+g2z*g2z);
  const delta=(-C-stiffness*lambda[t+2])/den;lambda[t+2]+=delta;
  const sa=wa*delta,sb=wb*delta,sc=wc*delta;
  x[a]=ax+sa*g0x;x[a+1]=ay+sa*g0y;x[a+2]=az+sa*g0z;
  x[b]=bx+sb*g1x;x[b+1]=by+sb*g1y;x[b+2]=bz+sb*g1z;
  x[c]=cx+sc*g2x;x[c+1]=cy+sc*g2y;x[c+2]=cz+sc*g2z;
 }
}
const single:MembraneBatch={faces:Int32Array.of(0,1,2),md:new Float64Array(3),cd:new Float64Array(3),alpha:new Float64Array(3),lambda:new Float64Array(3)};
const singleX=new Float64Array(9),singleW=new Float64Array(3);
/** One triangle on per-vertex arrays; the same kernel as the batched sweep. */
export function solveMembrane(p:V3[],w:number[],triangle:Membrane,m:SheetMaterial,dt:number,preparedAlpha?:V3){
 const alpha=preparedAlpha??membraneAlphas(triangle,m,dt);
 for(let j=0;j<3;j++){
  const vertex=p[triangle.face[j]];singleX[3*j]=vertex[0];singleX[3*j+1]=vertex[1];singleX[3*j+2]=vertex[2];
  singleW[j]=w[triangle.face[j]];single.md[j]=triangle.md[j];single.cd[j]=triangle.cd[j];single.alpha[j]=alpha[j];single.lambda[j]=triangle.lambda[j];
 }
 solveMembranes(singleX,singleW,single,true);
 for(let j=0;j<3;j++){
  const vertex=p[triangle.face[j]];vertex[0]=singleX[3*j];vertex[1]=singleX[3*j+1];vertex[2]=singleX[3*j+2];triangle.lambda[j]=single.lambda[j];
 }
}
