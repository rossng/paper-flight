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
/** Diagonal orthotropic membrane energy: Et A/2 times squared Green normal
 * strain, Gt A/2 times squared engineering shear. Poisson membrane coupling is
 * omitted; explicit shear prevents skinny triangles collapsing like bar nets. */
export function solveMembrane(p:V3[],w:number[],triangle:Membrane,m:SheetMaterial,dt:number,preparedAlpha?:V3){
 const [ia,ib,ic]=triangle.face,a=p[ia],b=p[ib],c=p[ic],md=triangle.md,cd=triangle.cd;
 for(let mode=0;mode<3;mode++){
  const bx=b[0]-a[0],by=b[1]-a[1],bz=b[2]-a[2],cx=c[0]-a[0],cy=c[1]-a[1],cz=c[2]-a[2];
  // Each normal mode needs only its own material derivative. Recompute it
  // after the preceding Gauss-Seidel correction; shear still needs both.
  const direction=mode===1?cd:md,normal=mode<2;
  const ux=direction[1]*bx+direction[2]*cx,uy=direction[1]*by+direction[2]*cy,uz=direction[1]*bz+direction[2]*cz;
  const vx=normal?0:cd[1]*bx+cd[2]*cx,vy=normal?0:cd[1]*by+cd[2]*cy,vz=normal?0:cd[1]*bz+cd[2]*cz;
  const C=normal?(ux*ux+uy*uy+uz*uz-1)/2:ux*vx+uy*vy+uz*vz;
  const modulus=mode===0?m.youngMD:mode===1?m.youngCD:m.shear;
  const alpha=preparedAlpha?.[mode]??1/(modulus*m.thickness*triangle.area*dt*dt);
  const g0x=normal?direction[0]*ux:md[0]*vx+cd[0]*ux;
  const g0y=normal?direction[0]*uy:md[0]*vy+cd[0]*uy;
  const g0z=normal?direction[0]*uz:md[0]*vz+cd[0]*uz;
  const g1x=normal?direction[1]*ux:md[1]*vx+cd[1]*ux;
  const g1y=normal?direction[1]*uy:md[1]*vy+cd[1]*uy;
  const g1z=normal?direction[1]*uz:md[1]*vz+cd[1]*uz;
  const g2x=normal?direction[2]*ux:md[2]*vx+cd[2]*ux;
  const g2y=normal?direction[2]*uy:md[2]*vy+cd[2]*uy;
  const g2z=normal?direction[2]*uz:md[2]*vz+cd[2]*uz;
  const den=alpha+w[ia]*(g0x*g0x+g0y*g0y+g0z*g0z)+w[ib]*(g1x*g1x+g1y*g1y+g1z*g1z)+w[ic]*(g2x*g2x+g2y*g2y+g2z*g2z);
  const delta=(-C-alpha*triangle.lambda[mode])/den;triangle.lambda[mode]+=delta;
  const wa=w[ia]*delta,wb=w[ib]*delta,wc=w[ic]*delta;
  a[0]+=wa*g0x;a[1]+=wa*g0y;a[2]+=wa*g0z;
  b[0]+=wb*g1x;b[1]+=wb*g1y;b[2]+=wb*g1z;
  c[0]+=wc*g2x;c[1]+=wc*g2y;c[2]+=wc*g2z;
 }
}
