import type { PaperMesh, Face } from '../geometry/dart';
import type { SheetMaterial } from './sheet-material';
import { sub, dot, cross, length, scale, type V3 } from './math';
import { kernel, F64, I32 } from './solver';
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
/** Stage a membrane batch for a solver kernel. */
export function stageMembranes(batch:MembraneBatch){
 kernel.i32(I32.faces,batch.faces);kernel.f64(F64.md,batch.md);kernel.f64(F64.cd,batch.cd);kernel.f64(F64.a,batch.alpha);kernel.f64(F64.b,batch.lambda);
}
/** Diagonal orthotropic membrane energy: Et A/2 times squared Green normal
 * strain, Gt A/2 times squared engineering shear. Poisson membrane coupling is
 * omitted; explicit shear prevents skinny triangles collapsing like bar nets.
 * One XPBD sweep over flat xyz coordinates; `alpha` is compliance/dt^2 per mode. */
export function solveMembranes(x:Float64Array,w:ArrayLike<number>,batch:MembraneBatch,forward:boolean){
 stageMembranes(batch);kernel.f64(F64.x,x);kernel.f64(F64.w,w);
 kernel.call.membranes_solve(forward?1:0);
 x.set(kernel.read64(F64.x));batch.lambda.set(kernel.read64(F64.b));
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
