import { cross, dot, length, scale, sub, type V3 } from './math';
import { type Edge, type Hinge } from '../geometry/dart';
export function solveEdge(p: V3[], w: number[], e: Edge, compliance: number, dt: number) {
  const a=p[e.a],b=p[e.b],dx=a[0]-b[0],dy=a[1]-b[1],dz=a[2]-b[2],l=Math.hypot(dx,dy,dz);
  if(l<1e-12)return;
  const alpha=compliance/(dt*dt),dl=(-(l-e.rest)-alpha*e.lambda)/(w[e.a]+w[e.b]+alpha);
  e.lambda+=dl;
  const sa=dl*w[e.a]/l,sb=dl*w[e.b]/l;
  a[0]+=sa*dx;a[1]+=sa*dy;a[2]+=sa*dz;
  b[0]-=sb*dx;b[1]-=sb*dy;b[2]-=sb*dz;
}
/** Signed-angle gradients adapted from MIT-licensed PositionBasedDynamics; see THIRD_PARTY.md. */
export function hingeGradients(p0: V3,p1: V3,p2: V3,p3: V3): V3[] | null {
  const e=sub(p3,p2),l=length(e);
  const raw1=cross(sub(p2,p0),sub(p3,p0)),raw2=cross(sub(p3,p1),sub(p2,p1));
  const a=dot(raw1,raw1),b=dot(raw2,raw2);
  if(l<1e-10 || a<1e-18 || b<1e-18)return null;
  const n1=scale(raw1,1/a),n2=scale(raw2,1/b);
  const g0=scale(n1,-l),g1=scale(n2,-l),g2: V3=[0,0,0],g3: V3=[0,0,0];
  const a2=-dot(sub(p0,p3),e)/l,b2=-dot(sub(p1,p3),e)/l;
  const a3=-dot(sub(p2,p0),e)/l,b3=-dot(sub(p2,p1),e)/l;
  for(let d=0;d<3;d++){g2[d]=a2*n1[d]+b2*n2[d];g3[d]=a3*n1[d]+b3*n2[d];}
  return [g0,g1,g2,g3];
}
/** Structure-of-arrays constraint data. Hot loops read flat xyz coordinates
 * (three per vertex) instead of chasing one small array per vertex. */
export interface HingeBatch { ids:Int32Array; rest:Float64Array; offset:Float64Array; targetSin:Float64Array; targetCos:Float64Array; alpha:Float64Array; lambda:Float64Array }
export interface EdgeBatch { ids:Int32Array; rest:Float64Array }
export type Weights=ArrayLike<number>;
export function hingeBatch(hinges:Hinge[]):HingeBatch {
  return {ids:Int32Array.from(hinges.flatMap(h=>h.ids)),rest:Float64Array.from(hinges.map(h=>h.rest)),
    offset:new Float64Array(hinges.length).fill(NaN),targetSin:new Float64Array(hinges.length),targetCos:new Float64Array(hinges.length),
    alpha:new Float64Array(hinges.length),lambda:new Float64Array(hinges.length)};
}
/** Set hinge j's permanent (plastic) offset from its rest angle. */
export function setHingeOffset(batch:HingeBatch,j:number,offset:number){
  if(batch.offset[j]===offset)return;
  batch.offset[j]=offset;batch.targetSin[j]=Math.sin(batch.rest[j]+offset);batch.targetCos[j]=Math.cos(batch.rest[j]+offset);
}
export function edgeBatch(edges:Edge[]):EdgeBatch {
  return {ids:Int32Array.from(edges.flatMap(e=>[e.a,e.b])),rest:Float64Array.from(edges.map(e=>e.rest))};
}
/** One XPBD sweep over every hinge: same signed gradients as hingeGradients,
 * without temporary vectors. `alpha` is compliance/dt^2 per hinge. */
export function solveHinges(x:Float64Array,w:Weights,batch:HingeBatch,forward:boolean) {
  const {ids,targetSin,targetCos,alpha,lambda}=batch,count=lambda.length;
  for(let k=0;k<count;k++){
    const j=forward?k:count-1-k,i0=ids[4*j],i1=ids[4*j+1],i2=ids[4*j+2],i3=ids[4*j+3],a=3*i0,b=3*i1,c=3*i2,d=3*i3;
    const ax=x[a],ay=x[a+1],az=x[a+2],bx=x[b],by=x[b+1],bz=x[b+2],cx=x[c],cy=x[c+1],cz=x[c+2],dx=x[d],dy=x[d+1],dz=x[d+2];
    const ex=dx-cx,ey=dy-cy,ez=dz-cz,el=Math.sqrt(ex*ex+ey*ey+ez*ez);
    const ux=cx-ax,uy=cy-ay,uz=cz-az,vx=dx-ax,vy=dy-ay,vz=dz-az;
    const rx=dx-bx,ry=dy-by,rz=dz-bz,sx=cx-bx,sy=cy-by,sz=cz-bz;
    let n1x=uy*vz-uz*vy,n1y=uz*vx-ux*vz,n1z=ux*vy-uy*vx;
    let n2x=ry*sz-rz*sy,n2y=rz*sx-rx*sz,n2z=rx*sy-ry*sx;
    const sq1=n1x*n1x+n1y*n1y+n1z*n1z,sq2=n2x*n2x+n2y*n2y+n2z*n2z;
    if(el<1e-10 || sq1<1e-18 || sq2<1e-18)continue;
    const sin=((n1y*n2z-n1z*n2y)*ex+(n1z*n2x-n1x*n2z)*ey+(n1x*n2y-n1y*n2x)*ez)/el;
    const cos=n1x*n2x+n1y*n2y+n1z*n2z;
    // Angle error from the rotated (sin, cos) pair: already wrapped, and for
    // the usual small error a short odd series replaces atan2 (error < 1e-12).
    const sinTarget=targetSin[j],cosTarget=targetCos[j],y=sin*cosTarget-cos*sinTarget,z=cos*cosTarget+sin*sinTarget;
    let C:number;
    if(z>0&&Math.abs(y)<.1*z){const t=y/z,q=t*t;C=t*(1-q*(1/3-q*(1/5-q*(1/7-q/9))));}
    else C=Math.atan2(y,z);
    n1x/=sq1;n1y/=sq1;n1z/=sq1;n2x/=sq2;n2y/=sq2;n2z/=sq2;
    const g0x=-el*n1x,g0y=-el*n1y,g0z=-el*n1z,g1x=-el*n2x,g1y=-el*n2y,g1z=-el*n2z;
    const k2=(vx*ex+vy*ey+vz*ez)/el,l2=(rx*ex+ry*ey+rz*ez)/el;
    const k3=-(ux*ex+uy*ey+uz*ez)/el,l3=-(sx*ex+sy*ey+sz*ez)/el;
    const g2x=k2*n1x+l2*n2x,g2y=k2*n1y+l2*n2y,g2z=k2*n1z+l2*n2z;
    const g3x=k3*n1x+l3*n2x,g3y=k3*n1y+l3*n2y,g3z=k3*n1z+l3*n2z;
    const stiffness=alpha[j];
    const denom=stiffness+w[i0]*(g0x*g0x+g0y*g0y+g0z*g0z)+w[i1]*(g1x*g1x+g1y*g1y+g1z*g1z)+w[i2]*(g2x*g2x+g2y*g2y+g2z*g2z)+w[i3]*(g3x*g3x+g3y*g3y+g3z*g3z);
    const dl=(-C-stiffness*lambda[j])/denom;lambda[j]+=dl;
    const w0=w[i0]*dl,w1=w[i1]*dl,w2=w[i2]*dl,w3=w[i3]*dl;
    x[a]=ax+w0*g0x;x[a+1]=ay+w0*g0y;x[a+2]=az+w0*g0z;
    x[b]=bx+w1*g1x;x[b+1]=by+w1*g1y;x[b+2]=bz+w1*g1z;
    x[c]=cx+w2*g2x;x[c+1]=cy+w2*g2y;x[c+2]=cz+w2*g2z;
    x[d]=dx+w3*g3x;x[d+1]=dy+w3*g3y;x[d+2]=dz+w3*g3z;
  }
}
/** Pairwise axial damping dissipates internal strain rates, preserving linear/angular momentum. */
export function dampEdges(x:Float64Array,v:Float64Array,w:Weights,edges:EdgeBatch,amount:number) {
  const ids=edges.ids;
  for(let j=0;j<edges.rest.length;j++){
    const ia=ids[2*j],ib=ids[2*j+1],a=3*ia,b=3*ib;
    const nx=x[b]-x[a],ny=x[b+1]-x[a+1],nz=x[b+2]-x[a+2],l=Math.sqrt(nx*nx+ny*ny+nz*nz);if(l<1e-12)continue;
    const dx=nx/l,dy=ny/l,dz=nz/l,rate=(v[b]-v[a])*dx+(v[b+1]-v[a+1])*dy+(v[b+2]-v[a+2])*dz;
    const impulse=amount*rate/(w[ia]+w[ib]),sa=w[ia]*impulse,sb=w[ib]*impulse;
    v[a]+=sa*dx;v[a+1]+=sa*dy;v[a+2]+=sa*dz;v[b]-=sb*dx;v[b+1]-=sb*dy;v[b+2]-=sb*dz;
  }
}

/** Effective laminate attachment. Conserves weighted position; not frictional contact. */
export function solveLayerTie(x:Float64Array,w:Weights,tie:import('../folding/flight-mesh').LayerTie){
 const {vertex,face,weights}=tie,den=w[vertex]+face.reduce((sum,i,j)=>sum+w[i]*weights[j]**2,0);
 for(let d=0;d<3;d++){const error=x[3*vertex+d]-face.reduce((sum,i,j)=>sum+x[3*i+d]*weights[j],0),delta=-error/den;x[3*vertex+d]+=w[vertex]*delta;face.forEach((i,j)=>x[3*i+d]-=w[i]*weights[j]*delta);}
}

/** Near-inextensible safety envelope for a game without tearing. This inequality
 * is inactive in the elastic range; it prevents under-resolved stiff modes from
 * turning a thin sheet into a rubber net. It is not an extra elastic modulus. */
export function limitEdgeStrains(x:Float64Array,w:Weights,edges:EdgeBatch,forward:boolean,limit=.02){
 const ids=edges.ids,rest=edges.rest,count=rest.length;
 for(let k=0;k<count;k++){
  const j=forward?k:count-1-k,ia=ids[2*j],ib=ids[2*j+1],a=3*ia,b=3*ib;
  const dx=x[a]-x[b],dy=x[a+1]-x[b+1],dz=x[a+2]-x[b+2],l=Math.sqrt(dx*dx+dy*dy+dz*dz);if(l<1e-12)continue;
  const target=Math.max(rest[j]*(1-limit),Math.min(rest[j]*(1+limit),l));
  // An inactive inequality has exactly zero impulse; avoid six position writes
  // in the overwhelmingly common elastic range without weakening the envelope.
  if(target===l)continue;
  const impulse=(target-l)/(l*(w[ia]+w[ib])),wa=w[ia]*impulse,wb=w[ib]*impulse;
  x[a]+=wa*dx;x[a+1]+=wa*dy;x[a+2]+=wa*dz;x[b]-=wb*dx;x[b+1]-=wb*dy;x[b+2]-=wb*dz;
 }
}
/** True when every edge is within `limit` of its rest length. */
export function edgeStrainsWithin(x:Float64Array,edges:EdgeBatch,limit:number){
 const ids=edges.ids,rest=edges.rest;
 for(let j=0;j<rest.length;j++){
  const a=3*ids[2*j],b=3*ids[2*j+1],dx=x[a]-x[b],dy=x[a+1]-x[b+1],dz=x[a+2]-x[b+2];
  if(!(Math.abs(Math.sqrt(dx*dx+dy*dy+dz*dz)/rest[j]-1)<limit))return false;
 }
 return true;
}
