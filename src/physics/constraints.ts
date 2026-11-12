import { cross, dot, length, scale, sub, wrapAngle, type V3 } from './math';
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
/** Scalar hot path: same signed gradients as hingeGradients, without temporary vectors. */
export function solveHinge(p: V3[],w: number[],h: Hinge,compliance: number,dt: number) {
  const [i0,i1,i2,i3]=h.ids,a=p[i0],b=p[i1],c=p[i2],d=p[i3];
  const ex=d[0]-c[0],ey=d[1]-c[1],ez=d[2]-c[2],el=Math.hypot(ex,ey,ez);
  const ux=c[0]-a[0],uy=c[1]-a[1],uz=c[2]-a[2],vx=d[0]-a[0],vy=d[1]-a[1],vz=d[2]-a[2];
  const rx=d[0]-b[0],ry=d[1]-b[1],rz=d[2]-b[2],sx=c[0]-b[0],sy=c[1]-b[1],sz=c[2]-b[2];
  let n1x=uy*vz-uz*vy,n1y=uz*vx-ux*vz,n1z=ux*vy-uy*vx;
  let n2x=ry*sz-rz*sy,n2y=rz*sx-rx*sz,n2z=rx*sy-ry*sx;
  const sq1=n1x*n1x+n1y*n1y+n1z*n1z,sq2=n2x*n2x+n2y*n2y+n2z*n2z;
  if(el<1e-10 || sq1<1e-18 || sq2<1e-18)return;
  const sin=((n1y*n2z-n1z*n2y)*ex+(n1z*n2x-n1x*n2z)*ey+(n1x*n2y-n1y*n2x)*ez)/el;
  const cos=n1x*n2x+n1y*n2y+n1z*n2z;
  const C=wrapAngle(Math.atan2(sin,cos)-h.rest-(h.plasticOffset??0));
  n1x/=sq1;n1y/=sq1;n1z/=sq1;n2x/=sq2;n2y/=sq2;n2z/=sq2;
  const g0x=-el*n1x,g0y=-el*n1y,g0z=-el*n1z,g1x=-el*n2x,g1y=-el*n2y,g1z=-el*n2z;
  const k2=(vx*ex+vy*ey+vz*ez)/el,l2=(rx*ex+ry*ey+rz*ez)/el;
  const k3=-(ux*ex+uy*ey+uz*ez)/el,l3=-(sx*ex+sy*ey+sz*ez)/el;
  const g2x=k2*n1x+l2*n2x,g2y=k2*n1y+l2*n2y,g2z=k2*n1z+l2*n2z;
  const g3x=k3*n1x+l3*n2x,g3y=k3*n1y+l3*n2y,g3z=k3*n1z+l3*n2z;
  const alpha=compliance/(dt*dt);
  const denom=alpha+w[i0]*(g0x*g0x+g0y*g0y+g0z*g0z)+w[i1]*(g1x*g1x+g1y*g1y+g1z*g1z)+w[i2]*(g2x*g2x+g2y*g2y+g2z*g2z)+w[i3]*(g3x*g3x+g3y*g3y+g3z*g3z);
  const dl=(-C-alpha*h.lambda)/denom;h.lambda+=dl;
  const w0=w[i0]*dl,w1=w[i1]*dl,w2=w[i2]*dl,w3=w[i3]*dl;
  a[0]+=w0*g0x;a[1]+=w0*g0y;a[2]+=w0*g0z;
  b[0]+=w1*g1x;b[1]+=w1*g1y;b[2]+=w1*g1z;
  c[0]+=w2*g2x;c[1]+=w2*g2y;c[2]+=w2*g2z;
  d[0]+=w3*g3x;d[1]+=w3*g3y;d[2]+=w3*g3z;
}
/** Pairwise axial damping dissipates internal strain rates, preserving linear/angular momentum. */
export function dampEdges(p: V3[],v: V3[],w:number[],edges:Edge[],amount:number) {
  for(const e of edges) {
    const n=sub(p[e.b],p[e.a]),l=length(n);if(l<1e-12)continue;
    const direction=scale(n,1/l),rate=dot(sub(v[e.b],v[e.a]),direction);
    const impulse=amount*rate/(w[e.a]+w[e.b]);
    for(let d=0;d<3;d++){v[e.a][d]+=w[e.a]*impulse*direction[d];v[e.b][d]-=w[e.b]*impulse*direction[d];}
  }
}

/** Effective laminate attachment. Conserves weighted position; not frictional contact. */
export function solveLayerTie(p:V3[],w:number[],tie:import('../folding/flight-mesh').LayerTie){
 const {vertex,face,weights}=tie,den=w[vertex]+face.reduce((sum,i,j)=>sum+w[i]*weights[j]**2,0);
 for(let d=0;d<3;d++){const error=p[vertex][d]-face.reduce((sum,i,j)=>sum+p[i][d]*weights[j],0),delta=-error/den;p[vertex][d]+=w[vertex]*delta;face.forEach((i,j)=>p[i][d]-=w[i]*weights[j]*delta);}
}

/** Near-inextensible safety envelope for a game without tearing. This inequality
 * is inactive in the elastic range; it prevents under-resolved stiff modes from
 * turning a thin sheet into a rubber net. It is not an extra elastic modulus. */
export function limitEdgeStrain(p:V3[],w:number[],e:Edge,limit=.02){
 const a=p[e.a],b=p[e.b],dx=a[0]-b[0],dy=a[1]-b[1],dz=a[2]-b[2],l=Math.hypot(dx,dy,dz);if(l<1e-12)return;
 const target=Math.max(e.rest*(1-limit),Math.min(e.rest*(1+limit),l));
 // An inactive inequality has exactly zero impulse; avoid six position writes
 // in the overwhelmingly common elastic range without weakening the envelope.
 if(target===l)return;
 const impulse=(target-l)/(l*(w[e.a]+w[e.b]));
 const wa=w[e.a]*impulse,wb=w[e.b]*impulse;
 a[0]+=wa*dx;a[1]+=wa*dy;a[2]+=wa*dz;b[0]-=wb*dx;b[1]-=wb*dy;b[2]-=wb*dz;
}
