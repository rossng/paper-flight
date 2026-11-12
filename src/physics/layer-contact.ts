import type { Face, PaperMesh } from '../geometry/dart';
import { sub, dot, cross, length, scale, type V3 } from './math';

export interface ContactCandidate { vertex:number; face:Face; sign:number; ambiguous?:boolean; sideChosen?:boolean; orientation?:number; sideState?:{side?:number} }
export interface LayerContact extends ContactCandidate { weights:V3; normal:V3; lambda:number; tangent:V3; oldRelative:V3 }
/** Rest-coplanar material groups only. Ordering of exactly coincident panels is
 * selected by the first local opening, not an inferred physical fold-stack order. Zero clearance preserves
 * the recipe's midsurfaces; thickness still controls mass and elastic response. */
export function buildContactCandidates(mesh:PaperMesh):ContactCandidate[]{
  if(!mesh.faceOwners)return [];
  const owners=mesh.rest.map(()=>new Set<number>());
  mesh.faces.forEach((f,fi)=>f.forEach(v=>owners[v].add(mesh.faceOwners![fi])));
  const result:ContactCandidate[]=[],sides=new Map<string,{side?:number}>();
  mesh.faces.forEach((face,fi)=>{
    const [a,b,c]=face.map(i=>mesh.rest[i]),av=cross(sub(b,a),sub(c,a)),len=length(av);if(len<1e-12)return;
    const normal=scale(av,1/len),canonical=normal.reduce((best,v,i)=>Math.abs(v)>Math.abs(normal[best])?i:best,0),orientation=normal[canonical]<0?-1:1;
    mesh.rest.forEach((p,vertex)=>{
      if(owners[vertex].has(mesh.faceOwners![fi])||Math.abs(dot(sub(p,a),normal))>1e-7)return;
      // Seed an orientation; ambiguous coincident sides are selected on first opening.
      const rank=Math.min(...owners[vertex]);
      const key=`${vertex}:${mesh.faceOwners![fi]}`;
      if(!sides.has(key))sides.set(key,{});
      result.push({vertex,face,sign:orientation*(rank>mesh.faceOwners![fi]?1:-1),ambiguous:true,orientation,sideState:sides.get(key)});
    });
  });
  return result;
}
/** Refresh barycentric contact points each substep so a vertex can slide across
 * triangle boundaries. Candidate groups persist through separation and recontact. */
export function detectLayerContacts(p:V3[],old:V3[],candidates:ContactCandidate[]):LayerContact[]{
  const contacts:LayerContact[]=[],geometry=new Map<Face,{a:V3;ab:V3;ac:V3;normal:V3;oldNormal:V3;oldA:V3;aa:number;bb:number;cc:number;den:number}>();
  for(const candidate of candidates){
    const {vertex,face}=candidate;
    if(candidate.sideState?.side){candidate.sign=candidate.sideState.side*candidate.orientation!;candidate.sideChosen=true;}
    let g=geometry.get(face);
    if(!g){
      const [a,b,c]=face.map(i=>p[i]),ab=sub(b,a),ac=sub(c,a),av=cross(ab,ac),len=length(av);
      if(len<1e-12)continue;
      const aa=dot(ab,ab),bb=dot(ac,ac),cc=dot(ab,ac),den=aa*bb-cc*cc;
      if(den<1e-20)continue;
      const [oa,ob,oc]=face.map(i=>old[i]),on=cross(sub(ob,oa),sub(oc,oa));
      g={a,ab,ac,normal:scale(av,1/len),oldNormal:scale(on,1/Math.max(length(on),1e-12)),oldA:oa,aa,bb,cc,den};geometry.set(face,g);
    }
    const px=p[vertex][0]-g.a[0],py=p[vertex][1]-g.a[1],pz=p[vertex][2]-g.a[2];
    const rawGap=px*g.normal[0]+py*g.normal[1]+pz*g.normal[2];
    const oldGap=dot(sub(old[vertex],g.oldA),g.oldNormal);
    // Follow the actual previous side, including large rotations and recontact.
    // A rest ordering must never become a long-range attractive constraint.
    if(candidate.ambiguous&&Math.abs(oldGap)>1e-8){
      candidate.sign=Math.sign(oldGap);candidate.sideChosen=true;
      if(candidate.sideState)candidate.sideState.side=candidate.sign*candidate.orientation!;
    }
    if(candidate.sideChosen&&rawGap*candidate.sign>.0005)continue;
    const dab=px*g.ab[0]+py*g.ab[1]+pz*g.ab[2],dac=px*g.ac[0]+py*g.ac[1]+pz*g.ac[2];
    const u=(dab*g.bb-dac*g.cc)/g.den,v=(dac*g.aa-dab*g.cc)/g.den;
    if(u<-1e-8||v<-1e-8||u+v>1+1e-8)continue;
    if(candidate.ambiguous&&!candidate.sideChosen){
      if(Math.abs(rawGap)<1e-8)continue;
      // Zero-thickness recipes have no ordered sides. The first opening selects
      // a side locally, rather than imposing incompatible ordering at shared seams.
      candidate.sign=Math.sign(rawGap);candidate.sideChosen=true;
      if(candidate.sideState)candidate.sideState.side=candidate.sign*candidate.orientation!;
    }
    const sign=candidate.sign;
    if(rawGap*sign>.0005)continue;
    const weights:V3=[1-u-v,u,v],oldRelative:V3=[...old[vertex]];
    face.forEach((id,j)=>{for(let d=0;d<3;d++)oldRelative[d]-=weights[j]*old[id][d];});
    contacts.push({...candidate,weights,normal:scale(g.normal,sign),lambda:0,tangent:[0,0,0],oldRelative});
  }
  return contacts;
}
/** Unilateral normal projection plus a Coulomb disk |lambda_t| <= mu lambda_n.
 * Internal equal/opposite barycentric corrections preserve the mass centre. */
export function solveLayerContact(p:V3[],w:number[],contact:LayerContact,friction:number){
  const {vertex,face,weights,normal}=contact,pv=p[vertex],a=p[face[0]],b=p[face[1]],c=p[face[2]];
  const rx=pv[0]-weights[0]*a[0]-weights[1]*b[0]-weights[2]*c[0];
  const ry=pv[1]-weights[0]*a[1]-weights[1]*b[1]-weights[2]*c[1];
  const rz=pv[2]-weights[0]*a[2]-weights[1]*b[2]-weights[2]*c[2];
  const den=w[vertex]+w[face[0]]*weights[0]**2+w[face[1]]*weights[1]**2+w[face[2]]*weights[2]**2;if(den<=0)return;
  const gap=rx*normal[0]+ry*normal[1]+rz*normal[2],next=Math.max(0,contact.lambda-gap/den),dl=next-contact.lambda;contact.lambda=next;
  const sx=rx-contact.oldRelative[0],sy=ry-contact.oldRelative[1],sz=rz-contact.oldRelative[2];
  const normalSlip=sx*normal[0]+sy*normal[1]+sz*normal[2];
  const tx=contact.tangent[0]-(sx-normalSlip*normal[0])/den;
  const ty=contact.tangent[1]-(sy-normalSlip*normal[1])/den;
  const tz=contact.tangent[2]-(sz-normalSlip*normal[2])/den;
  const magnitude=Math.hypot(tx,ty,tz),limit=friction*next,factor=magnitude>limit?limit/Math.max(magnitude,1e-20):1;
  for(let d=0;d<3;d++){
    const tangent=(d===0?tx:d===1?ty:tz)*factor,impulse=dl*normal[d]+tangent-contact.tangent[d];contact.tangent[d]=tangent;
    pv[d]+=w[vertex]*impulse;
    a[d]-=w[face[0]]*weights[0]*impulse;b[d]-=w[face[1]]*weights[1]*impulse;c[d]-=w[face[2]]*weights[2]*impulse;
  }
}
