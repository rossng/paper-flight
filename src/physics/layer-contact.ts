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
// Contact activates within NEAR of the face. Between full passes a candidate
// stays listed while within REACH (barycentric) and REACH_GAP (metres) of it.
const NEAR=.0005,REACH=.25,REACH_GAP=.01;
// An unloaded contact farther than this from closing may skip sweeps.
const COLD_GAP=1e-4,COLD_SWEEPS=8;
/** Flat mirror of the candidate list and of one substep's active contacts.
 * Candidate objects remain the public record; `sync` writes side state back. */
export class LayerContactSolver {
  private vertex:Int32Array; private faceIndex:Int32Array; private faces:Int32Array;
  private orientation:Float64Array; private ambiguous:Uint8Array; private sideIndex:Int32Array;
  private sign:Float64Array; private chosen:Uint8Array; private side:Float64Array;
  private sideStates:{side?:number}[]=[];
  // Per face: a, ab, ac, unit normal, previous unit normal, previous a, Gram terms.
  private geometry:Float64Array; private valid:Uint8Array;
  private near:Int32Array; private nearCount=0;
  count=0;
  private contactCandidate=new Int32Array(64); private ids=new Int32Array(256);
  // Per contact: weights, normal, old relative position, tangent impulse, normal impulse.
  private data=new Float64Array(64*13);
  // Unloaded contacts last seen clear of their face; rechecked every COLD_SWEEPS sweeps.
  private cold=new Uint8Array(64);
  constructor(readonly candidates:ContactCandidate[]){
    const count=candidates.length,faceIds=new Map<Face,number>(),sideIds=new Map<object,number>(),faces:number[]=[];
    this.vertex=new Int32Array(count);this.faceIndex=new Int32Array(count);this.orientation=new Float64Array(count);
    this.ambiguous=new Uint8Array(count);this.sideIndex=new Int32Array(count);this.sign=new Float64Array(count);this.chosen=new Uint8Array(count);this.near=new Int32Array(count);
    candidates.forEach((c,i)=>{
      let face=faceIds.get(c.face);if(face===undefined){face=faceIds.size;faceIds.set(c.face,face);faces.push(...c.face);}
      let side=-1;
      if(c.sideState){side=sideIds.get(c.sideState)??-1;if(side<0){side=this.sideStates.length;sideIds.set(c.sideState,side);this.sideStates.push(c.sideState);}}
      this.vertex[i]=c.vertex;this.faceIndex[i]=face;this.orientation[i]=c.orientation??NaN;this.ambiguous[i]=c.ambiguous?1:0;this.sideIndex[i]=side;
    });
    this.faces=Int32Array.from(faces);this.side=new Float64Array(this.sideStates.length);
    this.geometry=new Float64Array(faceIds.size*22);this.valid=new Uint8Array(faceIds.size);
    this.load();
  }
  /** Read side state from the candidate objects. */
  load(){
    this.candidates.forEach((c,i)=>{this.sign[i]=c.sign;this.chosen[i]=c.sideChosen?1:0;});
    this.sideStates.forEach((state,i)=>{this.side[i]=state.side||0;});
  }
  /** Write side state back to the candidate objects. */
  sync(){
    this.candidates.forEach((c,i)=>{c.sign=this.sign[i];if(this.chosen[i]||c.sideChosen!==undefined)c.sideChosen=!!this.chosen[i];});
    this.sideStates.forEach((state,i)=>{if(this.side[i]||state.side!==undefined)state.side=this.side[i]||undefined;});
  }
  /** Forget chosen sides at launch; the last seeded orientation stays. */
  reset(){this.chosen.fill(0);this.side.fill(0);this.count=0;}
  /** Refresh barycentric contact points each substep so a vertex can slide across
   * triangle boundaries. Candidate groups persist through separation and recontact. */
  detect(x:Float64Array,old:Float64Array,full=true){
    const faces=this.faces,g=this.geometry,valid=this.valid;
    for(let f=0;f<valid.length;f++){
      const a=3*faces[3*f],b=3*faces[3*f+1],c=3*faces[3*f+2],o=22*f;valid[f]=0;
      const ax=x[a],ay=x[a+1],az=x[a+2],abx=x[b]-ax,aby=x[b+1]-ay,abz=x[b+2]-az,acx=x[c]-ax,acy=x[c+1]-ay,acz=x[c+2]-az;
      const avx=aby*acz-abz*acy,avy=abz*acx-abx*acz,avz=abx*acy-aby*acx,len=Math.hypot(avx,avy,avz);
      if(len<1e-12)continue;
      const aa=abx*abx+aby*aby+abz*abz,bb=acx*acx+acy*acy+acz*acz,cc=abx*acx+aby*acy+abz*acz,den=aa*bb-cc*cc;
      if(den<1e-20)continue;
      const ux=old[b]-old[a],uy=old[b+1]-old[a+1],uz=old[b+2]-old[a+2],vx=old[c]-old[a],vy=old[c+1]-old[a+1],vz=old[c+2]-old[a+2];
      const ox=uy*vz-uz*vy,oy=uz*vx-ux*vz,oz=ux*vy-uy*vx,inverse=1/len,oldInverse=1/Math.max(Math.hypot(ox,oy,oz),1e-12);
      g[o]=ax;g[o+1]=ay;g[o+2]=az;g[o+3]=abx;g[o+4]=aby;g[o+5]=abz;g[o+6]=acx;g[o+7]=acy;g[o+8]=acz;
      g[o+9]=avx*inverse;g[o+10]=avy*inverse;g[o+11]=avz*inverse;g[o+12]=ox*oldInverse;g[o+13]=oy*oldInverse;g[o+14]=oz*oldInverse;
      g[o+15]=old[a];g[o+16]=old[a+1];g[o+17]=old[a+2];g[o+18]=aa;g[o+19]=bb;g[o+20]=cc;g[o+21]=den;valid[f]=1;
    }
    const vertex=this.vertex,faceIndex=this.faceIndex,sign=this.sign,chosen=this.chosen,side=this.side,sideIndex=this.sideIndex,orientation=this.orientation,ambiguous=this.ambiguous;
    this.count=0;
    // A full pass visits every candidate and records those within reach of
    // their face. Later substeps of the same frame revisit only that list.
    const near=this.near,total=full?vertex.length:this.nearCount;
    if(full)this.nearCount=0;
    // Candidates are grouped by face: keep its geometry in locals while it repeats.
    let f=-1,o=0,ax=0,ay=0,az=0,nx=0,ny=0,nz=0,onx=0,ony=0,onz=0,oax=0,oay=0,oaz=0,abx=0,aby=0,abz=0,acx=0,acy=0,acz=0,aa=0,bb=0,cc=0,den=1;
    for(let q=0;q<total;q++){
      const i=full?q:near[q],state=sideIndex[i];
      if(state>=0&&side[state]){sign[i]=side[state]*orientation[i];chosen[i]=1;}
      if(faceIndex[i]!==f){
        f=faceIndex[i];o=22*f;
        ax=g[o];ay=g[o+1];az=g[o+2];abx=g[o+3];aby=g[o+4];abz=g[o+5];acx=g[o+6];acy=g[o+7];acz=g[o+8];
        nx=g[o+9];ny=g[o+10];nz=g[o+11];onx=g[o+12];ony=g[o+13];onz=g[o+14];oax=g[o+15];oay=g[o+16];oaz=g[o+17];
        aa=g[o+18];bb=g[o+19];cc=g[o+20];den=g[o+21];
      }
      if(!valid[f])continue;
      const k=3*vertex[i],px=x[k]-ax,py=x[k+1]-ay,pz=x[k+2]-az;
      const rawGap=px*nx+py*ny+pz*nz;
      const oldGap=(old[k]-oax)*onx+(old[k+1]-oay)*ony+(old[k+2]-oaz)*onz;
      // Follow the actual previous side, including large rotations and recontact.
      // A rest ordering must never become a long-range attractive constraint.
      if(ambiguous[i]&&Math.abs(oldGap)>1e-8){
        sign[i]=Math.sign(oldGap);chosen[i]=1;
        if(state>=0)side[state]=sign[i]*orientation[i];
      }
      if(chosen[i]&&rawGap*sign[i]>NEAR&&!full)continue;
      const dab=px*abx+py*aby+pz*abz,dac=px*acx+py*acy+pz*acz;
      const u=(dab*bb-dac*cc)/den,v=(dac*aa-dab*cc)/den;
      if(full){
        if(u>=-REACH&&v>=-REACH&&u+v<=1+REACH&&!(chosen[i]&&rawGap*sign[i]>NEAR+REACH_GAP))near[this.nearCount++]=i;
        if(chosen[i]&&rawGap*sign[i]>NEAR)continue;
      }
      if(u<-1e-8||v<-1e-8||u+v>1+1e-8)continue;
      if(ambiguous[i]&&!chosen[i]){
        if(Math.abs(rawGap)<1e-8)continue;
        // Zero-thickness recipes have no ordered sides. The first opening selects
        // a side locally, rather than imposing incompatible ordering at shared seams.
        sign[i]=Math.sign(rawGap);chosen[i]=1;
        if(state>=0)side[state]=sign[i]*orientation[i];
      }
      const s=sign[i];
      if(rawGap*s>NEAR)continue;
      if(this.count===this.contactCandidate.length)this.grow();
      const n=this.count++,ids=this.ids,data=this.data,d=13*n,w0=1-u-v;
      const a=3*faces[3*f],b=3*faces[3*f+1],c=3*faces[3*f+2];
      this.contactCandidate[n]=i;ids[4*n]=vertex[i];ids[4*n+1]=faces[3*f];ids[4*n+2]=faces[3*f+1];ids[4*n+3]=faces[3*f+2];
      data[d]=w0;data[d+1]=u;data[d+2]=v;data[d+3]=nx*s;data[d+4]=ny*s;data[d+5]=nz*s;
      data[d+6]=old[k]-w0*old[a]-u*old[b]-v*old[c];data[d+7]=old[k+1]-w0*old[a+1]-u*old[b+1]-v*old[c+1];data[d+8]=old[k+2]-w0*old[a+2]-u*old[b+2]-v*old[c+2];
      data[d+9]=0;data[d+10]=0;data[d+11]=0;data[d+12]=0;this.cold[n]=0;
    }
  }
  private grow(){
    const size=this.contactCandidate.length*2,candidate=new Int32Array(size),ids=new Int32Array(size*4),data=new Float64Array(size*13),cold=new Uint8Array(size);
    candidate.set(this.contactCandidate);ids.set(this.ids);data.set(this.data);cold.set(this.cold);this.contactCandidate=candidate;this.ids=ids;this.data=data;this.cold=cold;
  }
  /** Unilateral normal projection plus a Coulomb disk |lambda_t| <= mu lambda_n.
   * Internal equal/opposite barycentric corrections preserve the mass centre. */
  solve(x:Float64Array,w:ArrayLike<number>,friction:number,sweep=0){
    const ids=this.ids,data=this.data,cold=this.cold,recheck=sweep%COLD_SWEEPS===0;
    for(let n=0;n<this.count;n++){
      if(cold[n]&&!recheck)continue;
      const d=13*n,iv=ids[4*n],i0=ids[4*n+1],i1=ids[4*n+2],i2=ids[4*n+3],pv=3*iv,a=3*i0,b=3*i1,c=3*i2;
      const k0=data[d],k1=data[d+1],k2=data[d+2],nx=data[d+3],ny=data[d+4],nz=data[d+5];
      const rx=x[pv]-k0*x[a]-k1*x[b]-k2*x[c];
      const ry=x[pv+1]-k0*x[a+1]-k1*x[b+1]-k2*x[c+1];
      const rz=x[pv+2]-k0*x[a+2]-k1*x[b+2]-k2*x[c+2];
      const gap=rx*nx+ry*ny+rz*nz,lambda=data[d+12];
      // A separated contact that carries no impulse projects to exactly nothing:
      // friction is bounded by the (zero) normal multiplier.
      if(lambda===0&&gap>=0){cold[n]=gap>COLD_GAP?1:0;continue;}
      cold[n]=0;
      const wv=w[iv],w0=w[i0]*k0,w1=w[i1]*k1,w2=w[i2]*k2;
      const den=wv+w[i0]*k0**2+w[i1]*k1**2+w[i2]*k2**2;if(den<=0)continue;
      const next=Math.max(0,lambda-gap/den),dl=next-lambda;data[d+12]=next;
      const sx=rx-data[d+6],sy=ry-data[d+7],sz=rz-data[d+8];
      const normalSlip=sx*nx+sy*ny+sz*nz;
      const tx=data[d+9]-(sx-normalSlip*nx)/den;
      const ty=data[d+10]-(sy-normalSlip*ny)/den;
      const tz=data[d+11]-(sz-normalSlip*nz)/den;
      const magnitude=Math.hypot(tx,ty,tz),limit=friction*next,factor=magnitude>limit?limit/Math.max(magnitude,1e-20):1;
      const fx=tx*factor,fy=ty*factor,fz=tz*factor;
      const ix=dl*nx+fx-data[d+9],iy=dl*ny+fy-data[d+10],iz=dl*nz+fz-data[d+11];
      data[d+9]=fx;data[d+10]=fy;data[d+11]=fz;
      x[pv]+=wv*ix;x[pv+1]+=wv*iy;x[pv+2]+=wv*iz;
      x[a]-=w0*ix;x[a+1]-=w0*iy;x[a+2]-=w0*iz;
      x[b]-=w1*ix;x[b+1]-=w1*iy;x[b+2]-=w1*iz;
      x[c]-=w2*ix;x[c+1]-=w2*iy;x[c+2]-=w2*iz;
    }
  }
  /** Contact n as a detached record (tests, diagnostics). */
  contact(n:number):LayerContact{
    const d=13*n,data=this.data;
    return {...this.candidates[this.contactCandidate[n]],weights:[data[d],data[d+1],data[d+2]],normal:[data[d+3],data[d+4],data[d+5]],
      oldRelative:[data[d+6],data[d+7],data[d+8]],tangent:[data[d+9],data[d+10],data[d+11]],lambda:data[d+12]};
  }
  /** Replace the active set with one explicit contact on vertices 0..3. */
  adopt(contact:LayerContact){
    const data=this.data;this.count=1;this.cold[0]=0;this.ids.set([0,1,2,3]);
    data.set(contact.weights,0);data.set(contact.normal,3);data.set(contact.oldRelative,6);data.set(contact.tangent,9);data[12]=contact.lambda;
  }
}
const flatten=(vectors:V3[])=>{const out=new Float64Array(vectors.length*3);for(let i=0;i<vectors.length;i++){out[3*i]=vectors[i][0];out[3*i+1]=vectors[i][1];out[3*i+2]=vectors[i][2];}return out;};
/** Per-vertex array form of LayerContactSolver.detect; updates the candidates' side state. */
export function detectLayerContacts(p:V3[],old:V3[],candidates:ContactCandidate[]):LayerContact[]{
  const solver=new LayerContactSolver(candidates);
  solver.detect(flatten(p),flatten(old));solver.sync();
  return Array.from({length:solver.count},(_,n)=>solver.contact(n));
}
const scratch=new LayerContactSolver([]),scratchX=new Float64Array(12),scratchW=new Float64Array(4);
/** Per-vertex array form of LayerContactSolver.solve for one contact. */
export function solveLayerContact(p:V3[],w:number[],contact:LayerContact,friction:number){
  const ids=[contact.vertex,...contact.face];
  ids.forEach((id,j)=>{scratchX[3*j]=p[id][0];scratchX[3*j+1]=p[id][1];scratchX[3*j+2]=p[id][2];scratchW[j]=w[id];});
  scratch.adopt(contact);scratch.solve(scratchX,scratchW,friction);
  const updated=scratch.contact(0);contact.lambda=updated.lambda;contact.tangent=updated.tangent;
  ids.forEach((id,j)=>{p[id][0]=scratchX[3*j];p[id][1]=scratchX[3*j+1];p[id][2]=scratchX[3*j+2];});
}
