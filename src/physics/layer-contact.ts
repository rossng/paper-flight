import type { Face, PaperMesh } from '../geometry/dart';
import { sub, dot, cross, length, scale, type V3 } from './math';
import { kernel, F64, I32 } from './solver';

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
/** Flat form of the candidate list for the solver. Candidate objects remain the public
 * record; `sync` writes side state back to them. */
export class LayerContactSolver {
  readonly vertex:Int32Array; readonly faceIndex:Int32Array; readonly faces:Int32Array;
  readonly orientation:Float64Array; readonly ambiguous:Uint8Array; readonly sideIndex:Int32Array;
  readonly sign:Float64Array; readonly chosen:Uint8Array; readonly side:Float64Array;
  readonly sideStates:{side?:number}[]=[];
  constructor(readonly candidates:ContactCandidate[]){
    const count=candidates.length,faceIds=new Map<Face,number>(),sideIds=new Map<object,number>(),faces:number[]=[];
    this.vertex=new Int32Array(count);this.faceIndex=new Int32Array(count);this.orientation=new Float64Array(count);
    this.ambiguous=new Uint8Array(count);this.sideIndex=new Int32Array(count);this.sign=new Float64Array(count);this.chosen=new Uint8Array(count);
    candidates.forEach((c,i)=>{
      let face=faceIds.get(c.face);if(face===undefined){face=faceIds.size;faceIds.set(c.face,face);faces.push(...c.face);}
      let side=-1;
      if(c.sideState){side=sideIds.get(c.sideState)??-1;if(side<0){side=this.sideStates.length;sideIds.set(c.sideState,side);this.sideStates.push(c.sideState);}}
      this.vertex[i]=c.vertex;this.faceIndex[i]=face;this.orientation[i]=c.orientation??NaN;this.ambiguous[i]=c.ambiguous?1:0;this.sideIndex[i]=side;
    });
    this.faces=Int32Array.from(faces);this.side=new Float64Array(this.sideStates.length);
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
  reset(){this.chosen.fill(0);this.side.fill(0);}
  /** Hand the candidates and their side state to the solver's staging slots. */
  stage(){
    kernel.i32(I32.layerVertex,this.vertex);kernel.i32(I32.layerFaceIndex,this.faceIndex);kernel.i32(I32.layerFaces,this.faces);kernel.i32(I32.layerAmbiguous,this.ambiguous);
    kernel.i32(I32.layerSideIndex,this.sideIndex);kernel.i32(I32.layerChosen,this.chosen);
    kernel.f64(F64.layerOrientation,this.orientation);kernel.f64(F64.layerSign,this.sign);kernel.f64(F64.layerSide,this.side);
  }
}
/** Contacts of the candidates at `p`, given the positions `old` before the step: a vertex
 * within 0.5 mm of its face on its own side, at its current barycentric point, so it can
 * slide across the face. Updates the candidates' side state. */
export function detectLayerContacts(p:V3[],old:V3[],candidates:ContactCandidate[]):LayerContact[]{
  const solver=new LayerContactSolver(candidates);solver.stage();
  const layers=kernel.call.layers_create();
  kernel.vectors(F64.x,p);kernel.vectors(F64.old,old);
  kernel.call.layers_detect(layers,1);kernel.call.layers_report(layers);kernel.call.layers_free(layers);
  const active=kernel.read32(I32.ids),data=kernel.read64(F64.a);
  solver.sign.set(kernel.read64(F64.b));solver.side.set(kernel.read64(F64.c));solver.chosen.set(kernel.read32(I32.flags));solver.sync();
  return Array.from(active,(candidate,n)=>{const d=13*n;
    return {...candidates[candidate],weights:[data[d],data[d+1],data[d+2]] as V3,normal:[data[d+3],data[d+4],data[d+5]] as V3,
      oldRelative:[data[d+6],data[d+7],data[d+8]] as V3,tangent:[data[d+9],data[d+10],data[d+11]] as V3,lambda:data[d+12]};});
}
/** One projection of one contact, with Coulomb friction; accumulates its impulses. */
export function solveLayerContact(p:V3[],w:number[],contact:LayerContact,friction:number){
  const ids=[contact.vertex,...contact.face];
  new LayerContactSolver([]).stage();
  const layers=kernel.call.layers_create();
  kernel.f64(F64.a,[...contact.weights,...contact.normal,...contact.oldRelative,...contact.tangent,contact.lambda]);kernel.call.layers_adopt(layers);
  kernel.vectors(F64.x,ids.map(id=>p[id]));kernel.f64(F64.w,ids.map(id=>w[id]));
  kernel.call.layers_solve(layers,friction,0);kernel.call.layers_report(layers);kernel.call.layers_free(layers);
  const data=kernel.read64(F64.a),x=kernel.read64(F64.x);
  contact.tangent=[data[9],data[10],data[11]];contact.lambda=data[12];
  ids.forEach((id,j)=>{p[id][0]=x[3*j];p[id][1]=x[3*j+1];p[id][2]=x[3*j+2];});
}
