import type { PaperMesh } from '../geometry/dart';
import type { V3 } from './math';
import { kernel, F64, I32 } from './solver';

type Ids = [number,number,number,number];
export interface SelfContact { kind:'vertex-face'|'edge-edge'; ids:Ids; sign:number; key:number }

/** Fixed-topology BVH over features (faces or edges), which the solver refits to swept
 * bounds: no spatial-cell explosion for fast motion and no all-pairs feature list. */
class SweptTree {
  readonly leaves:Int32Array;
  readonly right:Int32Array;
  readonly escape:Int32Array;
  constructor(indices:number[],centers:V3[]){
    // Preorder layout retains the original left-then-right visit order. Escape
    // indices skip entire rejected subtrees without recursive calls or a stack.
    const leaves:number[]=[],right:number[]=[],escape:number[]=[];
    const build=(ids:number[])=>{
      const node=leaves.length;leaves.push(-1);right.push(-1);escape.push(-1);
      if(ids.length===1)leaves[node]=ids[0];
      else{
        const ranges=[0,1,2].map(d=>Math.max(...ids.map(i=>centers[i][d]))-Math.min(...ids.map(i=>centers[i][d])));
        const axis=ranges.indexOf(Math.max(...ranges));ids.sort((a,b)=>centers[a][axis]-centers[b][axis]||a-b);
        const mid=ids.length>>1;build(ids.slice(0,mid));right[node]=leaves.length;build(ids.slice(mid));
      }
      escape[node]=leaves.length;
    };
    build(indices);this.leaves=Int32Array.from(leaves);this.right=Int32Array.from(right);this.escape=Int32Array.from(escape);
  }
}

/** Isolate every root of a cubic in [0,1] using its derivative extrema. Unlike
 * endpoint sign tests this also finds an even number of crossings in a step. */
export function coplanarityRoots(c:number[]):number[]{
  kernel.f64(F64.x,c);kernel.call.cubic_roots();
  return Array.from(kernel.read64(F64.old));
}

/** One contact on per-vertex arrays; the same kernel as the solver sweep: a
 * frictionless, zero-clearance unilateral projection. Recomputing the normal and
 * projected weights gives a distance gradient with zero net force and torque. */
export function solveSelfContact(p:V3[],invMass:number[],contact:SelfContact){
  kernel.vectors(F64.x,contact.ids.map(id=>p[id]));kernel.f64(F64.w,contact.ids.map(id=>invMass[id]));
  kernel.call.contact_project(contact.kind==='vertex-face'?1:0,contact.sign);
  const x=kernel.read64(F64.x);
  contact.ids.forEach((id,j)=>{p[id][0]=x[3*j];p[id][1]=x[3*j+1];p[id][2]=x[3*j+2];});
}

/** Swept self-contact of the sheet with itself: vertex against face and edge against
 * edge, excluding features that share a vertex or a mesh edge. This class holds the
 * topology the solver needs (adjacency, unique edges, BVH layout). A flight keeps its own
 * contact state inside the solver; the methods here run the same kernels on one mesh. */
export class SelfContactSolver {
  /** One-ring adjacency bitset, `ceil(count/32)` 32-bit words per vertex. */
  readonly neighbors:Uint32Array;
  readonly edges:Int32Array; readonly faces:Int32Array;
  readonly faceTree?:SweptTree; readonly edgeTree?:SweptTree;
  private vertices:number; private handle=0;
  /** Active contacts after the last detection. */
  count=0;
  /** This substep's narrow-phase tests and the contacts they added. */
  stats={narrowTests:0,vertexFace:0,edgeEdge:0};
  /** Size of the cached broad-phase pair lists (zero before the first detection). */
  pairCount=0;
  private active:SelfContact[]=[];
  constructor(mesh:PaperMesh){
    // Compact material adjacency: the broad phase tests bits instead of hashing
    // vertex IDs. One bit per pair, with the same one-ring exclusion.
    const count=this.vertices=mesh.rest.length,words=Math.ceil(count/32);
    this.neighbors=new Uint32Array(count*words);
    for(let i=0;i<count;i++)this.neighbors[i*words+(i>>>5)]|=1<<(i&31);
    const keys=new Set<string>(),edges:[number,number][]=[];
    for(const face of mesh.faces)for(let j=0;j<3;j++){
      const a=face[j],b=face[(j+1)%3];this.neighbors[a*words+(b>>>5)]|=1<<(b&31);this.neighbors[b*words+(a>>>5)]|=1<<(a&31);
      const key=`${Math.min(a,b)}:${Math.max(a,b)}`;if(!keys.has(key)){keys.add(key);edges.push([a,b]);}
    }
    const tree=(features:number[][])=>features.length?new SweptTree(features.map((_,i)=>i),features.map(f=>[0,1,2].map(d=>f.reduce((s,i)=>s+mesh.rest[i][d]/f.length,0)) as V3)):undefined;
    this.faceTree=tree(mesh.faces);this.edgeTree=tree(edges);
    this.faces=Int32Array.from(mesh.faces.flat());this.edges=Int32Array.from(edges.flat());
  }
  /** Hand the topology to the solver's staging slots. */
  stage(){
    kernel.i32(I32.faces,this.faces);
    kernel.i32(I32.neighbors,new Int32Array(this.neighbors.buffer,this.neighbors.byteOffset,this.neighbors.length));kernel.i32(I32.selfEdges,this.edges);
    kernel.i32(I32.faceTreeLeaves,this.faceTree?.leaves??[]);kernel.i32(I32.faceTreeRight,this.faceTree?.right??[]);kernel.i32(I32.faceTreeEscape,this.faceTree?.escape??[]);
    kernel.i32(I32.edgeTreeLeaves,this.edgeTree?.leaves??[]);kernel.i32(I32.edgeTreeRight,this.edgeTree?.right??[]);kernel.i32(I32.edgeTreeEscape,this.edgeTree?.escape??[]);
  }
  private get solver(){
    if(!this.handle){this.stage();const handle=this.handle=kernel.call.selfc_create(this.vertices);kernel.own(this,solver=>solver.selfc_free(handle));}
    return this.handle;
  }
  private report(){
    this.count=kernel.call.selfc_report(this.solver);
    const raw=kernel.read32(I32.ids),stats=kernel.read64(F64.a);
    this.active=Array.from({length:this.count},(_,i)=>({ids:Array.from(raw.subarray(7*i,7*i+4)) as Ids,kind:raw[7*i+4]?'vertex-face' as const:'edge-edge' as const,sign:raw[7*i+5],key:raw[7*i+6]}));
    this.stats={narrowTests:stats[0],vertexFace:stats[1],edgeEdge:stats[2]};this.pairCount=stats[3];
  }
  /** Active contacts in detection order. */
  get contacts():SelfContact[]{return this.active;}
  /** Forget every contact and the side each pair was last seen on. */
  reset(){kernel.call.selfc_reset(this.solver);this.report();}
  /** Start a substep: last substep's contacts become the history that tells a pair
   * resting in contact from one that has just crossed. */
  beginStep(){kernel.call.selfc_begin(this.solver);this.report();}
  /** Keep last substep's contacts that still touch inside their features at `old`, or
   * hover within `gap` on their own side, so every sweep of this substep enforces them. */
  carry(old:V3[],gap=0){kernel.vectors(F64.old,old);kernel.call.selfc_carry(this.solver,gap);this.report();}
  /** Swept detection of crossings between the positions before (`old`) and after (`p`) a step. */
  detect(old:V3[],p:V3[]){kernel.vectors(F64.old,old);kernel.vectors(F64.x,p);kernel.call.selfc_detect(this.solver);this.report();}
  /** One projection sweep over the active contacts. */
  solve(p:V3[],invMass:number[],forward=true){
    kernel.vectors(F64.x,p);kernel.f64(F64.w,invMass);kernel.call.selfc_solve(this.solver,forward?1:0,0);kernel.readVectors(F64.x,p);
  }
}
