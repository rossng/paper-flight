import type { Face } from '../geometry/dart';
import { stageHinges, type HingeBatch } from './constraints';
import { stageMembranes, type MembraneBatch } from './membrane';
import { kernel, F64, I32 } from './solver';

/** Direct solve of a substep's elastic response.
 *
 * Gauss–Seidel sweeps over stiff membrane and hinge constraints settle on a softer sheet
 * than the model describes, and more sweeps do not fix it (docs/reference-solver.md). This
 * takes Newton steps on the same implicit-Euler problem the sweeps approximate,
 *   minimise  |x − x̃|²_M / 2dt²  +  E_membrane(x)  +  E_hinge(x),
 * with a sparse LDLᵀ factorisation whose ordering and fill are fixed by the mesh. It then
 * hands the sweeps positions and multipliers that already satisfy every elastic constraint,
 * so they only have to reconcile contact.
 *
 * This class computes the vertex ordering and the matrix's block pattern; the factor's
 * pattern and the numerical solve are the flight solver's (wasm/solver/src/elastic.rs). */
export class ElasticSolver {
  private n:number; private handle=0;
  /** Fill-reducing vertex ordering, and the permuted matrix's upper triangle by vertex
   * column (`ap`, `ai`), a 3×3 block per entry at offset `source` in the block array. */
  readonly perm:Int32Array; readonly source:Int32Array; readonly ap:Int32Array; readonly ai:Int32Array;
  /** Length of the block array. */
  readonly blocks:number;
  /** Block offset of each vertex's diagonal and of each element's vertex pairs. */
  readonly diagonal:Int32Array; readonly faceSlots:Int32Array; readonly hingeSlots:Int32Array;
  constructor(count:number,faces:Face[],hinges:{ids:number[]}[]){
    this.n=count;
    const pairs:[number,number][]=[];
    for(const f of faces)for(let a=0;a<3;a++)for(let b=a+1;b<3;b++)pairs.push([f[a],f[b]]);
    for(const h of hinges)for(let a=0;a<4;a++)for(let b=a+1;b<4;b++)pairs.push([h.ids[a],h.ids[b]]);
    const perm=this.perm=minimumDegree(count,pairs);
    const columns:number[][]=Array.from({length:count},(_,j)=>[j]);
    for(const [i,j] of pairs){const a=Math.min(perm[i],perm[j]),b=Math.max(perm[i],perm[j]);if(a!==b)columns[b].push(a);}
    const slots=new Map<number,number>();let blockCount=0;
    columns.forEach((column,j)=>{
      const rows=[...new Set(column)].sort((a,b)=>a-b);columns[j]=rows;
      for(const i of rows)slots.set(i*count+j,blockCount++);
    });
    // Upper triangle of the permuted matrix by vertex column, a 3×3 block per entry; rows
    // ascend, so a column's last block is its diagonal.
    const ap=this.ap=new Int32Array(count+1),ai:number[]=[],source:number[]=[];
    for(let j=0;j<count;j++){
      for(const i of columns[j]){ai.push(i);source.push(9*slots.get(i*count+j)!);}
      ap[j+1]=ai.length;
    }
    this.ai=Int32Array.from(ai);this.source=Int32Array.from(source);this.blocks=9*blockCount;
    // Where each element's vertex pairs land: block offset, negative (as ~offset) when the
    // pair is stored transposed.
    const slot=(i:number,j:number)=>{const a=perm[i],b=perm[j];return a<=b?9*slots.get(a*count+b)!:~(9*slots.get(b*count+a)!);};
    this.diagonal=Int32Array.from({length:count},(_,i)=>slot(i,i));
    const collect=(ids:number[])=>{const out:number[]=[];for(let a=0;a<ids.length;a++)for(let b=a;b<ids.length;b++)out.push(slot(ids[a],ids[b]));return out;};
    this.faceSlots=Int32Array.from(faces.flatMap(collect));this.hingeSlots=Int32Array.from(hinges.flatMap(h=>collect(h.ids)));
  }
  /** Hand the symbolic set-up to a solver kernel's staging slots. */
  stage(){
    kernel.i32(I32.perm,this.perm);kernel.i32(I32.ap,this.ap);kernel.i32(I32.ai,this.ai);kernel.i32(I32.source,this.source);
    kernel.i32(I32.diagonal,this.diagonal);kernel.i32(I32.faceSlots,this.faceSlots);kernel.i32(I32.hingeSlots,this.hingeSlots);
  }
  /** Replace `x` (the unconstrained prediction) with the elastic implicit-Euler solution
   * and set the sweeps' multipliers to match. `alpha` values are compliance/dt². Newton
   * steps with a backtracking line search until the largest update is below `tolerance`
   * metres. Returns false, leaving the prediction untouched, if `iterations` steps do not
   * get there. Each solve starts from the previous one's correction until `reset`. */
  solve(x:Float64Array,masses:ArrayLike<number>,dt:number,membranes:MembraneBatch,hinges:HingeBatch,iterations:number,tolerance=1e-6){
    if(!this.handle){this.stage();const handle=this.handle=kernel.call.elastic_create(this.n,this.blocks);kernel.own(this,solver=>solver.elastic_free(handle));}
    stageMembranes(membranes);stageHinges(hinges);kernel.f64(F64.x,x);kernel.f64(F64.w,masses);
    const solved=kernel.call.elastic_solve(this.handle,dt,iterations,tolerance)===1;
    x.set(kernel.read64(F64.x));membranes.lambda.set(kernel.read64(F64.b));hinges.lambda.set(kernel.read64(F64.d));
    return solved;
  }
  reset(){if(this.handle)kernel.call.elastic_reset(this.handle);}
}
/** Greedy minimum-degree ordering of the vertex graph; run once per mesh. */
function minimumDegree(n:number,pairs:[number,number][]){
  const adjacent=new Uint8Array(n*n),degree=new Int32Array(n),done=new Uint8Array(n),perm=new Int32Array(n);
  for(const [i,j] of pairs)if(i!==j&&!adjacent[i*n+j]){adjacent[i*n+j]=adjacent[j*n+i]=1;degree[i]++;degree[j]++;}
  const neighbours:number[]=[];
  for(let position=0;position<n;position++){
    let v=-1;for(let i=0;i<n;i++)if(!done[i]&&(v<0||degree[i]<degree[v]))v=i;
    done[v]=1;perm[v]=position;neighbours.length=0;
    for(let j=0;j<n;j++)if(!done[j]&&adjacent[v*n+j])neighbours.push(j);
    for(const a of neighbours){
      degree[a]--;
      for(const b of neighbours)if(a<b&&!adjacent[a*n+b]){adjacent[a*n+b]=adjacent[b*n+a]=1;degree[a]++;degree[b]++;}
    }
  }
  return perm;
}
