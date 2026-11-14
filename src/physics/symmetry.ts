import type { PaperMesh } from '../geometry/dart';

/** The z → −z vertex correspondence of a left/right symmetric sheet, or
 * undefined. Coincident folded layers are told apart by mirrored material
 * coordinates, so each vertex maps to the same paper point on the other side. */
export function mirrorMap(mesh:PaperMesh):Int32Array|undefined {
  const rest=mesh.rest,n=rest.length,uv=mesh.materialUV;
  let extent=0;for(const p of rest)for(const x of p)extent=Math.max(extent,Math.abs(x));
  const tol=1e-9*Math.max(extent,1e-3);
  const mirror=new Int32Array(n).fill(-1);
  for(let i=0;i<n;i++){
    const [x,y,z]=rest[i];let found=-1,count=0;
    for(let j=0;j<n;j++){
      const q=rest[j];
      if(Math.abs(q[0]-x)>tol||Math.abs(q[1]-y)>tol||Math.abs(q[2]+z)>tol)continue;
      if(uv&&(Math.abs(uv[j][0]+uv[i][0])>tol||Math.abs(uv[j][1]-uv[i][1])>tol))continue;
      found=j;count++;
    }
    if(count!==1)return;
    mirror[i]=found;
  }
  for(let i=0;i<n;i++)if(mirror[mirror[i]]!==i||Math.abs(mesh.masses[i]-mesh.masses[mirror[i]])>1e-9*mesh.masses[i])return;
  const faces=new Set(mesh.faces.map(f=>idKey(f)));
  if(!mesh.faces.every(f=>faces.has(idKey([mirror[f[0]],mirror[f[1]],mirror[f[2]]]))))return;
  return mirror;
}
/** Order-independent name for a set of vertex ids. */
export const idKey=(ids:ArrayLike<number>)=>Array.from(ids).sort((a,b)=>a-b).join(',');

/** Mirror pairs adjacent in a Gauss-Seidel sweep. `shared[k]` is a position p
 * whose pair (p, p+1) touches common vertices; those vertices are
 * unionIds[unionStart[k] .. unionStart[k+1]). Buffers are reused between
 * rebuilds, so per-substep contact pairing does not allocate. */
export class PairPlan {
  count=0; sharedCount=0; order=new Int32Array(16);
  shared=new Int32Array(16); unionStart=new Int32Array(17); unionIds=new Int32Array(64);
  private used=new Uint8Array(16);
  constructor(count=0){this.plain(count);}
  /** Original order, no pairs. */
  plain(count:number){this.ensure(count);this.count=count;this.sharedCount=0;for(let i=0;i<count;i++)this.order[i]=i;return this;}
  private ensure(count:number){
    if(this.order.length<count){const size=Math.max(count,2*this.order.length);this.order=new Int32Array(size);this.used=new Uint8Array(size);this.shared=new Int32Array(size);this.unionStart=new Int32Array(size+1);}
  }
  /** Order items so each sits beside `partner(i)` (its mirror image, or −1).
   * `ids` holds `stride` vertex ids per item, for the shared-vertex test. */
  pair(count:number,partner:(i:number)=>number,ids:ArrayLike<number>,stride:number){
    this.ensure(count);this.count=count;this.sharedCount=0;
    const order=this.order,used=this.used;used.fill(0,0,count);
    let position=0,u=0;this.unionStart[0]=0;
    for(let i=0;i<count;i++){
      if(used[i])continue;used[i]=1;order[position++]=i;
      const j=partner(i);if(j<0||j===i||used[j])continue;
      used[j]=1;order[position++]=j;
      let overlap=false;
      for(let a=0;a<stride&&!overlap;a++)for(let b=0;b<stride;b++)if(ids[i*stride+a]===ids[j*stride+b]){overlap=true;break;}
      if(!overlap)continue;
      if(this.unionIds.length<u+2*stride){const grown=new Int32Array(2*(u+2*stride));grown.set(this.unionIds);this.unionIds=grown;}
      const start=u;
      for(let a=0;a<stride;a++)this.unionIds[u++]=ids[i*stride+a];
      for(let b=0;b<stride;b++){const v=ids[j*stride+b];let seen=false;for(let t=start;t<u;t++)if(this.unionIds[t]===v){seen=true;break;}if(!seen)this.unionIds[u++]=v;}
      this.shared[this.sharedCount++]=position-2;this.unionStart[this.sharedCount]=u;
    }
    return this;
  }
}
export const plainPlan=(count:number)=>new PairPlan(count);
/** Reorder `items` in place into mirror-paired sweep order (setup time only).
 * `name(item, map)` names an item with its vertex ids passed through `map`, so
 * the mirror image of an item is the one named name(item, mirror). */
export function arrangeInPlace<T>(items:T[],ids:(item:T)=>ArrayLike<number>,mirror:Int32Array|undefined,
  name:(item:T,map:(v:number)=>number)=>string=(item,map)=>idKey(Array.from(ids(item),map))):PairPlan {
  if(!mirror)return plainPlan(items.length);
  const index=new Map<string,number>();items.forEach((item,i)=>index.set(name(item,v=>v),i));
  const stride=Math.max(0,...items.map(item=>ids(item).length)),flat=new Int32Array(items.length*stride).fill(-1);
  items.forEach((item,i)=>Array.from(ids(item)).forEach((v,k)=>{flat[i*stride+k]=v;}));
  const pairing=new PairPlan().pair(items.length,i=>index.get(name(items[i],v=>mirror[v]))??-1,flat,stride);
  // Rewrite the plan's positions onto the reordered items.
  const sorted=Array.from(pairing.order.subarray(0,items.length),i=>items[i]);items.length=0;items.push(...sorted);
  for(let i=0;i<pairing.count;i++)pairing.order[i]=i;
  return pairing;
}
/** Solver memory per item: `stride` numbers each. Multipliers are averaged
 * over the two orders; skip flags take the minimum, so an item is skipped later
 * only if both orders agreed it could be. */
export interface ItemState { data:Float64Array; stride:number; combine?:'average'|'min'; /** Changing slice of each item's stride. */ from?:number; length?:number }
let coords=new Float64Array(96),saved=new Float64Array(96);
/** One sweep in plan order (positions, not item indices: callers keep their
 * items stored in plan order). Runs between shared pairs go straight to the
 * solver; each shared pair is solved in both orders from the same state and
 * the results averaged, so a symmetric state stays symmetric. */
export function pairedSweep(plan:PairPlan,forward:boolean,x:Float64Array,solve:(start:number,end:number,forward:boolean)=>void,state:ItemState[]=[]){
  const count=plan.count,shared=plan.shared,n=plan.sharedCount;
  if(!n){if(count)solve(0,count,forward);return;}
  if(forward){
    let cursor=0;
    for(let k=0;k<n;k++){const s=shared[k];if(s>cursor)solve(cursor,s,true);averaged(plan,k,x,solve,state);cursor=s+2;}
    if(cursor<count)solve(cursor,count,true);
  }else{
    let cursor=count;
    for(let k=n-1;k>=0;k--){const s=shared[k];if(s+2<cursor)solve(s+2,cursor,false);averaged(plan,k,x,solve,state);cursor=s;}
    if(cursor>0)solve(0,cursor,false);
  }
}
function averaged(plan:PairPlan,k:number,x:Float64Array,solve:(start:number,end:number,forward:boolean)=>void,state:ItemState[]){
  const s=plan.shared[k],ids=plan.unionIds,from=plan.unionStart[k],to=plan.unionStart[k+1];
  let size=3*(to-from);for(let q=0;q<state.length;q++)size+=2*(state[q].length??state[q].stride);
  if(coords.length<size){coords=new Float64Array(2*size);saved=new Float64Array(2*size);}
  // coords: base state; saved: result of the first order.
  let o=0;
  for(let t=from;t<to;t++){const v=3*ids[t];coords[o++]=x[v];coords[o++]=x[v+1];coords[o++]=x[v+2];}
  for(let r=0;r<state.length;r++){const st=state[r],data=st.data;for(let item=s;item<s+2;item++){const b=item*st.stride+(st.from??0);for(let q=b,e=b+(st.length??st.stride);q<e;q++)coords[o++]=data[q];}}
  solve(s,s+2,true);
  o=0;
  for(let t=from;t<to;t++){const v=3*ids[t];for(let d=0;d<3;d++){saved[o]=x[v+d];x[v+d]=coords[o];o++;}}
  for(let r=0;r<state.length;r++){const st=state[r],data=st.data;for(let item=s;item<s+2;item++){const b=item*st.stride+(st.from??0);for(let q=b,e=b+(st.length??st.stride);q<e;q++){saved[o]=data[q];data[q]=coords[o];o++;}}}
  solve(s,s+2,false);
  o=0;
  for(let t=from;t<to;t++){const v=3*ids[t];for(let d=0;d<3;d++){x[v+d]=(x[v+d]+saved[o])/2;o++;}}
  for(let r=0;r<state.length;r++){const st=state[r],data=st.data,min=st.combine==='min';for(let item=s;item<s+2;item++){const b=item*st.stride+(st.from??0);for(let q=b,e=b+(st.length??st.stride);q<e;q++){data[q]=min?Math.min(data[q],saved[o]):(data[q]+saved[o])/2;o++;}}}
}
