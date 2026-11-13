import type { PaperMesh } from '../geometry/dart';
import type { V3 } from './math';

type Ids = [number,number,number,number];
export interface SelfContact { kind:'vertex-face'|'edge-edge'; ids:Ids; sign:number; key:number }
const GAP_EPS=1e-9, PARAM_EPS=1e-7, PAIR_MARGIN=.003, BOUND=.05, COLD_GAP=3e-5;

/** Fixed topology BVH, refitted to swept bounds. No spatial-cell explosion for
 * fast motion and no all-pairs feature list retained between substeps. */
class SweptTree {
  readonly bounds:Float64Array;
  readonly leaves:Int32Array;
  private right:Int32Array;
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
    this.bounds=new Float64Array(leaves.length*6);
  }
  /** `boxes` holds six bounds per feature: low xyz, then high xyz. */
  refit(boxes:Float64Array){
    const bounds=this.bounds,leaves=this.leaves,rightChild=this.right;
    for(let node=leaves.length-1;node>=0;node--){
      const offset=node*6,index=leaves[node];
      if(index>=0){const source=index*6;for(let d=0;d<6;d++)bounds[offset+d]=boxes[source+d];}
      else{
        const left=offset+6,right=rightChild[node]*6;
        for(let d=0;d<3;d++){bounds[offset+d]=Math.min(bounds[left+d],bounds[right+d]);bounds[offset+d+3]=Math.max(bounds[left+d+3],bounds[right+d+3]);}
      }
    }
  }
}

const rootCuts=new Float64Array(4),rootScratch=new Float64Array(8);
/** Allocation-free root isolation; returns the number of roots written to `out`. */
function cubicRoots(c0:number,c1:number,c2:number,c3:number,out:Float64Array):number{
  const magnitude=Math.max(Math.abs(c0),Math.abs(c1),Math.abs(c2),Math.abs(c3));if(magnitude<1e-30)return 0;
  const a=c0/magnitude,b=c1/magnitude,d=c2/magnitude,e=c3/magnitude,epsilon=1e-12,cuts=rootCuts;
  let cutCount=2;cuts[0]=0;cuts[1]=1;
  if(Math.abs(e)<epsilon){if(Math.abs(d)>epsilon){const t=-b/(2*d);if(t>0&&t<1)cuts[cutCount++]=t;}}
  else{
    const disc=4*d*d-12*e*b;
    if(disc>=0){
      const root=Math.sqrt(disc),t0=(-2*d-root)/(6*e),t1=(-2*d+root)/(6*e);
      if(t0>0&&t0<1)cuts[cutCount++]=t0;if(t1>0&&t1<1)cuts[cutCount++]=t1;
    }
  }
  // At most four cuts: insertion sort.
  for(let i=1;i<cutCount;i++){const t=cuts[i];let j=i;for(;j>0&&cuts[j-1]>t;j--)cuts[j]=cuts[j-1];cuts[j]=t;}
  let count=0;
  for(let i=0;i<cutCount;i++){const t=cuts[i];if(Math.abs(((e*t+d)*t+b)*t+a)<epsilon)out[count++]=t;}
  for(let i=1;i<cutCount;i++){
    let lo=cuts[i-1],hi=cuts[i],vl=((e*lo+d)*lo+b)*lo+a;if(vl*(((e*hi+d)*hi+b)*hi+a)>=0)continue;
    // The cubic is monotonic between cuts: Newton steps, kept inside a
    // shrinking bracket by bisection, converge in a handful of evaluations.
    let t=(lo+hi)/2;
    for(let j=0;j<40&&hi-lo>1e-13;j++){
      const vm=((e*t+d)*t+b)*t+a;
      if(vl*vm<=0)hi=t;else{lo=t;vl=vm;}
      if(vm===0)break;
      const next=t-vm/((3*e*t+2*d)*t+b);
      t=next>lo&&next<hi?next:(lo+hi)/2;
    }
    out[count++]=t;
  }
  for(let i=1;i<count;i++){const t=out[i];let j=i;for(;j>0&&out[j-1]>t;j--)out[j]=out[j-1];out[j]=t;}
  // Merge roots closer than 1e-8 to their sorted predecessor.
  let kept=0,previous=0;
  for(let i=0;i<count;i++){const t=out[i];if(!i||t-previous>1e-8)out[kept++]=t;previous=t;}
  return kept;
}
/** Isolate every root of a cubic in [0,1] using its derivative extrema. Unlike
 * endpoint sign tests this also finds an even number of crossings in a step. */
export function coplanarityRoots(c:number[]):number[]{
  const count=cubicRoots(c[0],c[1],c[2],c[3],rootScratch);
  return Array.from(rootScratch.subarray(0,count));
}

/** Frictionless, zero-clearance unilateral projection. Recomputing the normal
 * and projected weights gives a distance gradient with zero net force/torque.
 * Negative weights are intentional for a feature that moved outside its face
 * after impact; only swept detection can create such a constraint.
 * Scalar hot path on flat xyz coordinates: no temporary vectors. */
function project(x:Float64Array,w:ArrayLike<number>,i0:number,i1:number,i2:number,i3:number,vf:boolean,sign:number,bounded=false){
  const a=3*i0,b=3*i1,c=3*i2,d=3*i3,origin=vf?b:c,edgeOrigin=vf?b:a,edgeEnd=vf?c:b;
  const rx=x[a]-x[origin],ry=x[a+1]-x[origin+1],rz=x[a+2]-x[origin+2];
  const ex=x[edgeEnd]-x[edgeOrigin],ey=x[edgeEnd+1]-x[edgeOrigin+1],ez=x[edgeEnd+2]-x[edgeOrigin+2];
  const fx=x[d]-x[origin],fy=x[d+1]-x[origin+1],fz=x[d+2]-x[origin+2];
  const nx=ey*fz-ez*fy,ny=ez*fx-ex*fz,nz=ex*fy-ey*fx,nn=nx*nx+ny*ny+nz*nz;
  const raw=rx*nx+ry*ny+rz*nz;if(raw*sign>=0)return raw*raw>COLD_GAP*COLD_GAP*nn?2:1;
  const ee=ex*ex+ey*ey+ez*ez,ff=fx*fx+fy*fy+fz*fz,ef=ex*fx+ey*fy+ez*fz;
  if(nn<1e-24||nn<1e-12*ee*ff)return 0;
  const re=rx*ex+ry*ey+rz*ez,rf=rx*fx+ry*fy+rz*fz;
  const u=(vf?re*ff-rf*ef:ef*rf-ff*re)/nn,v=(rf*ee-re*ef)/nn;
  // A contact kept from the previous substep saw no impact in this one: it
  // may only act where the features actually overlap.
  if(bounded&&!(u>=-BOUND&&v>=-BOUND&&(vf?u+v<=1+BOUND:u<=1+BOUND&&v<=1+BOUND)))return 0;
  const c0=vf?1:1-u,c1=vf?u+v-1:u,c2=vf?-u:v-1,c3=-v;
  const den=w[i0]*c0*c0+w[i1]*c1*c1+w[i2]*c2*c2+w[i3]*c3*c3;if(den<=0)return 0;
  const impulse=-raw/(nn*den),s0=w[i0]*c0*impulse,s1=w[i1]*c1*impulse,s2=w[i2]*c2*impulse,s3=w[i3]*c3*impulse;
  x[a]+=s0*nx;x[a+1]+=s0*ny;x[a+2]+=s0*nz;x[b]+=s1*nx;x[b+1]+=s1*ny;x[b+2]+=s1*nz;
  x[c]+=s2*nx;x[c+1]+=s2*ny;x[c+2]+=s2*nz;x[d]+=s3*nx;x[d+1]+=s3*ny;x[d+2]+=s3*nz;
  return 0;
}
const oneX=new Float64Array(12),oneW=new Float64Array(4);
/** One contact on per-vertex arrays; the same kernel as the solver sweep. */
export function solveSelfContact(p:V3[],invMass:number[],contact:SelfContact){
  contact.ids.forEach((id,j)=>{oneX[3*j]=p[id][0];oneX[3*j+1]=p[id][1];oneX[3*j+2]=p[id][2];oneW[j]=invMass[id];});
  project(oneX,oneW,0,1,2,3,contact.kind==='vertex-face',contact.sign);
  contact.ids.forEach((id,j)=>{p[id][0]=oneX[3*j];p[id][1]=oneX[3*j+1];p[id][2]=oneX[3*j+2];});
}

export class SelfContactSolver {
  // Flat one-ring adjacency bitset: `words` 32-bit words per vertex.
  private neighbors:Uint32Array; private words:number;
  private edges:Int32Array; private faces:Int32Array;
  private faceTree?:SweptTree; private edgeTree?:SweptTree;
  // Dense per-pair state replaces hashing in the narrow phase. A pair key is
  // vertex*faces+face, or vertices*faces+edge*edges+edge; one byte each.
  private previousSign:Int8Array; private activeFlag:Uint8Array;
  // Active contacts in detection order: four vertex ids, pair key, side, kind.
  /** Active contacts this substep. */
  count=0;
  private ids=new Int32Array(256); private keys=new Int32Array(64); private signs=new Int8Array(64); private vertexFace=new Uint8Array(64); private carried=new Uint8Array(64); private idle=new Uint8Array(64);
  private previousCount=0; private previousKeys=new Int32Array(64);
  private oldFlat:Float64Array; private newFlat:Float64Array; private ownOld:Float64Array; private ownNew:Float64Array;
  private low:Float64Array; private high:Float64Array;
  private faceBoxes:Float64Array; private edgeBoxes:Float64Array;
  private roots=new Float64Array(8);
  // Broad-phase pair lists, in BVH visit order, built from bounds padded by
  // PAIR_MARGIN. They remain a superset of every overlapping pair until some
  // vertex drifts that far from `reference` in the mean-translating frame.
  private reference:Float64Array; private pairsValid=false;
  private vertexFacePairs=new Int32Array(2048); private vertexFacePairCount=0;
  private edgePairs=new Int32Array(2048); private edgePairCount=0;
  stats={narrowTests:0,vertexFace:0,edgeEdge:0};
  constructor(mesh:PaperMesh){
    // Compact material adjacency: hot broad-phase visitors test bits instead
    // of hashing vertex IDs. One bit per pair, with the same one-ring exclusion.
    const count=mesh.rest.length,words=this.words=Math.ceil(count/32);
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
    const pairs=count*mesh.faces.length+edges.length*edges.length;
    this.previousSign=new Int8Array(pairs);this.activeFlag=new Uint8Array(pairs);
    this.oldFlat=this.ownOld=new Float64Array(count*3);this.newFlat=this.ownNew=new Float64Array(count*3);
    this.low=new Float64Array(count*3);this.high=new Float64Array(count*3);this.reference=new Float64Array(count*3);
    this.faceBoxes=new Float64Array(mesh.faces.length*6);this.edgeBoxes=new Float64Array(edges.length*6);
  }
  private forget(){
    for(let i=0;i<this.previousCount;i++)this.previousSign[this.previousKeys[i]]=0;
    for(let i=0;i<this.count;i++)this.activeFlag[this.keys[i]]=0;
  }
  reset(){this.forget();this.pairsValid=false;this.previousCount=0;this.count=0;this.stats={narrowTests:0,vertexFace:0,edgeEdge:0};}
  beginStep(){
    this.forget();
    if(this.previousKeys.length<this.count)this.previousKeys=new Int32Array(this.keys.length);
    for(let i=0;i<this.count;i++){this.previousKeys[i]=this.keys[i];this.previousSign[this.keys[i]]=this.signs[i];}
    this.previousCount=this.count;this.count=0;
    this.stats={narrowTests:0,vertexFace:0,edgeEdge:0};
  }
  /** Keep last substep's contacts that still touch inside their features at
   * `old` (or hover within `gap` on the side they were pushed to), with their
   * side, so every sweep of this substep enforces them. Fresh crossings are
   * added by detection. */
  carry(old:Float64Array,gap=0){
    this.oldFlat=this.newFlat=old;
    const faceCount=this.faces.length/3,edgeCount=this.edges.length/2,vertexFaceKeys=old.length/3*faceCount;
    for(let n=0;n<this.previousCount;n++){
      const key=this.previousKeys[n],vf=key<vertexFaceKeys;
      let i0:number,i1:number,i2:number,i3:number;
      if(vf){const fi=key%faceCount;i0=(key-fi)/faceCount;i1=this.faces[3*fi];i2=this.faces[3*fi+1];i3=this.faces[3*fi+2];}
      else{const pair=key-vertexFaceKeys,j=pair%edgeCount,i=(pair-j)/edgeCount;i0=this.edges[2*i];i1=this.edges[2*i+1];i2=this.edges[2*j];i3=this.edges[2*j+1];}
      const ia=3*i0,ib=3*(vf?i1:i2),ic=3*(vf?i2:i1),ie=3*(vf?i1:i0),jf=3*i3;
      const rx=old[ia]-old[ib],ry=old[ia+1]-old[ib+1],rz=old[ia+2]-old[ib+2];
      const ex=old[ic]-old[ie],ey=old[ic+1]-old[ie+1],ez=old[ic+2]-old[ie+2];
      const fx=old[jf]-old[ib],fy=old[jf+1]-old[ib+1],fz=old[jf+2]-old[ib+2];
      const nx=ey*fz-ez*fy,ny=ez*fx-ex*fz,nz=ex*fy-ey*fx,a=rx*nx+ry*ny+rz*nz,nn=nx*nx+ny*ny+nz*nz;
      if(!(nn>1e-24))continue;
      // Touching, or still close on the side it was last pushed to.
      const sign=this.previousSign[key],near=a*a<=GAP_EPS*GAP_EPS*nn||(a*sign>0&&a*a<=gap*gap*nn);
      if(near&&this.inside(0,i0,i1,i2,i3,vf))this.add(i0,i1,i2,i3,vf,sign,key,true);
    }
  }
  /** Size of the cached broad-phase pair lists (zero before the first detection). */
  get pairCount(){return this.pairsValid?this.vertexFacePairCount+this.edgePairCount:0;}
  get contacts():SelfContact[]{
    return Array.from({length:this.count},(_,i)=>({ids:Array.from(this.ids.subarray(4*i,4*i+4)) as Ids,kind:this.vertexFace[i]?'vertex-face' as const:'edge-edge' as const,sign:this.signs[i],key:this.keys[i]}));
  }
  private add(i0:number,i1:number,i2:number,i3:number,vf:boolean,sign:number,key:number,carried=false){
    if(this.count===this.keys.length){
      const size=this.count*2,ids=new Int32Array(size*4),keys=new Int32Array(size),signs=new Int8Array(size),kinds=new Uint8Array(size);
      const carried=new Uint8Array(size);carried.set(this.carried);this.carried=carried;const idle=new Uint8Array(size);idle.set(this.idle);this.idle=idle;
      ids.set(this.ids);keys.set(this.keys);signs.set(this.signs);kinds.set(this.vertexFace);this.ids=ids;this.keys=keys;this.signs=signs;this.vertexFace=kinds;
    }
    const n=this.count++;this.ids[4*n]=i0;this.ids[4*n+1]=i1;this.ids[4*n+2]=i2;this.ids[4*n+3]=i3;
    this.keys[n]=key;this.signs[n]=sign;this.vertexFace[n]=vf?1:0;this.carried[n]=carried?1:0;this.idle[n]=0;this.activeFlag[key]=1;
    if(vf)this.stats.vertexFace++;else this.stats.edgeEdge++;
  }
  /** Projected impact coordinates at time t of the step lie inside both features. */
  private inside(t:number,i0:number,i1:number,i2:number,i3:number,vf:boolean){
    const O=this.oldFlat,P=this.newFlat,a=3*i0,b=3*i1,c=3*i2,d=3*i3;
    const ax=O[a]+t*(P[a]-O[a]),ay=O[a+1]+t*(P[a+1]-O[a+1]),az=O[a+2]+t*(P[a+2]-O[a+2]);
    const bx=O[b]+t*(P[b]-O[b]),by=O[b+1]+t*(P[b+1]-O[b+1]),bz=O[b+2]+t*(P[b+2]-O[b+2]);
    const cx=O[c]+t*(P[c]-O[c]),cy=O[c+1]+t*(P[c+1]-O[c+1]),cz=O[c+2]+t*(P[c+2]-O[c+2]);
    const dx=O[d]+t*(P[d]-O[d]),dy=O[d+1]+t*(P[d+1]-O[d+1]),dz=O[d+2]+t*(P[d+2]-O[d+2]);
    const rx=vf?ax-bx:ax-cx,ry=vf?ay-by:ay-cy,rz=vf?az-bz:az-cz;
    const ex=vf?cx-bx:bx-ax,ey=vf?cy-by:by-ay,ez=vf?cz-bz:bz-az;
    const fx=vf?dx-bx:dx-cx,fy=vf?dy-by:dy-cy,fz=vf?dz-bz:dz-cz;
    const nx=ey*fz-ez*fy,ny=ez*fx-ex*fz,nz=ex*fy-ey*fx;
    const ee=ex*ex+ey*ey+ez*ez,ff=fx*fx+fy*fy+fz*fz,ef=ex*fx+ey*fy+ez*fz,den=nx*nx+ny*ny+nz*nz;
    // Degenerate triangles and almost parallel edges have no reliable normal.
    if(den<1e-24||den<1e-12*ee*ff)return false;
    const re=rx*ex+ry*ey+rz*ez,rf=rx*fx+ry*fy+rz*fz;
    const u=vf?(re*ff-rf*ef)/den:(ef*rf-ff*re)/den,v=(rf*ee-re*ef)/den;
    return u>=-PARAM_EPS&&v>=-PARAM_EPS&&(vf?u+v<=1+PARAM_EPS:u<=1+PARAM_EPS&&v<=1+PARAM_EPS);
  }
  private test(i0:number,i1:number,i2:number,i3:number,vf:boolean,key:number){
    this.stats.narrowTests++;
    // Cubic coplanarity polynomial of the linearly moving features.
    const O=this.oldFlat,P=this.newFlat,ia=3*i0,ib=3*(vf?i1:i2),ic=3*(vf?i2:i1),ie=3*(vf?i1:i0),jf=3*i3;
    const rx=O[ia]-O[ib],ry=O[ia+1]-O[ib+1],rz=O[ia+2]-O[ib+2];
    const ex=O[ic]-O[ie],ey=O[ic+1]-O[ie+1],ez=O[ic+2]-O[ie+2];
    const fx=O[jf]-O[ib],fy=O[jf+1]-O[ib+1],fz=O[jf+2]-O[ib+2];
    const drx=P[ia]-P[ib]-rx,dry=P[ia+1]-P[ib+1]-ry,drz=P[ia+2]-P[ib+2]-rz;
    const dex=P[ic]-P[ie]-ex,dey=P[ic+1]-P[ie+1]-ey,dez=P[ic+2]-P[ie+2]-ez;
    const dfx=P[jf]-P[ib]-fx,dfy=P[jf+1]-P[ib+1]-fy,dfz=P[jf+2]-P[ib+2]-fz;
    const nx=ey*fz-ez*fy,ny=ez*fx-ex*fz,nz=ex*fy-ey*fx;
    const mx=dey*fz-dez*fy+ey*dfz-ez*dfy,my=dez*fx-dex*fz+ez*dfx-ex*dfz,mz=dex*fy-dey*fx+ex*dfy-ey*dfx;
    const qx=dey*dfz-dez*dfy,qy=dez*dfx-dex*dfz,qz=dex*dfy-dey*dfx;
    const a=rx*nx+ry*ny+rz*nz,b=drx*nx+dry*ny+drz*nz+rx*mx+ry*my+rz*mz;
    const c=drx*mx+dry*my+drz*mz+rx*qx+ry*qy+rz*qz,d=drx*qx+dry*qy+drz*qz,nn=nx*nx+ny*ny+nz*nz;
    // Bernstein convex hull: all four same-sign control values exclude roots.
    const b1=a+b/3,b2=a+2*b/3+c/3,b3=a+b+c+d;
    const sameSign=(a>0&&b1>0&&b2>0&&b3>0)||(a<0&&b1<0&&b2<0&&b3<0);
    // Nearly every pair stays on one side without touching: decide that before
    // reading the (cache-unfriendly) per-pair history.
    const touching=nn>1e-24&&a*a<=GAP_EPS*GAP_EPS*nn;
    if(sameSign&&!touching)return;
    if(this.activeFlag[key])return;
    const prior=touching?this.previousSign[key]:0;
    if(!prior&&touching)return;
    let sign=0;
    if(prior&&this.inside(0,i0,i1,i2,i3,vf))sign=prior;
    else if(sameSign)return;
    if(!sign){
      const roots=this.roots,count=cubicRoots(a,b,c,d,roots);
      for(let k=0;k<count;k++){
        const t=roots[k];
        // Perfect initial coincidence has no stack order. Let it open; do not
        // create arbitrary opposition at a folded seam or a grazing touch.
        if(t<1e-8&&(nn<=1e-24||touching))continue;
        if(!this.inside(t,i0,i1,i2,i3,vf))continue;
        const before=Math.max(0,t-1e-6),after=Math.min(1,t+1e-6);
        const valueBefore=((d*before+c)*before+b)*before+a,valueAfter=((d*after+c)*after+b)*after+a;
        if(valueBefore*valueAfter>0)continue;
        sign=Math.sign(valueBefore);if(sign)break;
      }
    }
    if(!sign)return;
    this.add(i0,i1,i2,i3,vf,sign,key);
  }
  /** Feature bounds pad once per member vertex, as the swept box always has. */
  private fill(ids:Int32Array,size:number,boxes:Float64Array){
    const low=this.low,high=this.high;
    for(let f=0,features=ids.length/size;f<features;f++){
      let lx=Infinity,ly=Infinity,lz=Infinity,hx=-Infinity,hy=-Infinity,hz=-Infinity;
      for(let j=0;j<size;j++){
        const k=3*ids[f*size+j];
        lx=Math.min(lx,low[k])-GAP_EPS;ly=Math.min(ly,low[k+1])-GAP_EPS;lz=Math.min(lz,low[k+2])-GAP_EPS;
        hx=Math.max(hx,high[k])+GAP_EPS;hy=Math.max(hy,high[k+1])+GAP_EPS;hz=Math.max(hz,high[k+2])+GAP_EPS;
      }
      const b=6*f;boxes[b]=lx;boxes[b+1]=ly;boxes[b+2]=lz;boxes[b+3]=hx;boxes[b+4]=hy;boxes[b+5]=hz;
    }
  }
  /** Per-vertex array form of detectFlat. */
  detect(old:V3[],p:V3[]){
    const O=this.ownOld,P=this.ownNew;
    for(let i=0;i<p.length;i++){const o=old[i],q=p[i],k=3*i;O[k]=o[0];O[k+1]=o[1];O[k+2]=o[2];P[k]=q[0];P[k+1]=q[1];P[k+2]=q[2];}
    this.detectFlat(O,P);
  }
  /** Swept detection between flat xyz positions before and after the step. */
  detectFlat(O:Float64Array,P:Float64Array){
    this.oldFlat=O;this.newFlat=P;
    const count=O.length/3,low=this.low,high=this.high;
    // A linearly translating frame preserves every impact time and prevents a
    // fast rigid launch from inflating every swept box by the flight distance.
    let mx=0,my=0,mz=0;
    for(let k=0;k<3*count;k+=3){mx+=(P[k]-O[k])/count;my+=(P[k+1]-O[k+1])/count;mz+=(P[k+2]-O[k+2])/count;}
    for(let k=0;k<3*count;k+=3){
      low[k]=Math.min(O[k],P[k]-mx);low[k+1]=Math.min(O[k+1],P[k+1]-my);low[k+2]=Math.min(O[k+2],P[k+2]-mz);
      high[k]=Math.max(O[k],P[k]-mx);high[k+1]=Math.max(O[k+1],P[k+1]-my);high[k+2]=Math.max(O[k+2],P[k+2]-mz);
    }
    const faces=this.faces,edges=this.edges,faceCount=faces.length/3,edgeCount=edges.length/2;
    const faceBoxes=this.faceBoxes,edgeBoxes=this.edgeBoxes;
    this.fill(faces,3,faceBoxes);this.fill(edges,2,edgeBoxes);
    // Largest per-axis drift of either endpoint from the pair-list reference,
    // after removing the common translation.
    const R=this.reference;let stale=!this.pairsValid;
    if(!stale){
      let tx=0,ty=0,tz=0;
      for(let k=0;k<3*count;k+=3){tx+=(O[k]-R[k])/count;ty+=(O[k+1]-R[k+1])/count;tz+=(O[k+2]-R[k+2])/count;}
      const ux=tx+mx,uy=ty+my,uz=tz+mz;
      for(let k=0;k<3*count&&!stale;k+=3){
        stale=!(Math.abs(O[k]-R[k]-tx)<=PAIR_MARGIN&&Math.abs(O[k+1]-R[k+1]-ty)<=PAIR_MARGIN&&Math.abs(O[k+2]-R[k+2]-tz)<=PAIR_MARGIN
          &&Math.abs(P[k]-R[k]-ux)<=PAIR_MARGIN&&Math.abs(P[k+1]-R[k+1]-uy)<=PAIR_MARGIN&&Math.abs(P[k+2]-R[k+2]-uz)<=PAIR_MARGIN);
      }
    }
    if(stale)this.buildPairs();
    const pairs=this.vertexFacePairs,pairCount=this.vertexFacePairCount;
    for(let n=0;n<pairCount;n++){
      const v=pairs[2*n],fi=pairs[2*n+1],k=3*v,b=6*fi;
      if(!(faceBoxes[b]<=high[k]+GAP_EPS&&low[k]-GAP_EPS<=faceBoxes[b+3]&&faceBoxes[b+1]<=high[k+1]+GAP_EPS&&low[k+1]-GAP_EPS<=faceBoxes[b+4]&&faceBoxes[b+2]<=high[k+2]+GAP_EPS&&low[k+2]-GAP_EPS<=faceBoxes[b+5]))continue;
      this.test(v,faces[3*fi],faces[3*fi+1],faces[3*fi+2],true,v*faceCount+fi);
    }
    const edgePairList=this.edgePairs,edgePairCount=this.edgePairCount,base=count*faceCount;
    for(let n=0;n<edgePairCount;n++){
      const i=edgePairList[2*n],j=edgePairList[2*n+1],a=6*i,b=6*j;
      if(!(edgeBoxes[b]<=edgeBoxes[a+3]&&edgeBoxes[a]<=edgeBoxes[b+3]&&edgeBoxes[b+1]<=edgeBoxes[a+4]&&edgeBoxes[a+1]<=edgeBoxes[b+4]&&edgeBoxes[b+2]<=edgeBoxes[a+5]&&edgeBoxes[a+2]<=edgeBoxes[b+5]))continue;
      this.test(edges[2*i],edges[2*i+1],edges[2*j],edges[2*j+1],false,base+i*edgeCount+j);
    }
  }
  /** Collect nonadjacent feature pairs whose padded swept bounds overlap. */
  private buildPairs(){
    const low=this.low,high=this.high,faces=this.faces,edges=this.edges,count=low.length/3,edgeCount=edges.length/2;
    const neighbors=this.neighbors,words=this.words,pad=2*PAIR_MARGIN;
    this.reference.set(this.oldFlat);this.pairsValid=true;
    this.vertexFacePairCount=0;this.edgePairCount=0;
    if(this.faceTree){
      this.faceTree.refit(this.faceBoxes);
      const bounds=this.faceTree.bounds,leaves=this.faceTree.leaves,escape=this.faceTree.escape,nodes=leaves.length;
      for(let v=0;v<count;v++){
        const k=3*v,lx=low[k]-GAP_EPS-pad,ly=low[k+1]-GAP_EPS-pad,lz=low[k+2]-GAP_EPS-pad,hx=high[k]+GAP_EPS+pad,hy=high[k+1]+GAP_EPS+pad,hz=high[k+2]+GAP_EPS+pad,row=v*words;
        for(let node=0;node<nodes;){
          const b=node*6;
          if(!(bounds[b]<=hx&&lx<=bounds[b+3]&&bounds[b+1]<=hy&&ly<=bounds[b+4]&&bounds[b+2]<=hz&&lz<=bounds[b+5])){node=escape[node];continue;}
          const fi=leaves[node++];if(fi<0)continue;
          const f0=faces[3*fi],f1=faces[3*fi+1],f2=faces[3*fi+2];
          if((neighbors[row+(f0>>>5)]&(1<<(f0&31)))||(neighbors[row+(f1>>>5)]&(1<<(f1&31)))||(neighbors[row+(f2>>>5)]&(1<<(f2&31))))continue;
          if(2*this.vertexFacePairCount===this.vertexFacePairs.length){const grown=new Int32Array(this.vertexFacePairs.length*2);grown.set(this.vertexFacePairs);this.vertexFacePairs=grown;}
          const n=this.vertexFacePairCount++;this.vertexFacePairs[2*n]=v;this.vertexFacePairs[2*n+1]=fi;
        }
      }
    }
    if(this.edgeTree){
      const boxes=this.edgeBoxes;this.edgeTree.refit(boxes);
      const bounds=this.edgeTree.bounds,leaves=this.edgeTree.leaves,escape=this.edgeTree.escape,nodes=leaves.length;
      for(let i=0;i<edgeCount;i++){
        const k=6*i,lx=boxes[k]-pad,ly=boxes[k+1]-pad,lz=boxes[k+2]-pad,hx=boxes[k+3]+pad,hy=boxes[k+4]+pad,hz=boxes[k+5]+pad;
        const rowA=edges[2*i]*words,rowB=edges[2*i+1]*words;
        for(let node=0;node<nodes;){
          const n=node*6;
          if(!(bounds[n]<=hx&&lx<=bounds[n+3]&&bounds[n+1]<=hy&&ly<=bounds[n+4]&&bounds[n+2]<=hz&&lz<=bounds[n+5])){node=escape[node];continue;}
          const j=leaves[node++];if(j<=i)continue;
          const c=edges[2*j],d=edges[2*j+1],cw=c>>>5,dw=d>>>5,cb=1<<(c&31),db=1<<(d&31);
          if((neighbors[rowA+cw]&cb)||(neighbors[rowA+dw]&db)||(neighbors[rowB+cw]&cb)||(neighbors[rowB+dw]&db))continue;
          if(2*this.edgePairCount===this.edgePairs.length){const grown=new Int32Array(this.edgePairs.length*2);grown.set(this.edgePairs);this.edgePairs=grown;}
          const m=this.edgePairCount++;this.edgePairs[2*m]=i;this.edgePairs[2*m+1]=j;
        }
      }
    }
  }
  /** One projection sweep. With a `sweep` index, separated contacts are
   * revisited every other sweep, and clearly separated ones every eighth. */
  solveFlat(x:Float64Array,w:ArrayLike<number>,forward=true,sweep=0){
    const ids=this.ids,signs=this.signs,kinds=this.vertexFace,carried=this.carried,idle=this.idle,count=this.count;
    const skip=sweep%8===0?3:sweep%2===0?2:1;
    for(let k=0;k<count;k++){
      const j=forward?k:count-1-k;if(idle[j]>=skip)continue;
      idle[j]=project(x,w,ids[4*j],ids[4*j+1],ids[4*j+2],ids[4*j+3],kinds[j]===1,signs[j],carried[j]===1);
    }
  }
  /** Per-vertex array form of solveFlat. */
  solve(p:V3[],invMass:number[],forward=true){
    const x=new Float64Array(p.length*3);
    for(let i=0;i<p.length;i++){x[3*i]=p[i][0];x[3*i+1]=p[i][1];x[3*i+2]=p[i][2];}
    this.solveFlat(x,invMass,forward);
    for(let i=0;i<p.length;i++){p[i][0]=x[3*i];p[i][1]=x[3*i+1];p[i][2]=x[3*i+2];}
  }
}
