import type { PaperMesh } from '../geometry/dart';
import { cross, dot, length, scale, sub, type V3 } from './math';

type Ids = [number,number,number,number];
type Bounds = [number,number,number,number,number,number];
export interface SelfContact { kind:'vertex-face'|'edge-edge'; ids:Ids; sign:number; key:number }
const GAP_EPS=1e-9, PARAM_EPS=1e-7;
const emptyBounds=():Bounds=>[Infinity,Infinity,Infinity,-Infinity,-Infinity,-Infinity];
function overlaps(a:Bounds,b:Bounds){return a[0]<=b[3]&&b[0]<=a[3]&&a[1]<=b[4]&&b[1]<=a[4]&&a[2]<=b[5]&&b[2]<=a[5];}
function sweptBounds(ids:readonly number[],old:V3[],p:V3[],motion:V3):Bounds {
  const box=emptyBounds();
  for(const i of ids)for(let d=0;d<3;d++){box[d]=Math.min(box[d],old[i][d],p[i][d]-motion[d])-GAP_EPS;box[d+3]=Math.max(box[d+3],old[i][d],p[i][d]-motion[d])+GAP_EPS;}
  return box;
}

/** Fixed topology BVH, refitted to swept bounds. No spatial-cell explosion for
 * fast motion and no all-pairs feature list retained between substeps. */
class SweptTree {
  box=emptyBounds(); left?:SweptTree; right?:SweptTree; index=-1;
  constructor(indices:number[],centers:V3[]){
    if(indices.length===1){this.index=indices[0];return;}
    const ranges=[0,1,2].map(d=>Math.max(...indices.map(i=>centers[i][d]))-Math.min(...indices.map(i=>centers[i][d])));
    const axis=ranges.indexOf(Math.max(...ranges));indices.sort((a,b)=>centers[a][axis]-centers[b][axis]||a-b);
    const mid=indices.length>>1;this.left=new SweptTree(indices.slice(0,mid),centers);this.right=new SweptTree(indices.slice(mid),centers);
  }
  refit(boxes:Bounds[]){
    if(this.index>=0){this.box=boxes[this.index];return;}
    this.left!.refit(boxes);this.right!.refit(boxes);
    for(let d=0;d<3;d++){this.box[d]=Math.min(this.left!.box[d],this.right!.box[d]);this.box[d+3]=Math.max(this.left!.box[d+3],this.right!.box[d+3]);}
  }
  query(box:Bounds,visit:(index:number)=>void){
    if(!overlaps(box,this.box))return;
    if(this.index>=0)visit(this.index);else{this.left!.query(box,visit);this.right!.query(box,visit);}
  }
}

/** Isolate every root of a cubic in [0,1] using its derivative extrema. Unlike
 * endpoint sign tests this also finds an even number of crossings in a step. */
export function coplanarityRoots(c:number[]):number[]{
  const magnitude=Math.max(...c.map(Math.abs));if(magnitude<1e-30)return [];
  const [a,b,d,e]=c.map(v=>v/magnitude),value=(t:number)=>((e*t+d)*t+b)*t+a;
  const cuts=[0,1],epsilon=1e-12;
  if(Math.abs(e)<epsilon){if(Math.abs(d)>epsilon){const t=-b/(2*d);if(t>0&&t<1)cuts.push(t);}}
  else{
    const disc=4*d*d-12*e*b;
    if(disc>=0){const root=Math.sqrt(disc);for(const t of [(-2*d-root)/(6*e),(-2*d+root)/(6*e)])if(t>0&&t<1)cuts.push(t);}
  }
  cuts.sort((x,y)=>x-y);const roots:number[]=[];
  for(const t of cuts)if(Math.abs(value(t))<epsilon)roots.push(t);
  for(let i=1;i<cuts.length;i++){
    let lo=cuts[i-1],hi=cuts[i],vl=value(lo);if(vl*value(hi)>=0)continue;
    for(let j=0;j<36;j++){const mid=(lo+hi)/2,vm=value(mid);if(vl*vm<=0)hi=mid;else{lo=mid;vl=vm;}}
    roots.push((lo+hi)/2);
  }
  return roots.sort((x,y)=>x-y).filter((t,i,list)=>!i||t-list[i-1]>1e-8);
}

function vectors(p:V3[],ids:Ids,kind:SelfContact['kind']){
  const [a,b,c,d]=ids.map(i=>p[i]);
  return kind==='vertex-face'?[sub(a,b),sub(c,b),sub(d,b)]:[sub(a,c),sub(b,a),sub(d,c)];
}
function polynomial(old:V3[],p:V3[],ids:Ids,kind:SelfContact['kind']){
  const vf=kind==='vertex-face',a=old[ids[0]],b=old[ids[vf?1:2]],c=old[ids[vf?2:1]],eo=old[ids[vf?1:0]],f=old[ids[3]];
  const ap=p[ids[0]],bp=p[ids[vf?1:2]],cp=p[ids[vf?2:1]],ep=p[ids[vf?1:0]],fp=p[ids[3]];
  const rx=a[0]-b[0],ry=a[1]-b[1],rz=a[2]-b[2],ex=c[0]-eo[0],ey=c[1]-eo[1],ez=c[2]-eo[2],fx=f[0]-b[0],fy=f[1]-b[1],fz=f[2]-b[2];
  const drx=ap[0]-bp[0]-rx,dry=ap[1]-bp[1]-ry,drz=ap[2]-bp[2]-rz;
  const dex=cp[0]-ep[0]-ex,dey=cp[1]-ep[1]-ey,dez=cp[2]-ep[2]-ez,dfx=fp[0]-bp[0]-fx,dfy=fp[1]-bp[1]-fy,dfz=fp[2]-bp[2]-fz;
  const nx=ey*fz-ez*fy,ny=ez*fx-ex*fz,nz=ex*fy-ey*fx;
  const mx=dey*fz-dez*fy+ey*dfz-ez*dfy,my=dez*fx-dex*fz+ez*dfx-ex*dfz,mz=dex*fy-dey*fx+ex*dfy-ey*dfx;
  const qx=dey*dfz-dez*dfy,qy=dez*dfx-dex*dfz,qz=dex*dfy-dey*dfx;
  return [rx*nx+ry*ny+rz*nz,drx*nx+dry*ny+drz*nz+rx*mx+ry*my+rz*mz,
    drx*mx+dry*my+drz*mz+rx*qx+ry*qy+rz*qz,drx*qx+dry*qy+drz*qz,nx*nx+ny*ny+nz*nz];
}
function geometry(p:V3[],ids:Ids,kind:SelfContact['kind']){
  const [r,e,f]=vectors(p,ids,kind),n=cross(e,f),norm=length(n);
  const ee=dot(e,e),ff=dot(f,f),ef=dot(e,f),den=dot(n,n);
  // Degenerate triangles and almost parallel edges have no reliable normal.
  if(den<1e-24||den<1e-12*ee*ff)return;
  const re=dot(r,e),rf=dot(r,f);
  const u=kind==='vertex-face'?(re*ff-rf*ef)/den:(ef*rf-ff*re)/den;
  const v=(rf*ee-re*ef)/den;
  const weights=kind==='vertex-face'?[1,u+v-1,-u,-v]:[1-u,u,v-1,-v];
  const inside=u>=-PARAM_EPS&&v>=-PARAM_EPS&&(kind==='vertex-face'?u+v<=1+PARAM_EPS:u<=1+PARAM_EPS&&v<=1+PARAM_EPS);
  return {normal:scale(n,1/norm),gap:dot(r,n)/norm,weights,inside};
}

/** Frictionless, zero-clearance unilateral projection. Recomputing the normal
 * and projected weights gives a distance gradient with zero net force/torque.
 * Negative weights are intentional for a feature that moved outside its face
 * after impact; only swept detection can create such a constraint. */
export function solveSelfContact(p:V3[],invMass:number[],contact:SelfContact){
  // Scalar hot path: no temporary vectors in the iterative projection loop.
  const [i0,i1,i2,i3]=contact.ids,a=p[i0],b=p[i1],c=p[i2],d=p[i3],vf=contact.kind==='vertex-face';
  const origin=vf?b:c,edgeOrigin=vf?b:a,edgeEnd=vf?c:b;
  const rx=a[0]-origin[0],ry=a[1]-origin[1],rz=a[2]-origin[2];
  const ex=edgeEnd[0]-edgeOrigin[0],ey=edgeEnd[1]-edgeOrigin[1],ez=edgeEnd[2]-edgeOrigin[2];
  const fx=d[0]-origin[0],fy=d[1]-origin[1],fz=d[2]-origin[2];
  const nx=ey*fz-ez*fy,ny=ez*fx-ex*fz,nz=ex*fy-ey*fx,nn=nx*nx+ny*ny+nz*nz;
  const raw=rx*nx+ry*ny+rz*nz;if(raw*contact.sign>=0)return;
  const ee=ex*ex+ey*ey+ez*ez,ff=fx*fx+fy*fy+fz*fz,ef=ex*fx+ey*fy+ez*fz;
  if(nn<1e-24||nn<1e-12*ee*ff)return;
  const re=rx*ex+ry*ey+rz*ez,rf=rx*fx+ry*fy+rz*fz;
  const u=(vf?re*ff-rf*ef:ef*rf-ff*re)/nn,v=(rf*ee-re*ef)/nn;
  const c0=vf?1:1-u,c1=vf?u+v-1:u,c2=vf?-u:v-1,c3=-v;
  const den=invMass[i0]*c0*c0+invMass[i1]*c1*c1+invMass[i2]*c2*c2+invMass[i3]*c3*c3;if(den<=0)return;
  const impulse=-raw/(nn*den);
  for(let j=0;j<4;j++){
    const id=contact.ids[j],amount=invMass[id]*(j===0?c0:j===1?c1:j===2?c2:c3)*impulse;
    p[id][0]+=amount*nx;p[id][1]+=amount*ny;p[id][2]+=amount*nz;
  }
}

export class SelfContactSolver {
  private neighbors:Set<number>[];
  private edges:[number,number][]=[];
  private faceTree?:SweptTree; private edgeTree?:SweptTree;
  private previous=new Map<number,SelfContact>();
  private active=new Map<number,SelfContact>();
  stats={narrowTests:0,vertexFace:0,edgeEdge:0};
  constructor(private mesh:PaperMesh){
    this.neighbors=mesh.rest.map((_,i)=>new Set([i]));const keys=new Set<string>();
    for(const face of mesh.faces)for(let j=0;j<3;j++){
      const a=face[j],b=face[(j+1)%3];this.neighbors[a].add(b);this.neighbors[b].add(a);
      const key=`${Math.min(a,b)}:${Math.max(a,b)}`;if(!keys.has(key)){keys.add(key);this.edges.push([a,b]);}
    }
    const tree=(features:number[][])=>features.length?new SweptTree(features.map((_,i)=>i),features.map(f=>[0,1,2].map(d=>f.reduce((s,i)=>s+mesh.rest[i][d]/f.length,0)) as V3)):undefined;
    this.faceTree=tree(mesh.faces);this.edgeTree=tree(this.edges);
  }
  reset(){this.previous.clear();this.active.clear();this.stats={narrowTests:0,vertexFace:0,edgeEdge:0};}
  beginStep(){this.previous=this.active;this.active=new Map();this.stats={narrowTests:0,vertexFace:0,edgeEdge:0};}
  get contacts(){return [...this.active.values()];}
  detect(old:V3[],p:V3[]){
    // A linearly translating frame preserves every impact time and prevents a
    // fast rigid launch from inflating every swept box by the flight distance.
    const motion:V3=[0,0,0];
    for(let i=0;i<p.length;i++)for(let d=0;d<3;d++)motion[d]+=(p[i][d]-old[i][d])/p.length;
    const test=(ids:Ids,kind:SelfContact['kind'],key:number)=>{
      if(this.active.has(key))return;
      this.stats.narrowTests++;
      const coefficients=polynomial(old,p,ids,kind);
      // Bernstein convex hull: all four same-sign control values exclude roots.
      const [a,b,c,d,nn]=coefficients,b1=a+b/3,b2=a+2*b/3+c/3,b3=a+b+c+d;
      const sameSign=(a>0&&b1>0&&b2>0&&b3>0)||(a<0&&b1<0&&b2<0&&b3<0);
      const prior=this.previous.get(key);
      if(!prior&&sameSign)return;
      const touching=nn>1e-24&&a*a<=GAP_EPS*GAP_EPS*nn;
      if(!prior&&touching)return;
      let sign=0;
      if(prior&&touching&&geometry(old,ids,kind)?.inside)sign=prior.sign;
      else if(sameSign)return;
      if(!sign){
        for(const t of coplanarityRoots(coefficients.slice(0,4))){
          // Perfect initial coincidence has no stack order. Let it open; do not
          // create arbitrary opposition at a folded seam or a grazing touch.
          if(t<1e-8&&(nn<=1e-24||touching))continue;
          const hit=ids.map(i=>p[i].map((x,d)=>old[i][d]+t*(x-old[i][d])) as V3);
          const g=geometry(hit,[0,1,2,3],kind);if(!g?.inside)continue;
          const before=Math.max(0,t-1e-6),after=Math.min(1,t+1e-6),value=(s:number)=>((d*s+c)*s+b)*s+a;
          if(value(before)*value(after)>0)continue;
          sign=Math.sign(value(before));if(sign)break;
        }
      }
      if(!sign)return;
      const contact={ids,kind,sign,key};this.active.set(key,contact);
      if(kind==='vertex-face')this.stats.vertexFace++;else this.stats.edgeEdge++;
    };
    const faces=this.mesh.faces,faceBoxes=faces.map(f=>sweptBounds(f,old,p,motion));this.faceTree?.refit(faceBoxes);
    for(let v=0;v<p.length;v++)this.faceTree?.query(sweptBounds([v],old,p,motion),fi=>{
      const f=faces[fi];if(f.some(i=>this.neighbors[v].has(i)))return;
      test([v,...f],'vertex-face',v*faces.length+fi);
    });
    const edgeBoxes=this.edges.map(e=>sweptBounds(e,old,p,motion));this.edgeTree?.refit(edgeBoxes);
    this.edges.forEach(([a,b],i)=>this.edgeTree?.query(edgeBoxes[i],j=>{
      if(j<=i)return;const [c,d]=this.edges[j];
      if(this.neighbors[a].has(c)||this.neighbors[a].has(d)||this.neighbors[b].has(c)||this.neighbors[b].has(d))return;
      test([a,b,c,d],'edge-edge',p.length*faces.length+i*this.edges.length+j);
    }));
  }
  solve(p:V3[],invMass:number[],forward=true){
    const contacts=this.contacts;
    for(let j=0;j<contacts.length;j++)solveSelfContact(p,invMass,contacts[forward?j:contacts.length-1-j]);
  }
}
