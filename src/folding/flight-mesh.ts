import {topology,inspect,type FoldState} from './model';
import {hingeAngle,type PaperMesh,type Face,type Hinge} from '../geometry/dart';
import {add,sub,scale,cross,dot,length,unit,type V3} from '../physics/math';
export interface LayerTie { vertex:number; face:Face; weights:V3 }
export interface CompileOptions { size?:number; layerCoupling?:number; maxEdge?:number }
/** Barycentric coordinates on a coplanar triangle, including its boundary. */
export function barycentric(p:V3,a:V3,b:V3,c:V3):V3|undefined {
 const ab=sub(b,a),ac=sub(c,a),ap=sub(p,a),n=cross(ab,ac),nl=length(n);if(nl<1e-12||Math.abs(dot(ap,n))/nl>1e-7)return;
 const aa=dot(ab,ab),bb=dot(ac,ac),cc=dot(ab,ac),den=aa*bb-cc*cc;if(Math.abs(den)<1e-20)return;
 const u=(dot(ap,ab)*bb-dot(ap,ac)*cc)/den,v=(dot(ap,ac)*aa-dot(ap,ab)*cc)/den;
 return u>=-1e-7&&v>=-1e-7&&u+v<=1+1e-7?[1-u-v,u,v]:undefined;
}
/** Compile the material mesh and grain coordinates; retain laminate metadata for comparison mode. */
export function compileFlightMesh(state:FoldState,options:CompileOptions={}):PaperMesh {
 const report=inspect(state);if(report.seamGap>1e-7||report.maxStrain>1e-7||!report.manifold)throw Error('Cannot fly a torn or stretched sheet');
 const size=options.size??1,coupling=options.layerCoupling??0.5,maxEdge=(options.maxEdge??0.055)*size,t=topology(state);
 if(![size,coupling,maxEdge].every(Number.isFinite)||size<=0||maxEdge<=0||coupling<0||coupling>1)throw Error('Invalid laminate compilation parameters');
 const materialUV=t.vertices.map(p=>[p.uv[0]*size,p.uv[1]*size] as [number,number]);
 const rest:V3[]=t.vertices.map(p=>[-p.position[1]*size,p.position[0]*size,p.position[2]*size]);
 let faces:Face[]=[],owners:number[]=[];
 // Centroid fans retain conformed boundary vertices, including collinear crease intersections.
 for(const [owner,polygon] of t.faces.entries()){
  const center:V3=[0,0,0];for(const i of polygon)for(let d=0;d<3;d++)center[d]+=rest[i][d]/polygon.length;
  const ci=rest.length;materialUV.push([0,1].map(d=>polygon.reduce((sum,i)=>sum+materialUV[i][d]/polygon.length,0)) as [number,number]);rest.push(center);for(let j=0;j<polygon.length;j++){const face:Face=[polygon[j],polygon[(j+1)%polygon.length],ci];if(length(cross(sub(rest[face[1]],rest[face[0]]),sub(rest[ci],rest[face[0]])))>1e-12){faces.push(face);owners.push(owner);}}
 }
 // Shared-edge bisection, so refinement never creates a material T-junction.
 for(let pass=0;pass<8;pass++){
  // Decide splits geometrically, invariant under the z mirror. Coincident
  // layers and mirror-image panels then refine identically; deciding per
  // vertex-index edge let the two keel layers (one per wing) diverge, and an
  // asymmetric mesh seeds a one-sided roll in a symmetric fold.
  const point=(i:number,sign:number)=>`${Math.round(rest[i][0]*1e8)},${Math.round(rest[i][1]*1e8)},${Math.round(sign*rest[i][2]*1e8)}`;
  const edgeKey=(a:number,b:number)=>[1,-1].map(sign=>[point(a,sign),point(b,sign)].sort().join('|')).sort()[0];
  const chosen=new Set<string>();for(const f of faces){const lengths=[0,1,2].map(k=>length(sub(rest[f[k]],rest[f[(k+1)%3]]))),longest=Math.max(...lengths);if(longest<=maxEdge)continue;for(let k=0;k<3;k++)if(lengths[k]>=longest*(1-1e-9))chosen.add(edgeKey(f[k],f[(k+1)%3]));}
  const split=new Map<string,[number,number]>();for(const f of faces)for(let k=0;k<3;k++){const edge:[number,number]=[f[k],f[(k+1)%3]];if(chosen.has(edgeKey(...edge)))split.set([...edge].sort((a,b)=>a-b).join(':'),edge);}
  if(!split.size)break;
  // A face receiving two splits is subdivided in split order; order by length
  // and the mirror-invariant key so mirror-image faces get the same diagonal.
  const ordered=[...split.values()].map(edge=>({edge,length:length(sub(rest[edge[0]],rest[edge[1]])),key:edgeKey(...edge)})).sort((p,q)=>q.length-p.length>1e-12?1:p.length-q.length>1e-12?-1:p.key<q.key?-1:p.key>q.key?1:0);
  for(const {edge:[a,b]} of ordered){const mid=rest.length;materialUV.push([(materialUV[a][0]+materialUV[b][0])/2,(materialUV[a][1]+materialUV[b][1])/2]);rest.push(scale(add(rest[a],rest[b]),.5));const next:Face[]=[],nextOwners:number[]=[];faces.forEach((f,i)=>{let found=false;for(let k=0;k<3;k++)if((f[k]===a&&f[(k+1)%3]===b)||(f[k]===b&&f[(k+1)%3]===a)){next.push([f[k],mid,f[(k+2)%3]],[mid,f[(k+1)%3],f[(k+2)%3]]);nextOwners.push(owners[i],owners[i]);found=true;break;}if(!found){next.push(f);nextOwners.push(owners[i]);}});faces=next;owners=nextOwners;}
 }
 const areas=faces.map(([a,b,c])=>length(cross(sub(rest[b],rest[a]),sub(rest[c],rest[a])))/2),masses=rest.map(()=>0);
 faces.forEach((f,i)=>f.forEach(v=>masses[v]+=areas[i]*state.recipe.gsm/1000/3));
 if(masses.some(m=>m<=0))throw Error('Unreferenced flight vertex');
 const originalPanels=state.panels.map(panel=>panel.points.map(p=>[-p.position[1]*size,p.position[0]*size,p.position[2]*size] as V3));
 const covers=(p:V3,polygon:V3[])=>{for(let i=1;i<polygon.length-1;i++)if(barycentric(p,polygon[0],polygon[i],polygon[i+1]))return true;return false;};
 const layerCounts:number[]=[],aeroWeights:number[]=[];
 // Deterministic area sampling: each coincident layer receives its share of aerodynamic area.
 // Static reference for the panel/bonded baseline; default flight recomputes exposure on aerodynamic patches.
 // Eight base points under all six vertex permutations, so a face's estimate
 // does not depend on its vertex order (mirror-image faces are wound oppositely).
 const permutations=[[0,1,2],[0,2,1],[1,0,2],[1,2,0],[2,0,1],[2,1,0]];
 const samples=Array.from({length:8},(_,s)=>{const u=Math.sqrt((s+.5)/8),v=((s+.5)*0.61803398875)%1;return [1-u,u*(1-v),u*v];}).flatMap(w=>permutations.map(order=>order.map(k=>w[k])));
 faces.forEach((f,fi)=>{let exposure=0,layers=0;for(const w of samples){const p:V3=[0,0,0];f.forEach((id,j)=>{for(let d=0;d<3;d++)p[d]+=rest[id][d]*w[j];});let n=0;for(const polygon of originalPanels)if(covers(p,polygon))n++;n=Math.max(1,n);exposure+=1/n;layers+=n;}aeroWeights[fi]=exposure/48;layerCounts[fi]=layers/48;});
 const vertexOwners=rest.map(()=>new Set<number>());faces.forEach((f,i)=>f.forEach(v=>vertexOwners[v].add(owners[i])));
 const layerTies:LayerTie[]=[];
 // Largest covering face; equal areas (a vertex on a shared edge) are settled
 // by a mirror-invariant centroid key, so mirror-image vertices tie to
 // mirror-image faces rather than to whichever face is listed last.
 // Coincident layers share a centroid, so material coordinates (u signed by
 // the vertex's own side) settle those.
 const tieKey=(f:Face,vertex:number)=>{const c=[0,1,2].map(d=>(rest[f[0]][d]+rest[f[1]][d]+rest[f[2]][d])/3),uv=[0,1].map(d=>(materialUV[f[0]][d]+materialUV[f[1]][d]+materialUV[f[2]][d])/3);return [c[0],c[1],Math.abs(c[2]),Math.sign(materialUV[vertex][0])*uv[0],uv[1]].map(v=>Math.round(v*1e9));};
 // -1: face i is preferred over face j; 0: an exact (mirror) tie.
 const compare=(i:number,j:number,vertex:number)=>{if(areas[i]>areas[j]*(1+1e-9))return -1;if(areas[i]<areas[j]*(1-1e-9))return 1;const a=tieKey(faces[i],vertex),b=tieKey(faces[j],vertex);for(let d=0;d<a.length;d++)if(a[d]!==b[d])return a[d]<b[d]?-1:1;return 0;};
 // A centre-line vertex can face a mirror pair of coincident faces; it ties
 // to both rather than favouring one wing's layer.
 rest.forEach((p,vertex)=>{let best:{face:number;weights:V3}[]=[];faces.forEach((f,i)=>{if(f.includes(vertex)||vertexOwners[vertex].has(owners[i]))return;const order=best.length?compare(i,best[0].face,vertex):-1;if(order>0)return;const bary=barycentric(p,...f.map(v=>rest[v]) as [V3,V3,V3]);if(!bary)return;if(order<0)best=[];best.push({face:i,weights:bary});});for(const tie of best)layerTies.push({vertex,face:faces[tie.face],weights:tie.weights});});
 const edges:PaperMesh['edges']=[],hinges:Hinge[]=[],adjacency=new Map<string,{a:number;b:number;op:number;fi:number}>();
 faces.forEach((f,fi)=>{for(let k=0;k<3;k++){const a=f[k],b=f[(k+1)%3],op=f[(k+2)%3],key=[a,b].sort((a,b)=>a-b).join(':'),old=adjacency.get(key);if(old){const ids:Hinge['ids']=[old.op,op,old.a,old.b];const angle=hingeAngle(...ids.map(i=>rest[i]) as [V3,V3,V3,V3]);hinges.push({ids,rest:angle,crease:owners[fi]!==owners[old.fi]&&Math.abs(angle)>1e-5,weight:length(sub(rest[a],rest[b]))**2/(areas[fi]+areas[old.fi]),lambda:0,layerStiffness:Math.max(1,(layerCounts[fi]+layerCounts[old.fi])/2)**(2*coupling)});}else{adjacency.set(key,{a,b,op,fi});edges.push({a,b,rest:length(sub(rest[a],rest[b])),lambda:0});}}});
 const nose=rest.reduce((best,p,i)=>p[0]>rest[best][0]?i:best,0),tail=rest.reduce((best,p,i)=>Math.abs(p[2])<1e-6&&(Math.abs(rest[best][2])>=1e-6||p[0]<rest[best][0])?i:best,0),leftRoot=rest.reduce((best,p,i)=>p[2]<rest[best][2]?i:best,0),rightRoot=rest.reduce((best,p,i)=>p[2]>rest[best][2]?i:best,0);
 if(rest[rightRoot][2]-rest[leftRoot][2]<.01*size)throw Error('Open both wings before flight');
 return {rest,faces,edges,hinges,masses,materialUV,faceOwners:owners,regions:rest.map(p=>Math.abs(p[2])<1e-6?0:Math.sign(p[2])),nose,tail,leftRoot,rightRoot,aeroWeights,layerTies,layerCounts,sourceDesign:state.recipe.id};
}
