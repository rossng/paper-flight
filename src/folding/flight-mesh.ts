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
  const split=new Map<string,[number,number]>();for(const f of faces){let best:[number,number]=[f[0],f[1]];for(let k=1;k<3;k++)if(length(sub(rest[f[k]],rest[f[(k+1)%3]]))>length(sub(rest[best[0]],rest[best[1]])))best=[f[k],f[(k+1)%3]];if(length(sub(rest[best[0]],rest[best[1]]))>maxEdge)split.set([...best].sort((a,b)=>a-b).join(':'),best);}
  if(!split.size)break;
  for(const [a,b] of split.values()){const mid=rest.length;materialUV.push([(materialUV[a][0]+materialUV[b][0])/2,(materialUV[a][1]+materialUV[b][1])/2]);rest.push(scale(add(rest[a],rest[b]),.5));const next:Face[]=[],nextOwners:number[]=[];faces.forEach((f,i)=>{let found=false;for(let k=0;k<3;k++)if((f[k]===a&&f[(k+1)%3]===b)||(f[k]===b&&f[(k+1)%3]===a)){next.push([f[k],mid,f[(k+2)%3]],[mid,f[(k+1)%3],f[(k+2)%3]]);nextOwners.push(owners[i],owners[i]);found=true;break;}if(!found){next.push(f);nextOwners.push(owners[i]);}});faces=next;owners=nextOwners;}
 }
 const areas=faces.map(([a,b,c])=>length(cross(sub(rest[b],rest[a]),sub(rest[c],rest[a])))/2),masses=rest.map(()=>0);
 faces.forEach((f,i)=>f.forEach(v=>masses[v]+=areas[i]*state.recipe.gsm/1000/3));
 if(masses.some(m=>m<=0))throw Error('Unreferenced flight vertex');
 const originalPanels=state.panels.map(panel=>panel.points.map(p=>[-p.position[1]*size,p.position[0]*size,p.position[2]*size] as V3));
 const covers=(p:V3,polygon:V3[])=>{for(let i=1;i<polygon.length-1;i++)if(barycentric(p,polygon[0],polygon[i],polygon[i+1]))return true;return false;};
 const layerCounts:number[]=[],aeroWeights:number[]=[];
 // Deterministic area sampling: each coincident layer receives its share of aerodynamic area.
 // Static reference for the panel/bonded baseline; default flight recomputes exposure on aerodynamic patches.
 faces.forEach((f,fi)=>{let exposure=0,layers=0;for(let s=0;s<48;s++){const u=Math.sqrt((s+.5)/48),v=((s+.5)*0.61803398875)%1,w=[1-u,u*(1-v),u*v];const p:V3=[0,0,0];f.forEach((id,j)=>{for(let d=0;d<3;d++)p[d]+=rest[id][d]*w[j];});let n=0;for(const polygon of originalPanels)if(covers(p,polygon))n++;n=Math.max(1,n);exposure+=1/n;layers+=n;}aeroWeights[fi]=exposure/48;layerCounts[fi]=layers/48;});
 const vertexOwners=rest.map(()=>new Set<number>());faces.forEach((f,i)=>f.forEach(v=>vertexOwners[v].add(owners[i])));
 const layerTies:LayerTie[]=[];
 rest.forEach((p,vertex)=>{let best=-1,weights:V3|undefined;faces.forEach((f,i)=>{if(f.includes(vertex)||vertexOwners[vertex].has(owners[i])||areas[i]<(best<0?0:areas[best]))return;const bary=barycentric(p,...f.map(v=>rest[v]) as [V3,V3,V3]);if(bary){best=i;weights=bary;}});if(best>=0)layerTies.push({vertex,face:faces[best],weights:weights!});});
 const edges:PaperMesh['edges']=[],hinges:Hinge[]=[],adjacency=new Map<string,{a:number;b:number;op:number;fi:number}>();
 faces.forEach((f,fi)=>{for(let k=0;k<3;k++){const a=f[k],b=f[(k+1)%3],op=f[(k+2)%3],key=[a,b].sort((a,b)=>a-b).join(':'),old=adjacency.get(key);if(old){const ids:Hinge['ids']=[old.op,op,old.a,old.b];const angle=hingeAngle(...ids.map(i=>rest[i]) as [V3,V3,V3,V3]);hinges.push({ids,rest:angle,crease:owners[fi]!==owners[old.fi]&&Math.abs(angle)>1e-5,weight:length(sub(rest[a],rest[b]))**2/(areas[fi]+areas[old.fi]),lambda:0,layers:Math.max(1,(layerCounts[fi]+layerCounts[old.fi])/2),layerStiffness:Math.max(1,(layerCounts[fi]+layerCounts[old.fi])/2)**(2*coupling)});}else{adjacency.set(key,{a,b,op,fi});edges.push({a,b,rest:length(sub(rest[a],rest[b])),lambda:0});}}});
 const nose=rest.reduce((best,p,i)=>p[0]>rest[best][0]?i:best,0),tail=rest.reduce((best,p,i)=>Math.abs(p[2])<1e-6&&(Math.abs(rest[best][2])>=1e-6||p[0]<rest[best][0])?i:best,0),leftRoot=rest.reduce((best,p,i)=>p[2]<rest[best][2]?i:best,0),rightRoot=rest.reduce((best,p,i)=>p[2]>rest[best][2]?i:best,0);
 if(rest[rightRoot][2]-rest[leftRoot][2]<.01*size)throw Error('Open both wings before flight');
 return {rest,faces,edges,hinges,masses,materialUV,faceOwners:owners,regions:rest.map(p=>Math.abs(p[2])<1e-6?0:Math.sign(p[2])),nose,tail,leftRoot,rightRoot,aeroWeights,layerTies,layerCounts,sourceDesign:state.recipe.id};
}
