import {add,sub,scale,dot,cross,length,unit,type V3} from '../physics/math';
export type UV=[number,number];
export interface PaperPoint { uv:UV; position:V3 }
export interface Panel { id:string; points:PaperPoint[]; tags:string[] }
export interface FoldOperation {
 id:string; label:string; axis:[V3,V3]; movingNormal:V3; angleDegrees:number;
 layers?:{ includeTag?:string; excludeTag?:string };
}
export interface FoldRecipe { id:string; name:string; source:string; width:number; height:number; gsm:number; operations:FoldOperation[] }
export interface FoldState { panels:Panel[]; applied:FoldOperation[]; recipe:FoldRecipe }
const EPS=1e-9;
export function materialArea(points:UV[]){return Math.abs(points.reduce((sum,p,i)=>{const q=points[(i+1)%points.length];return sum+p[0]*q[1]-q[0]*p[1];},0))/2;}
export function sheet(recipe:FoldRecipe):FoldState {const w=recipe.width/2,h=recipe.height;return {recipe,applied:[],panels:[{id:'sheet',tags:[],points:([[-w,0],[w,0],[w,h],[-w,h]] as UV[]).map(uv=>({uv,position:[...uv,0]}))}]};}
function interpolate(a:PaperPoint,b:PaperPoint,t:number):PaperPoint{return {uv:[a.uv[0]+(b.uv[0]-a.uv[0])*t,a.uv[1]+(b.uv[1]-a.uv[1])*t],position:add(a.position,scale(sub(b.position,a.position),t))};}
function clip(points:PaperPoint[],distance:(p:PaperPoint)=>number,sign:number){const result:PaperPoint[]=[];for(let i=0;i<points.length;i++){const a=points[i],b=points[(i+1)%points.length],da=distance(a)*sign,db=distance(b)*sign;if(da>=-EPS)result.push(a);if((da>EPS&&db<-EPS)||(da<-EPS&&db>EPS))result.push(interpolate(a,b,da/(da-db)));}return result.filter((p,i)=>{const q=result[(i+result.length-1)%result.length];return !q||Math.hypot(p.uv[0]-q.uv[0],p.uv[1]-q.uv[1])>EPS;});}
export function fold(state:FoldState,operation:FoldOperation):FoldState {
 if(![...operation.axis.flat(),...operation.movingNormal,operation.angleDegrees].every(Number.isFinite))throw Error('Fold parameters must be finite');
 const axis=unit(sub(operation.axis[1],operation.axis[0])),normal=unit(operation.movingNormal),angle=operation.angleDegrees*Math.PI/180;
 if(length(sub(operation.axis[1],operation.axis[0]))<EPS||length(operation.movingNormal)<EPS||Math.abs(dot(axis,normal))>1e-6)throw Error(`Invalid fold axis: ${operation.id}`);
 const panels:Panel[]=[];
 for(const panel of state.panels){
  const selected=(!operation.layers?.includeTag||panel.tags.includes(operation.layers.includeTag))&&(!operation.layers?.excludeTag||!panel.tags.includes(operation.layers.excludeTag));
  if(!selected){panels.push(panel);continue;}
  const distance=(p:PaperPoint)=>dot(sub(p.position,operation.axis[0]),normal);
  if(panel.points.every(p=>Math.abs(distance(p))<EPS)){panels.push(panel);continue;}
  for(const sign of [-1,1]){
   const points=clip(panel.points,distance,sign);if(points.length<3||materialArea(points.map(p=>p.uv))<1e-13)continue;
   const moved=sign===1;panels.push({id:`${panel.id}/${operation.id}${moved?'+':'-'}`,tags:moved?[...panel.tags,operation.id]:[...panel.tags],points:points.map(p=>{
    if(!moved)return {uv:[...p.uv],position:[...p.position]};const v=sub(p.position,operation.axis[0]);
    return {uv:[...p.uv],position:add(operation.axis[0],add(add(scale(v,Math.cos(angle)),scale(cross(axis,v),Math.sin(angle))),scale(axis,dot(axis,v)*(1-Math.cos(angle)))))};
   })});
  }
 }
 return {recipe:state.recipe,applied:[...state.applied,operation],panels};
}
export function build(recipe:FoldRecipe,count=recipe.operations.length){let state=sheet(recipe);for(const op of recipe.operations.slice(0,count))state=checkedFold(state,op);return state;}
/** Conform T-junctions in material space, never weld distinct layers by world position. */
export function topology(state:FoldState){
 const vertices:PaperPoint[]=[],keys=new Map<string,number>(),key=(p:UV)=>p.map(v=>Math.round(v*1e9)).join(':');
 for(const face of state.panels)for(const p of face.points)if(!keys.has(key(p.uv))){keys.set(key(p.uv),vertices.length);vertices.push(p);}
 const faces=state.panels.map(face=>{const result:number[]=[];for(let i=0;i<face.points.length;i++){
  const a=face.points[i].uv,b=face.points[(i+1)%face.points.length].uv,dx=b[0]-a[0],dy=b[1]-a[1],len2=dx*dx+dy*dy;
  const onEdge=vertices.map((p,id)=>({id,t:((p.uv[0]-a[0])*dx+(p.uv[1]-a[1])*dy)/len2,cross:Math.abs((p.uv[0]-a[0])*dy-(p.uv[1]-a[1])*dx)})).filter(p=>p.t>=-EPS&&p.t<1-EPS&&p.cross<1e-10).sort((a,b)=>a.t-b.t);result.push(...onEdge.map(p=>p.id));
 }return result;});
 const edges:number[][]=[],counts:number[]=[],edgeMap=new Map<string,number>();for(const face of faces)for(let i=0;i<face.length;i++){const a=face[i],b=face[(i+1)%face.length],k=[a,b].sort((a,b)=>a-b).join(':');if(edgeMap.has(k))counts[edgeMap.get(k)!]++;else{edgeMap.set(k,edges.length);edges.push([a,b]);counts.push(1);}}
 return {vertices,faces,edges,counts};
}
export function inspect(state:FoldState){
 let area=0,maxStrain=0,seamGap=0;const all=state.panels.flatMap(p=>p.points);
 for(const panel of state.panels){area+=materialArea(panel.points.map(p=>p.uv));for(const a of panel.points)for(const b of panel.points){const rest=Math.hypot(a.uv[0]-b.uv[0],a.uv[1]-b.uv[1]);if(rest>EPS)maxStrain=Math.max(maxStrain,Math.abs(length(sub(a.position,b.position))/rest-1));}
  // Also check vertices that split an adjacent panel's material edge.
  for(let i=0;i<panel.points.length;i++){const a=panel.points[i],b=panel.points[(i+1)%panel.points.length],dx=b.uv[0]-a.uv[0],dy=b.uv[1]-a.uv[1],len2=dx*dx+dy*dy;for(const p of all){const t=((p.uv[0]-a.uv[0])*dx+(p.uv[1]-a.uv[1])*dy)/len2;if(t>=-EPS&&t<=1+EPS&&Math.abs((p.uv[0]-a.uv[0])*dy-(p.uv[1]-a.uv[1])*dx)<1e-10)seamGap=Math.max(seamGap,length(sub(p.position,interpolate(a,b,t).position)));}}
 }
 const mesh=topology(state);return {panels:state.panels.length,area,mass:area*state.recipe.gsm/1000,maxStrain,seamGap,manifold:mesh.counts.every(n=>n<=2),euler:mesh.vertices.length-mesh.edges.length+mesh.faces.length};
}
/** Gate for future editor commands: invalid operations leave the previous state intact. */
export function checkedFold(state:FoldState,operation:FoldOperation){const next=fold(state,operation),report=inspect(next);if(report.maxStrain>1e-7||report.seamGap>1e-7||!report.manifold||report.euler!==1||Math.abs(report.area-state.recipe.width*state.recipe.height)>1e-10)throw Error(`Fold ${operation.id} would stretch or disconnect the sheet`);return next;}
export function exportFold(state:FoldState){const t=topology(state),shared={edges_vertices:t.edges,edges_assignment:t.counts.map(n=>n===1?'B':'U'),faces_vertices:t.faces,frame_unit:'m'};return {file_spec:1.2,file_creator:'Paper Flight representation study',file_title:state.recipe.name,file_classes:['singleModel'],frame_classes:['creasePattern'],vertices_coords:t.vertices.map(p=>p.uv),...shared,'paperflight:recipe':state.recipe,'paperflight:appliedSteps':state.applied.length,'paperflight:validationScope':'Zero-thickness kinematics only; no collision, layer-order or aerodynamic certification.',file_frames:[{frame_classes:['foldedForm'],vertices_coords:t.vertices.map(p=>p.position),...shared}]};}
