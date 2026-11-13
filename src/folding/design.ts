import {findDesign} from './library';
import {build,type FoldOperation,type FoldRecipe,type FoldState,type Panel} from './model';
import {compileFlightMesh} from './flight-mesh';
import {add,cross,dot,scale,sub,unit,type V3} from '../physics/math';
import type {Parameters} from '../physics/simulation';
/** Only validated wing-root adjustments are exposed by the shared registry. */
export function supportsWingSpan(id:string){return findDesign(id)?.adjustableSpan??false;}
/** Chord of the elevator flap at the wingtip, in sheet metres (before size scaling). */
export const ELEVATOR_DEPTH=.03;
function deployedRecipe(id:string,parameters:Parameters):FoldRecipe {
 const original=findDesign(id)?.recipe;if(!original)throw Error('Unknown fold design');const recipe=structuredClone(original);recipe.gsm=parameters.paperWeight;
 for(const op of recipe.operations)if(op.id==='left-wing'||op.id==='right-wing'){
  const delta=(1-(supportsWingSpan(id)?parameters.span:1))*recipe.width/4;
  if(op.axis[0][0]>1e-8)op.axis[0][0]+=delta;op.axis[1][0]+=delta;
  const slope=(op.axis[1][0]-op.axis[0][0])/(op.axis[1][1]-op.axis[0][1]);op.movingNormal=[1,-slope,0];op.angleDegrees=(op.id==='left-wing'?1:-1)*(90-parameters.dihedral);
 }
 return recipe;
}
/** Map a material point into its folded position through a panel that contains it. */
function locate(panels:Panel[],uv:[number,number]):{position:V3;panel:Panel}|undefined {
 for(const panel of panels){
  const [p0,p1,p2]=panel.points;
  for(let k=1;k+1<panel.points.length;k++){
   const a=p0.uv,b=panel.points[k].uv,c=panel.points[k+1].uv;
   const det=(b[0]-a[0])*(c[1]-a[1])-(c[0]-a[0])*(b[1]-a[1]);if(Math.abs(det)<1e-14)continue;
   const u=((uv[0]-a[0])*(c[1]-a[1])-(c[0]-a[0])*(uv[1]-a[1]))/det,v=((b[0]-a[0])*(uv[1]-a[1])-(uv[0]-a[0])*(b[1]-a[1]))/det;
   if(u<-1e-9||v<-1e-9||u+v>1+1e-9)continue;
   const q=panel.points[k].position,r=panel.points[k+1].position;
   return {panel,position:add(p0.position,add(scale(sub(q,p0.position),u),scale(sub(r,p0.position),v)))};
  }
  void p1;void p2;
 }
}
/** Trailing-edge up-elevator on each wing: a straight crease from the wing-root
 * trailing-edge corner to the outer wing edge ELEVATOR_DEPTH ahead of the trailing
 * edge. Starting the crease at the root corner keeps the keel attached, so this is a
 * valid rigid fold of every wing layer under the flap. Positive angles raise the
 * trailing edge. Returns undefined when a design's wing cannot take this flap. */
export function elevatorOperations(state:FoldState,degrees:number):FoldOperation[]|undefined {
 const ops:FoldOperation[]=[];
 for(const side of ['left','right'] as const){
  const wing=state.recipe.operations.find(op=>op.id===`${side}-wing`);if(!wing)return;
  const panels=state.panels.filter(p=>p.tags.includes(wing.id));if(!panels.length)return;
  // Stacks can carry layers from the other half; use the side whose wing holds all three points.
  const root=wing.axis[1][0],trailing=wing.axis[1][1],outer=state.recipe.width/2;
  const points=([-1,1] as const).map(sign=>[locate(panels,[sign*root,trailing]),locate(panels,[sign*outer,trailing-ELEVATOR_DEPTH]),locate(panels,[sign*outer,trailing])]).find(p=>p.every(Boolean));
  if(!points)return;
  const [a,b,c]=points as {position:V3;panel:Panel}[];
  const [p0,p1,p2]=b.panel.points.map(p=>p.position),normal=unit(cross(sub(p1,p0),sub(p2,p0))),axis=unit(sub(b.position,a.position));
  let moving=unit(cross(normal,axis));if(dot(moving,sub(c.position,a.position))<0)moving=scale(moving,-1);
  // Positive angle raises the trailing edge toward the folded +x (flight up) side.
  const angle=degrees*Math.PI/180,v=sub(c.position,a.position);
  const raised=add(add(scale(v,Math.cos(angle)),scale(cross(axis,v),Math.sin(angle))),scale(axis,dot(axis,v)*(1-Math.cos(angle))));
  const direction=Math.sign(degrees)*(raised[0]-v[0])>=0?1:-1;
  ops.push({id:`${side}-elevator`,label:`Bend up the ${side} trailing edge`,axis:[a.position,b.position],movingNormal:moving,angleDegrees:direction*degrees,layers:{includeTag:wing.id}});
 }
 return ops;
}
const elevatorSupport=new Map<string,boolean>();
/** Validated per design: the flap must be a strain-free fold of the deployed wing. */
export function supportsElevator(id:string){
 if(id==='legacy')return true;
 if(!elevatorSupport.has(id)){
  let ok=false;
  try{const recipe=deployedRecipe(id,{dihedral:8,span:1,paperWeight:80} as Parameters),ops=elevatorOperations(build(recipe),10);if(ops){build({...recipe,operations:[...recipe.operations,...ops]});ok=true;}}catch{ok=false;}
  elevatorSupport.set(id,ok);
 }
 return elevatorSupport.get(id)!;
}
export function flightRecipe(id:string,parameters:Parameters):FoldRecipe {
 const recipe=deployedRecipe(id,parameters);
 if(parameters.elevator&&supportsElevator(id)){const ops=elevatorOperations(build(recipe),parameters.elevator);if(ops)recipe.operations.push(...ops);}
 return recipe;
}
export function designMesh(id:string,parameters:Parameters){return compileFlightMesh(build(flightRecipe(id,parameters)),{size:parameters.size});}
