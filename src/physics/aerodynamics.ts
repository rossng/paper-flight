import type { LiftingSurface } from './lifting-surface';
import { dot, length, scale, type V3 } from './math';
import { stagedWeather, type Weather } from './weather';
import type { Face } from '../geometry/dart';
import { kernel, F64, I32 } from './solver';
export interface PanelLoad { center: V3; force: V3; lift: V3; alpha: number; area: number; stalled: boolean }
/** `wind` is uniform; `air` replaces it with the weather's gusts, turbulence and air feature at `time`. */
export interface AeroSettings { density: number; wind: V3; enabled: boolean; model?:'coupled'|'panels'; surface?:LiftingSurface; areaWeights?:number[]; air?:{weather:Weather;time:number} }
/** Quasi-steady low-aspect-ratio polar with a smooth, unvalidated separated-flow blend. */
export function coefficients(alpha: number) {
  return {cl:kernel.call.polar(alpha,0),cd:kernel.call.polar(alpha,1),stalled:kernel.call.polar(alpha,2)===1};
}
/** Twelve values per panel, as the solver and flight messages pack them. */
export function unpackPanels(raw:ArrayLike<number>):PanelLoad[] {
  return Array.from({length:raw.length/12},(_,i)=>{const o=12*i;
    return {center:[raw[o],raw[o+1],raw[o+2]],force:[raw[o+3],raw[o+4],raw[o+5]],lift:[raw[o+6],raw[o+7],raw[o+8]],alpha:raw[o+9],area:raw[o+10],stalled:raw[o+11]===1};});
}
// One solver-side lifting surface per surface object, rebuilt if it is used with other faces.
const surfaces=new WeakMap<LiftingSurface,{faces:Int32Array;aero:number;key:object}>();
function solverSurface(surface:LiftingSurface,faces:Face[]){
  const flat=Int32Array.from(faces.flat()),held=surfaces.get(surface);
  if(held&&held.faces.length===flat.length&&held.faces.every((v,i)=>v===flat[i]))return held.aero;
  if(held)kernel.call.aero_free(held.aero);
  kernel.i32(I32.faces,flat);kernel.i32(I32.faceOwners,surface.owners??[]);kernel.f64(F64.faceSigns,surface.signs);kernel.f64(F64.boundaries,surface.boundaries);
  kernel.i32(I32.patchFace,surface.patches.map(q=>q.face));kernel.i32(I32.patchStation,surface.patches.map(q=>q.station));kernel.i32(I32.patchPanel,surface.patches.map(q=>q.panel??q.station));
  kernel.f64(F64.patchWeights,surface.patches.flatMap(q=>q.weights));kernel.f64(F64.patchFraction,surface.patches.map(q=>q.fraction));
  kernel.i32(I32.landmarks,[surface.nose,surface.tail,surface.left,surface.right]);
  kernel.f64(F64.a,[surface.shieldingDistance,surface.span,surface.chordPanels??1,surface.owners?1:0]);
  const aero=kernel.call.aero_create(),key={};
  surfaces.set(surface,{faces:flat,aero,key});
  return aero;
}
/** Air loads of one state, from CURRENT vertices, areas, normals and velocities: the
 * coupled lifting surface, or with no surface (or `model: 'panels'`) each triangle as a
 * flat plate at its own incidence. Adds each vertex's share to `out`. */
export function aerodynamicLoads(positions: V3[], velocities: V3[], faces: Face[], settings: AeroSettings, out: V3[]): PanelLoad[] {
  const coupled=settings.surface&&settings.model!=='panels',aero=coupled?solverSurface(settings.surface!,faces):0;
  const weather=settings.air?.weather,wind=weather?.wind??settings.wind;
  kernel.vectors(F64.positions,positions);kernel.vectors(F64.velocities,velocities);kernel.vectors(F64.x,out);
  kernel.f64(F64.a,stagedWeather({wind,windHeading:weather?.windHeading,gusts:weather?.gusts??0,turbulence:weather?.turbulence??0,windMode:weather?.windMode}));
  const time=settings.air?.time??0,enabled=settings.enabled?1:0;
  if(coupled)kernel.call.aero_loads(aero,time,enabled,settings.density);
  else{
    kernel.i32(I32.faces,faces.flat());kernel.f64(F64.areaWeights,settings.areaWeights??[]);
    kernel.call.panel_loads(time,enabled,settings.density);
  }
  kernel.readVectors(F64.x,out);
  return unpackPanels(kernel.read64(F64.b));
}

/** Signed lift centroid projected along net lift. This is an estimate for
 * nonparallel panel loads, not a unique 3D centre of pressure or aerodynamic centre. */
export function centerOfLift(panels:PanelLoad[]):V3|undefined {
  const net:V3=[0,0,0];let gross=0;
  for(const panel of panels){gross+=length(panel.lift);for(let d=0;d<3;d++)net[d]+=panel.lift[d];}
  const magnitude=length(net);
  // Avoid an unstable location when opposing loads nearly cancel.
  if(magnitude<Math.max(1e-8,gross*0.001))return;
  const direction=scale(net,1/magnitude),center:V3=[0,0,0];
  for(const panel of panels){const weight=dot(panel.lift,direction)/magnitude;for(let d=0;d<3;d++)center[d]+=panel.center[d]*weight;}
  return center;
}
