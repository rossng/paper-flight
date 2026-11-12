import type { PaperMesh } from '../geometry/dart';
import type { Simulation, Parameters, Status } from './simulation';
import type { V3 } from './math';
import type { FlightFrame } from '../rendering/flight-recording';

export type FlightRequest = {type:'launch';parameters:Parameters;mesh:PaperMesh;target:number}
  | {type:'pause';paused:boolean} | {type:'target';time:number};
export interface FlightState {
  type:'state';sequence:number;time:number;status:Status;pathLength:number;peakDeformation:number;lastCenter:V3;
  positions:Float64Array;velocities:Float64Array;forces:Float64Array;panels:Float64Array;plastic:Float64Array;
  contacts:{narrowTests:number;vertexFace:number;edgeEdge:number};
  frames:FlightFrame[];workMs:number;
}
export type FlightResponse = FlightState | {type:'ready'} | {type:'error';message:string};
const pack=(vectors:V3[])=>{
  const out=new Float64Array(vectors.length*3);
  for(let i=0;i<vectors.length;i++)for(let d=0;d<3;d++)out[3*i+d]=vectors[i][d];
  return out;
};
/** Live poses retain doubles; detached 120 Hz recordings retain their original
 * Float32 format. Transfer owns these copies, never the solver's working state. */
export function flightState(sim:Simulation,frames:FlightFrame[],sequence:number,workMs:number):FlightState {
  const panels=new Float64Array(sim.panels.length*12),plastic=new Float64Array(sim.mesh.hinges.length*2);
  sim.panels.forEach((p,i)=>{
    const base=12*i;for(let d=0;d<3;d++){panels[base+d]=p.center[d];panels[base+3+d]=p.force[d];panels[base+6+d]=p.lift[d];}
    panels[base+9]=p.alpha;panels[base+10]=p.area;panels[base+11]=Number(p.stalled);
  });
  sim.mesh.hinges.forEach((h,i)=>{plastic[2*i]=h.plasticOffset??0;plastic[2*i+1]=h.plasticAccumulated??0;});
  return {type:'state',sequence,time:sim.time,status:sim.status,pathLength:sim.pathLength,peakDeformation:sim.peakDeformation,lastCenter:[...sim.lastCenter],
    positions:pack(sim.positions),velocities:pack(sim.velocities),forces:pack(sim.forces),panels,plastic,contacts:{...sim.selfContact.stats},frames,workMs};
}
export function flightTransfers(state:FlightState):ArrayBuffer[]{
  return [state.positions.buffer,state.velocities.buffer,state.forces.buffer,state.panels.buffer,state.plastic.buffer,
    ...state.frames.flatMap(frame=>[frame.positions.buffer,frame.velocities.buffer])] as ArrayBuffer[];
}
/** Update the renderer's mirror. Contact/membrane history stays worker-owned;
 * this mirror must never resume integration after accepting a worker state. */
export function applyFlightState(sim:Simulation,state:FlightState){
  const size=sim.positions.length*3;
  if(state.positions.length!==size||state.velocities.length!==size||state.forces.length!==size||state.panels.length%12||state.plastic.length!==sim.mesh.hinges.length*2)throw Error('Invalid flight state dimensions');
  for(let i=0;i<sim.positions.length;i++)for(let d=0;d<3;d++){
    sim.positions[i][d]=state.positions[3*i+d];sim.velocities[i][d]=state.velocities[3*i+d];sim.forces[i][d]=state.forces[3*i+d];
  }
  sim.panels.length=state.panels.length/12;
  for(let i=0;i<sim.panels.length;i++){
    const p=sim.panels[i]??={center:[0,0,0],force:[0,0,0],lift:[0,0,0],area:0,alpha:0,stalled:false},base=12*i;
    for(let d=0;d<3;d++){p.center[d]=state.panels[base+d];p.force[d]=state.panels[base+3+d];p.lift[d]=state.panels[base+6+d];}
    p.alpha=state.panels[base+9];p.area=state.panels[base+10];p.stalled=state.panels[base+11]===1;
  }
  sim.mesh.hinges.forEach((h,i)=>{h.plasticOffset=state.plastic[2*i];h.plasticAccumulated=state.plastic[2*i+1];});
  sim.time=state.time;sim.status=state.status;sim.pathLength=state.pathLength;sim.peakDeformation=state.peakDeformation;
  sim.lastCenter=[...state.lastCenter];sim.selfContact.stats={...state.contacts};
}
