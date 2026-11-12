import type { V3 } from '../physics/math';
import type { Simulation } from '../physics/simulation';
export interface FlightFrame { time:number; positions:Float32Array; velocities:Float32Array; center:V3 }
/** Detached samples: reviewing a flight never rewinds or mutates its solver. */
export class FlightRecording {
 frames:FlightFrame[]=[];
 capture(simulation:Simulation){
  if(this.frames.at(-1)?.time===simulation.time)return;
  this.frames.push({time:simulation.time,positions:new Float32Array(simulation.positions.flat()),velocities:new Float32Array(simulation.velocities.flat()),center:[...simulation.center()]});
 }
 get duration(){return this.frames.at(-1)?.time??0;}
 sample(time:number):FlightFrame|undefined {
  if(!this.frames.length)return;
  const t=Math.max(0,Math.min(this.duration,time));
  let lo=0,hi=this.frames.length-1;
  while(lo<hi){const mid=Math.floor((lo+hi)/2);if(this.frames[mid].time<t)lo=mid+1;else hi=mid;}
  const b=this.frames[lo],a=this.frames[Math.max(0,lo-1)];
  const f=b.time===a.time?0:(t-a.time)/(b.time-a.time);
  return {time:t,positions:a.positions.map((value,i)=>value+(b.positions[i]-value)*f),velocities:a.velocities.map((value,i)=>value+(b.velocities[i]-value)*f),center:a.center.map((value,i)=>value+(b.center[i]-value)*f) as V3};
 }
}
