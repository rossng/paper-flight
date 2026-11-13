import { FixedClock, FRAME_PHYSICS_BUDGET_MS, isActiveStatus, type Simulation, type Parameters } from './simulation';
import { FlightRecording } from '../rendering/flight-recording';
import { applyFlightState, type FlightState, type FlightResponse } from './flight-messages';

/** Seconds the worker may compute ahead of the display. Lead built while a
 * phase runs faster than real time carries playback through slower ones
 * (launch stroke, impact) at an even pace. */
export const FLIGHT_LOOKAHEAD=1;
export const PREPARE_DELAY_MS=180;
export type FlightWorkerPort=Pick<Worker,'postMessage'|'onmessage'|'onmessageerror'|'onerror'|'terminate'>;
export const createFlightWorker=():FlightWorkerPort=>new Worker(new URL('./flight-worker.ts',import.meta.url),{type:'module'});
/** One computation and a separate presentation clock. Only the worker integrates
 * after startup; displayed poses and detached replay frames are copies. */
export class FlightRunner {
  private worker?:FlightWorkerPort;
  private disposed=false;private paused=false;private suspended=false;private active:boolean;
  private sequence=0;private states:FlightState[]=[];private clock=new FixedClock();
  private presentationTime=0;private target=FLIGHT_LOOKAHEAD;private trailIndex=1;
  computedTime=0;workMs=0;updates=0;error?:string;
  constructor(private simulation:Simulation,private recording:FlightRecording,createWorker=createFlightWorker,
    options:{precompute?:boolean;parameters?:Parameters}={}){
    this.active=!options.precompute;
    try{
      const worker=createWorker();this.worker=worker;
      worker.onmessage=event=>this.accept(event.data as FlightResponse);
      worker.onerror=event=>{event.preventDefault();this.fail(event.message||'Flight worker failed');};
      worker.onmessageerror=()=>this.fail('Flight worker state could not be decoded');
      worker.postMessage({type:'launch',parameters:options.parameters??simulation.parameters,mesh:simulation.mesh,target:this.target});
    }catch(error){this.fail(String(error));}
  }
  get workerActive(){return !!this.worker;}
  get canActivate(){return !this.disposed&&!this.error&&(!!this.worker||this.states.length>0);}
  private get frozen(){return this.paused||this.suspended;}
  activate(simulation:Simulation,recording:FlightRecording){
    if(!this.canActivate||this.active)throw Error('Prepared flight is unavailable');
    this.simulation=simulation;this.recording=recording;this.active=true;
    for(const state of this.states)this.record(state);
  }
  private record(state:FlightState){
    for(const frame of state.frames)if(frame.time>(this.recording.frames.at(-1)?.time??-Infinity))this.recording.frames.push(frame);
  }
  accept(message:FlightResponse){
    if(this.disposed||this.error)return;
    if(message.type==='error'){this.fail(message.message);return;}
    if(message.type!=='state'||message.sequence<=this.sequence)return;
    this.sequence=message.sequence;this.workMs=message.workMs;this.updates++;this.computedTime=message.time;
    if(this.active)this.record(message);
    // Retain complete outer states, but only the latest unfinished pose. Storage
    // is bounded by outer steps, even for the 96-substep adaptive ceiling.
    if(this.states.at(-1)?.frames.length===0)this.states.pop();
    this.states.push(message);
    if(!isActiveStatus(message.status)){this.worker?.terminate();this.worker=undefined;}
  }
  private fail(message:string){
    if(this.disposed)return;
    this.worker?.terminate();this.worker=undefined;this.error=message;
    // Startup may fall back to the untouched initial-value problem. A display
    // mirror lacks contact history: never integrate it after receiving state.
    if(this.active&&this.sequence>0){this.simulation.status='error';this.states=[];}
    this.clock.reset();
  }
  advance(elapsed:number){
    if(this.disposed||!this.active||this.frozen||!this.simulation.active)return;
    if(this.worker||this.sequence>0){
      this.presentationTime=Math.min(this.computedTime,this.presentationTime+Math.max(0,Math.min(.1,elapsed)));
      let latest:FlightState|undefined;
      while(this.states.length&&this.states[0].time<=this.presentationTime+1e-12)latest=this.states.shift();
      if(latest){try{applyFlightState(this.simulation,latest);}catch(error){this.fail(String(error));return;}}
      while(this.trailIndex<this.recording.frames.length&&this.recording.frames[this.trailIndex].time<=this.simulation.time+1e-12){
        this.simulation.trail.push([...this.recording.frames[this.trailIndex++].center]);
      }
      const target=this.presentationTime+FLIGHT_LOOKAHEAD;
      if(target>this.target+1e-6){this.target=target;this.worker?.postMessage({type:'target',time:target});}
    }else{
      const deadline=performance.now()+FRAME_PHYSICS_BUDGET_MS,shouldYield=()=>performance.now()>=deadline||!this.simulation.active;
      this.clock.advance(elapsed,()=>{const complete=this.simulation.stepUntil(shouldYield);if(complete)this.recording.capture(this.simulation);return complete;},shouldYield);
    }
  }
  setPaused(paused:boolean){this.paused=paused;this.updatePause();}
  setSuspended(suspended:boolean){this.suspended=suspended;this.updatePause();}
  private updatePause(){if(!this.disposed){this.clock.reset();this.worker?.postMessage({type:'pause',paused:this.frozen});}}
  dispose(){this.disposed=true;this.worker?.terminate();this.worker=undefined;this.states=[];}
}

/** Debounced exact-parameter speculation. A matching release adopts the worker;
 * moving/changing the launch cancels it. There is never a second forecast worker. */
export class FlightPreparation {
  private key?:string;private timer?:ReturnType<typeof setTimeout>;private runner?:FlightRunner;
  constructor(private createWorker=createFlightWorker){}
  prepare(key:string,simulation:Simulation,parameters:Parameters){
    if(key===this.key)return;
    this.clear();this.key=key;
    const captured=structuredClone(parameters);
    this.timer=setTimeout(()=>{
      this.timer=undefined;
      this.runner=new FlightRunner(simulation,new FlightRecording(),this.createWorker,{precompute:true,parameters:captured});
    },PREPARE_DELAY_MS);
  }
  take(key:string){
    const runner=key===this.key&&this.runner?.canActivate?this.runner:undefined;
    if(runner)this.runner=undefined;
    this.clear();return runner;
  }
  clear(){if(this.timer!==undefined)clearTimeout(this.timer);this.timer=undefined;this.runner?.dispose();this.runner=undefined;this.key=undefined;}
}
