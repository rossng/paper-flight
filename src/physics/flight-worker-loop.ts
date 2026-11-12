import { Simulation, FRAME_PHYSICS_BUDGET_MS } from './simulation';
import { FlightRecording } from '../rendering/flight-recording';
import { flightState, type FlightRequest, type FlightResponse } from './flight-messages';

/** Advances to a bounded requested model time, independently of the displayed
 * frame. Contact history stays here; integration is never rewound or replayed. */
export class FlightWorkerLoop {
  private simulation?:Simulation;
  private recording=new FlightRecording();
  private target=0;private paused=false;private queued=false;private failed=false;
  private sequence=0;private workMs=0;
  constructor(private send:(message:FlightResponse)=>void,private schedule:(callback:()=>void)=>void,private now=()=>performance.now()){}
  handle(message:FlightRequest){
    if(message.type==='launch'){
      if(this.simulation)throw Error('A flight worker owns one launch');
      this.simulation=new Simulation(message.parameters,message.mesh);this.simulation.launch();this.target=message.target;
      this.send({type:'ready'});
    }else if(message.type==='pause')this.paused=message.paused;
    else this.target=Math.max(this.target,message.time);
    this.requestTick();
  }
  private requestTick(){
    if(this.queued||this.paused||this.failed||!this.simulation?.active||this.simulation.time>=this.target-1e-12)return;
    this.queued=true;
    this.schedule(()=>{this.queued=false;try{this.tick();}catch(error){this.failed=true;this.send({type:'error',message:String(error)});}});
  }
  private emit(){
    const frames=this.recording.frames;this.recording.frames=[];
    this.send(flightState(this.simulation!,frames,++this.sequence,this.workMs));
  }
  private tick(){
    const simulation=this.simulation;if(!simulation||this.paused)return;
    const deadline=this.now()+FRAME_PHYSICS_BUDGET_MS;
    const shouldYield=()=>this.now()>=deadline||!simulation.active||simulation.time>=this.target-1e-12;
    while(simulation.active&&simulation.time<this.target-1e-12){
      const time=simulation.time,start=this.now(),complete=simulation.stepUntil(shouldYield);
      this.workMs+=this.now()-start;
      if(complete)this.recording.capture(simulation);
      // Keep every 120 Hz frame. Partial poses keep slow outer steps visible;
      // the receiver may coalesce unfinished poses but never recording frames.
      if(simulation.time!==time||!simulation.active)this.emit();
      if(shouldYield())break;
    }
    this.requestTick();
  }
}
