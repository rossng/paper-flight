import test from 'node:test';
import assert from 'node:assert/strict';
import { Simulation, DEFAULTS, FIXED_DT } from '../src/physics/simulation';
import { materialSettings } from '../src/physics/materials';
import { designMesh } from '../src/folding/design';
import { FlightRecording } from '../src/rendering/flight-recording';
import { FlightWorkerLoop } from '../src/physics/flight-worker-loop';
import { FlightRunner, FlightPreparation, PREPARE_DELAY_MS, FLIGHT_LOOKAHEAD, type FlightWorkerPort } from '../src/physics/flight-runner';
import { flightTransfers, type FlightRequest, type FlightResponse, type FlightState } from '../src/physics/flight-messages';

class LocalWorker {
 onmessage:FlightWorkerPort['onmessage']=null;
 onmessageerror:FlightWorkerPort['onmessageerror']=null;
 onerror:FlightWorkerPort['onerror']=null;
 tasks:(()=>void)[]=[];messages:FlightResponse[]=[];requests:FlightRequest[]=[];terminated=false;clock=0;
 loop=new FlightWorkerLoop(message=>{
  const copy=structuredClone(message,{transfer:message.type==='state'?flightTransfers(message):[]});
  this.messages.push(copy);this.deliver(copy);
 },callback=>this.tasks.push(callback),()=>this.clock+=3);
 deliver(message:FlightResponse){this.onmessage?.call(this as unknown as Worker,{data:message} as MessageEvent);}
 postMessage(message:FlightRequest){this.requests.push(structuredClone(message));this.loop.handle(structuredClone(message));}
 terminate(){this.terminated=true;this.tasks=[];}
 drain(){let ticks=0;while(this.tasks.length&&!this.terminated){assert.ok(++ticks<10000);this.tasks.shift()!();}}
 port(){return this as unknown as FlightWorkerPort;}
}
function launched(parameters={}){const simulation=new Simulation(parameters),recording=new FlightRecording();simulation.launch();recording.capture(simulation);return {simulation,recording};}

for(const material of ['paper','tissue'] as const)test(`worker transfer and yielded computation preserve ${material} contact flight and every replay sample`,()=>{
 const parameters={...DEFAULTS,...materialSettings(material),speed:11,wind:[0,0,3] as [number,number,number],gusts:1.5,turbulence:1};
 const mesh=designMesh('dart',parameters),sim=new Simulation(parameters,mesh),recording=new FlightRecording();sim.launch();
 const worker=new LocalWorker();worker.postMessage({type:'launch',parameters,mesh:sim.mesh,target:FIXED_DT*4});worker.drain();
 assert.equal(worker.tasks.length,0,'idle at requested horizon');
 const frames=worker.messages.filter((m):m is FlightState=>m.type==='state').flatMap(s=>s.frames);
 assert.equal(frames.length,4);
 for(let i=0;i<4;i++){sim.step();recording.capture(sim);assert.deepEqual(frames[i],recording.frames[i]);}
 const final=worker.messages.at(-1) as FlightState;
 assert.deepEqual(Array.from(final.positions),sim.positions.flat());assert.deepEqual(Array.from(final.velocities),sim.velocities.flat());
 assert.deepEqual(Array.from(final.forces),sim.forces.flat());assert.deepEqual(final.contacts,sim.selfContact.stats);assert.equal(final.peakDeformation,sim.peakDeformation);
 // Continue after transferred buffers have been detached in the sender.
 worker.postMessage({type:'pause',paused:true});worker.postMessage({type:'target',time:FIXED_DT*5});worker.drain();assert.equal(worker.messages.at(-1),final);
 worker.postMessage({type:'pause',paused:false});worker.drain();sim.step();recording.capture(sim);
 const resumed=worker.messages.at(-1) as FlightState;assert.deepEqual(resumed.frames.at(-1),recording.frames.at(-1));
 assert.deepEqual(Array.from(resumed.positions),sim.positions.flat());
});

test('a prepared prefix stays invisible until release, then follows display time and pauses without revealing future poses',()=>{
 const preview=new Simulation(),worker=new LocalWorker();
 const runner=new FlightRunner(preview,new FlightRecording(),()=>worker.port(),{precompute:true});worker.drain();
 assert.equal(preview.status,'ready');assert.equal(preview.time,0);assert.ok(runner.computedTime>=FLIGHT_LOOKAHEAD-1e-12);
 assert.equal(worker.tasks.length,0);assert.equal(worker.requests.length,1);
 const {simulation,recording}=launched();runner.activate(simulation,recording);
 assert.equal(simulation.time,0);assert.equal(simulation.trail.length,1);assert.ok(recording.duration>=FLIGHT_LOOKAHEAD-1e-12);
 runner.advance(1/60);assert.ok(simulation.time<=1/60+1e-12);assert.ok(simulation.time>0);
 const paused=structuredClone(simulation.positions),time=simulation.time;runner.setPaused(true);worker.drain();runner.advance(100);
 assert.equal(simulation.time,time);assert.deepEqual(simulation.positions,paused);
 runner.setPaused(false);runner.setSuspended(true);runner.advance(100);assert.equal(simulation.time,time);
 runner.setSuspended(false);runner.advance(1/60);assert.ok(simulation.time>time);assert.ok(simulation.time<.04);
 assert.equal(simulation.trail.length,recording.frames.filter(f=>f.time<=simulation.time+1e-12).length);
 const before=recording.frames.length;worker.deliver(worker.messages.at(-1)!);assert.equal(recording.frames.length,before,'duplicate packet ignored');
 runner.dispose();worker.deliver({...worker.messages.find((m):m is FlightState=>m.type==='state')!,sequence:100000});assert.equal(recording.frames.length,before,'stale reply ignored');
});

test('forecast landing does not land the displayed plane early and preserves the final detached replay',()=>{
 const parameters={height:.2,speed:0},preview=new Simulation(parameters),worker=new LocalWorker();
 const runner=new FlightRunner(preview,new FlightRecording(),()=>worker.port(),{precompute:true});worker.drain();
 assert.equal(worker.terminated,true);assert.equal(runner.canActivate,true);assert.equal(preview.status,'ready');
 const {simulation,recording}=launched(parameters);runner.activate(simulation,recording);runner.advance(.01);assert.equal(simulation.status,'flying');
 while(simulation.status==='flying')runner.advance(.01);
 const expected=launched(parameters);while(expected.simulation.status==='flying'){expected.simulation.step();expected.recording.capture(expected.simulation);}
 assert.equal(simulation.status,'landed');assert.deepEqual(simulation.positions,expected.simulation.positions);assert.deepEqual(recording.frames,expected.recording.frames);
});

test('startup failure falls back, but failure after accepting worker state interrupts instead of integrating a mirror',()=>{
 const a=launched(),fallback=new FlightRunner(a.simulation,a.recording,()=>{throw Error('Workers unavailable');});fallback.advance(FIXED_DT);
 assert.ok(a.simulation.time>0);assert.equal(a.simulation.status,'flying');
 const b=launched(),worker=new LocalWorker(),runner=new FlightRunner(b.simulation,b.recording,()=>worker.port());worker.drain();
 worker.deliver({type:'error',message:'Failed after computation'});runner.advance(.1);assert.equal(b.simulation.status,'error');assert.equal(b.simulation.time,0);
 const updates=runner.updates;worker.deliver({...worker.messages.find((m):m is FlightState=>m.type==='state')!,sequence:100000});assert.equal(runner.updates,updates,'queued state after worker failure ignored');
});

test('preparation debounces, cancels mismatches, and reuses exactly one matching worker',t=>{
 t.mock.timers.enable({apis:['setTimeout']});
 const workers:LocalWorker[]=[],preparation=new FlightPreparation(()=>{const w=new LocalWorker();workers.push(w);return w.port();});
 const sim=new Simulation();preparation.prepare('pull-a',sim,sim.parameters);t.mock.timers.tick(PREPARE_DELAY_MS-1);assert.equal(workers.length,0);
 preparation.prepare('pull-b',sim,sim.parameters);t.mock.timers.tick(PREPARE_DELAY_MS);assert.equal(workers.length,1);
 preparation.prepare('pull-b',sim,sim.parameters);t.mock.timers.tick(PREPARE_DELAY_MS);assert.equal(workers.length,1);
 assert.equal(preparation.take('changed-height'),undefined);assert.equal(workers[0].terminated,true);
 preparation.prepare('pull-c',sim,sim.parameters);t.mock.timers.tick(PREPARE_DELAY_MS);const runner=preparation.take('pull-c');assert.ok(runner);assert.equal(workers[1].terminated,false);
 preparation.clear();assert.equal(workers[1].terminated,false,'adopted worker survives clearing preparation');runner.dispose();assert.equal(workers[1].terminated,true);
 preparation.prepare('canceled',sim,sim.parameters);preparation.clear();t.mock.timers.tick(PREPARE_DELAY_MS);assert.equal(workers.length,2);
});
