import test from 'node:test';
import assert from 'node:assert/strict';
import { Simulation, DEFAULTS, FixedClock, FIXED_DT } from '../src/physics/simulation';
import { materialSettings } from '../src/physics/materials';
import { designMesh } from '../src/folding/design';
import { FlightRecording } from '../src/rendering/flight-recording';
import { membraneAlphas, solveMembrane } from '../src/physics/membrane';

for(const material of ['paper','tissue'] as const)for(const design of ['legacy','dart'])test(`yielded ${material}/${design} preserves every outer state and recording`,()=>{
 const parameters={...DEFAULTS,...materialSettings(material),speed:11,wind:[0,0,3] as [number,number,number],gusts:1.5,turbulence:1};
 const mesh=design==='legacy'?undefined:designMesh(design,parameters);
 const synchronous=new Simulation(parameters,mesh),yielded=new Simulation(parameters,mesh);
 const a=new FlightRecording(),b=new FlightRecording();synchronous.launch();yielded.launch();
 for(let i=0;i<12;i++){
  synchronous.step();a.capture(synchronous);
  let calls=0;
  while(!yielded.stepUntil(()=>true)){
   assert.ok(++calls<96,'bounded internal substeps');
   // Rendering/UI queries during a suspended outer step must be read-only.
   yielded.metrics();yielded.deformation();
  }
  b.capture(yielded);
  assert.deepEqual(yielded.positions,synchronous.positions);
  assert.deepEqual(yielded.velocities,synchronous.velocities);
  assert.deepEqual(yielded.forces,synchronous.forces);
  assert.deepEqual(yielded.metrics(),synchronous.metrics());
  assert.deepEqual(yielded.contactCandidates,synchronous.contactCandidates);
  assert.deepEqual(yielded.mesh.hinges,synchronous.mesh.hinges);
  assert.equal(yielded.peakDeformation,synchronous.peakDeformation);
 }
 assert.deepEqual(a.frames,b.frames);
});

test('reset discards suspended work, and synchronous step can finish it',()=>{
 const a=new Simulation({material:'tissue'}),b=new Simulation({material:'tissue'});a.launch();b.launch();
 assert.equal(a.stepUntil(()=>true),false);a.step();b.step();assert.deepEqual(a.positions,b.positions);
 assert.equal(a.stepUntil(()=>true),false);a.reset();assert.equal(a.time,0);assert.equal(a.status,'ready');
 a.launch();b.launch();a.step();b.step();assert.deepEqual(a.positions,b.positions);assert.deepEqual(a.trail,b.trail);
});

test('partial steps keep clock credit, bounded debt is reported, pause resumes exactly',()=>{
 const clock=new FixedClock();let done=false;
 assert.equal(clock.advance(FIXED_DT,()=>done),0);assert.equal(clock.accumulator,FIXED_DT);
 done=true;assert.equal(clock.advance(0,()=>done),1);assert.equal(clock.accumulator,0);
 for(let i=0;i<100;i++)clock.advance(.1,()=>false);
 assert.ok(Math.abs(clock.accumulator-.1)<1e-12);assert.ok(Math.abs(clock.droppedTime-9.9)<1e-10);
 let calls=0;assert.equal(clock.advance(0,()=>{calls++;},()=>true),1);assert.equal(calls,1);
 clock.reset();assert.equal(clock.accumulator,0);assert.equal(clock.droppedTime,0);
 const a=new Simulation(),b=new Simulation();a.launch();b.launch();
 clock.advance(FIXED_DT,()=>a.stepUntil(()=>true));const time=a.time;clock.reset();assert.equal(a.time,time);
 while(!a.stepUntil(()=>true)){}b.step();assert.deepEqual(a.positions,b.positions);
});

test('yielding still completes landing and records its final state exactly',()=>{
 const a=new Simulation({height:.2,speed:0}),b=new Simulation({height:.2,speed:0});a.launch();b.launch();
 const recording=new FlightRecording();let calls=0;
 while(a.status==='flying'){if(a.stepUntil(()=>true))recording.capture(a);assert.ok(++calls<500);}
 while(b.status==='flying')b.step();
 assert.equal(a.status,'landed');assert.deepEqual(a.positions,b.positions);assert.equal(recording.duration,b.time);
 assert.equal(a.stepUntil(()=>true),true);
});

test('prepared membrane coefficients retain live material and timestep changes',()=>{
 const sim=new Simulation();
 for(const dt of [1/360,1/1080])for(const scale of [1,.25]){
  const material={...sim.material,youngMD:sim.material.youngMD*scale,thickness:sim.material.thickness*scale};
  const a=structuredClone(sim.positions),b=structuredClone(a),ta=structuredClone(sim.membranes[0]),tb=structuredClone(ta);
  a[ta.face[1]][1]+=.001;b[tb.face[1]][1]+=.001;
  solveMembrane(a,sim.invMass,ta,material,dt);
  solveMembrane(b,sim.invMass,tb,material,dt,membraneAlphas(tb,material,dt));
  assert.deepEqual(a,b);assert.deepEqual(ta.lambda,tb.lambda);
 }
});
