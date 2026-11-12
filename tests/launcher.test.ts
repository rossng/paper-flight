import test from 'node:test';
import assert from 'node:assert/strict';
import {elasticPull,pullTravel,launchSpeed,springRemaining,springSpeed,springVelocityDelta,RELEASE_DURATION_MS} from '../src/physics/launcher';
import {Simulation,FIXED_DT} from '../src/physics/simulation';
import {length,sub,type V3} from '../src/physics/math';
import {materialSettings} from '../src/physics/materials';

test('equal extra hand travel produces progressively less spring extension',()=>{
 const steps=[0,30,60,90,120,150].map(y=>elasticPull(0,y).y);
 for(let i=2;i<steps.length;i++){assert.ok(steps[i]>steps[i-1]);assert.ok(steps[i]-steps[i-1]<steps[i-1]-steps[i-2]);}
 assert.ok(elasticPull(0,100).y<70);assert.equal(elasticPull(0,10000).y,100);
});
test('elastic drag preserves direction, stays bounded, and can be re-grabbed continuously',()=>{
 for(const [x,y] of [[0,0],[25,50],[-70,40],[0,100]]){
  const travel=pullTravel(x,y),pull=elasticPull(travel.x,travel.y);
  assert.ok(Math.abs(pull.x-x)<1e-10);assert.ok(Math.abs(pull.y-y)<1e-10);
 }
 const pull=elasticPull(-100,100);assert.ok(Math.abs(pull.x/pull.y+1)<1e-10);assert.ok(Math.hypot(pull.x,pull.y)<=100);
 assert.deepEqual(elasticPull(0,-50),{x:0,y:0});
});
test('pull controls a bounded speed range and spring relaxation accelerates',()=>{
 assert.equal(launchSpeed(-10),0);assert.equal(launchSpeed(0),0);
 assert.equal(launchSpeed(50),5.5);assert.equal(launchSpeed(100),11);assert.equal(launchSpeed(200),11);
 assert.equal(springRemaining(0),1);assert.ok(springRemaining(1)<1e-12);
 assert.ok(1-springRemaining(.2)<springRemaining(.8)-springRemaining(1));
});

test('elastic acceleration tapers to zero and its impulse is independent of timestep',()=>{
 const duration=RELEASE_DURATION_MS/1000,speed=11;
 assert.equal(springSpeed(0,speed),0);assert.equal(springSpeed(duration,speed),speed);
 const impulses=Array.from({length:10},(_,i)=>springVelocityDelta(i*duration/10,duration/10,speed));
 for(let i=1;i<impulses.length;i++)assert.ok(impulses[i]<impulses[i-1]);
 for(const dt of [1/120,1/360,.019,.2]){
  let impulse=0;for(let time=0;time<duration+.1;time+=dt)impulse+=springVelocityDelta(time,dt,speed);
  assert.ok(Math.abs(impulse-speed)<1e-12);
 }
 assert.equal(springVelocityDelta(duration,.1,speed),0);
 assert.equal(springVelocityDelta(0,.1,speed,0),0);
 assert.equal(springVelocityDelta(0,.1,0),0);
});

test('simulation accelerates from rest along the aim, then coasts without a second impulse',()=>{
 for(const substeps of [3,9]){
  const sim=new Simulation({aero:false,gravity:0,height:10,speed:11,heading:37,elevation:16,substeps});sim.launch();
  assert.ok(sim.velocities.every(v=>length(v)===0));
  const yaw=37*Math.PI/180,elevation=16*Math.PI/180,direction:V3=[Math.cos(elevation)*Math.cos(yaw),Math.sin(elevation),Math.cos(elevation)*Math.sin(yaw)];
  for(let i=0;i<30;i++){
   sim.step();const expected=direction.map(d=>d*11*Math.sin(Math.min(1,sim.time/.125)*Math.PI/2)) as V3;
   assert.ok(length(sub(sim.center(sim.velocities),expected))<1e-8);
   assert.ok(sim.deformation().rms<1e-8,'supported acceleration must not strain the sheet');
  }
  const before=sim.center();sim.step();
  assert.ok(length(sub(sub(sim.center(),before),direction.map(d=>11*FIXED_DT*d) as V3))<1e-9);
  sim.launch();assert.equal(sim.time,0);assert.ok(sim.velocities.every(v=>length(v)===0));sim.step();
  assert.ok(Math.abs(length(sim.center(sim.velocities))-11*Math.sin(FIXED_DT/.125*Math.PI/2))<1e-8);
 }
});

test('equal pull retains nominal exit speed across masses, with gravity still acting during launch',()=>{
 for(const material of ['paper','tissue','steel'] as const){
  const sim=new Simulation({...materialSettings(material),aero:false,height:10,speed:6});sim.launch();
  for(let i=0;i<15;i++)sim.step();
  assert.ok(length(sub(sim.center(sim.velocities),[6,-9.81*.125,0]))<1e-8);
 }
 const instant=new Simulation({launchDuration:0,speed:6});instant.launch();assert.ok(Math.abs(length(instant.center(instant.velocities))-6)<1e-10);
});

test('elastic launch builds air loads gradually and does not overwrite aerodynamic losses',()=>{
 const elastic=new Simulation({speed:11,pitch:15,gravity:0}),instant=new Simulation({speed:11,pitch:15,gravity:0,launchDuration:0});
 elastic.launch();instant.launch();elastic.step();instant.step();
 assert.ok(length(elastic.metrics().force)<length(instant.metrics().force)*.1,'first-step aerodynamic shock is reduced');
 while(elastic.time<.125-1e-12)elastic.step();
 assert.ok(elastic.center(elastic.velocities)[0]<10.99,'drag reduces actual exit speed');
});

test('adaptive stepping resolves the spring stroke even when initial airspeed is zero',()=>{
 const sim=new Simulation({aero:false,gravity:0});sim.launch();
 assert.equal(sim.stepUntil(()=>true),false);
 assert.ok(Math.abs(sim.time-FIXED_DT/6)<1e-12);
 while(sim.time<.125-1e-12)sim.step();
 // Finish the current outer step before checking the free-flight substep.
 sim.step();const time=sim.time;sim.stepUntil(()=>true);
 assert.ok(Math.abs(sim.time-time-FIXED_DT/3)<1e-12);
});
