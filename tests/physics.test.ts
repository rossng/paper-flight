import test from 'node:test';
import assert from 'node:assert/strict';
import { createDart, hingeAngle } from '../src/geometry/dart';
import { hingeGradients } from '../src/physics/constraints';
import { aerodynamicLoads, coefficients, centerOfLift, type PanelLoad } from '../src/physics/aerodynamics';
import { Simulation, FixedClock } from '../src/physics/simulation';
import { dot, length, sub, type V3 } from '../src/physics/math';

test('dart topology is non-degenerate, mass is 5 g and creases are distinct',()=>{
 const m=createDart(); assert.ok(m.faces.length>100);assert.ok(Math.abs(m.masses.reduce((a,b)=>a+b,0)-0.005)<1e-12);
 assert.ok(m.edges.every(e=>e.rest>1e-6));assert.ok(m.hinges.some(h=>h.crease));assert.ok(m.hinges.some(h=>!h.crease));
 const counts=new Map<string,number>();for(const f of m.faces)for(let k=0;k<3;k++){const key=[f[k],f[(k+1)%3]].sort((a,b)=>a-b).join(':');counts.set(key,(counts.get(key)??0)+1);}assert.ok([...counts.values()].every(n=>n<=2));
});
test('signed hinge gradients agree with finite differences, including flat paper',()=>{
 for(const z of [0,0.04,-0.08]){
 const p:V3[]=[[0,0.13,0],[0.02,-0.1,z],[0,0,0],[0.2,0,0]];
 const g=hingeGradients(...p as [V3,V3,V3,V3])!;
 for(let i=0;i<4;i++)for(let d=0;d<3;d++){
 p[i][d]+=1e-7;const a=hingeAngle(...p as [V3,V3,V3,V3]);p[i][d]-=2e-7;const b=hingeAngle(...p as [V3,V3,V3,V3]);p[i][d]+=1e-7;
 assert.ok(Math.abs((a-b)/2e-7-g[i][d])<1e-5,`gradient ${i},${d}`);
 }
 }
});
test('aerodynamics is dissipative, winding-invariant, zero at rest, and responds to deformation',()=>{
 const p:V3[]=[[0,0,0],[0.2,0,0],[0,0,0.2]],v:V3[]=[[6,-1,0],[6,-1,0],[6,-1,0]],faces:[number,number,number][]=[[0,1,2]];
 const settings={enabled:true,density:1.225,wind:[0,0,0] as V3};const zeros=()=>p.map(()=>[0,0,0] as V3);
 const a=aerodynamicLoads(p,v,faces,settings,zeros())[0];assert.ok(a.force[1]>0);assert.ok(dot(a.force,v[0])<0);
 assert.ok(Math.abs(dot(a.lift,v[0]))<1e-12,'lift must exclude drag');
 const b=aerodynamicLoads(p,v,[[0,2,1]],settings,zeros())[0];assert.ok(length(sub(a.force,b.force))<1e-12);
 assert.equal(length(aerodynamicLoads(p,zeros(),faces,settings,zeros())[0].force),0);
 p[2][1]=0.08;const c=aerodynamicLoads(p,v,faces,settings,zeros())[0];assert.ok(length(sub(a.force,c.force))>0.005);
 assert.ok(coefficients(1.5).cd>coefficients(0.1).cd);
});
test('free mesh conserves translation and follows gravity without aerodynamic support',()=>{
 const s=new Simulation({aero:false,gravity:0,height:100,launchDuration:0});s.launch();for(let i=0;i<60;i++)s.step();
 assert.ok(Math.abs(s.metrics().distance-3)<1e-6);assert.ok(s.metrics().deformation<1e-7);
 const fall=new Simulation({aero:false,height:100});fall.launch();for(let i=0;i<60;i++)fall.step();
 assert.ok(Math.abs(fall.metrics().altitude-(100-0.5*9.81*0.5**2))<0.01);
});
test('fixed step produces identical advancement across render frame rates',()=>{
 function count(fps:number){const clock=new FixedClock();let n=0;for(let i=0;i<fps;i++)clock.advance(1/fps,()=>n++);return n;}
 assert.equal(count(30),120);assert.equal(count(60),120);assert.equal(count(144),120);
});

test('coupled comparison: identical launches, less bending with stiffer paper, different forces and flight',()=>{
 const soft=new Simulation({stiffness:0.2}),stiff=new Simulation({stiffness:5});soft.launch();stiff.launch();
 assert.deepEqual(soft.positions,stiff.positions);assert.deepEqual(soft.velocities,stiff.velocities);
 for(let i=0;i<120;i++){soft.step();stiff.step();}
 const a=soft.metrics(),b=stiff.metrics();
 assert.ok(a.deformation>b.deformation*1.5,`matched-time deformation ${a.deformation} vs ${b.deformation}`);
 assert.ok(a.bend>b.bend*2);assert.ok(length(sub(a.force,b.force))>0.001);assert.ok(length(sub(a.center,b.center))>0.1);
 while(soft.status==='flying')soft.step();while(stiff.status==='flying')stiff.step();
 assert.equal(soft.status,'landed');assert.equal(stiff.status,'landed');
 assert.ok(soft.peakDeformation>stiff.peakDeformation*1.5);assert.ok(Math.abs(soft.metrics().distance-stiff.metrics().distance)>0.5);
});
test('repeatability and reset restore the identical initial-value problem',()=>{
 const s=new Simulation({stiffness:0.2});s.launch();for(let i=0;i<30;i++)s.step();const before=s.positions.map(p=>[...p]);
 s.launch();for(let i=0;i<30;i++)s.step();assert.deepEqual(s.positions,before);
 s.reset();assert.equal(s.time,0);assert.equal(s.status,'ready');assert.equal(s.metrics().speed,0);assert.ok(s.metrics().deformation<1e-12);
});
test('paper size and weight set physical mass without shifting release height',()=>{
 for(const size of [0.85,1,1.15])for(const paperWeight of [60,80,110]){
  const s=new Simulation({size,paperWeight,span:1.2});
  assert.ok(Math.abs(s.mesh.masses.reduce((a,b)=>a+b,0)-0.005*size**2*paperWeight/80)<1e-12);
  assert.ok(Math.abs(s.center()[1]-1.8)<1e-12);
  assert.ok(Math.abs(s.center()[0])<1e-12);
  assert.ok(s.mesh.edges.every(e=>e.rest>0));
 }
});
test('directional launches follow their chosen heading and elevation without aerodynamics',()=>{
 for(const heading of [-70,0,70]){
  const speed=7,elevation=16,s=new Simulation({heading,elevation,speed,gravity:0,aero:false,height:100,launchDuration:0});s.launch();
  for(let i=0;i<60;i++)s.step();
  const yaw=heading*Math.PI/180,e=elevation*Math.PI/180,c=s.center();
  assert.ok(Math.abs(c[0]-3.5*Math.cos(e)*Math.cos(yaw))<1e-6);
  assert.ok(Math.abs(c[2]-3.5*Math.cos(e)*Math.sin(yaw))<1e-6);
  assert.ok(Math.abs(c[1]-100-3.5*Math.sin(e))<1e-6);
 }
});

test('vertical, backward and inverted pitches set actual attitude and start the elastic launch at rest',()=>{
 for(const pitch of [-180,-135,-90,90,135,180]){
  const s=new Simulation({pitch,speed:6});s.launch();
  const original=sub(s.mesh.rest[s.mesh.nose],s.mesh.rest[s.mesh.tail]),actual=sub(s.positions[s.mesh.nose],s.positions[s.mesh.tail]);
  const a=pitch*Math.PI/180,expected:V3=[Math.cos(a)*original[0]-Math.sin(a)*original[1],Math.sin(a)*original[0]+Math.cos(a)*original[1],original[2]];
  assert.ok(length(sub(actual,expected))<1e-12);
  assert.deepEqual(s.velocities[0],[0,0,0]);
 }
});
test('flight recording preserves actual solver states and interpolates without mutating them',async()=>{
 const {FlightRecording}=await import('../src/rendering/flight-recording');
 const s=new Simulation();s.launch();const recording=new FlightRecording();recording.capture(s);
 const initial=s.positions[0][0];s.step();recording.capture(s);const end=recording.duration;
 assert.equal(recording.frames[0].positions[0],Math.fround(initial));
 const before=s.positions.map(p=>[...p]),velocityBefore=s.velocities.map(v=>[...v]);const frame=recording.sample(end/2)!;
 assert.equal(recording.frames[0].velocities[0],0);
 assert.ok(Math.abs(frame.velocities[1]-(recording.frames[0].velocities[1]+recording.frames[1].velocities[1])/2)<1e-6);
 assert.ok(Math.abs(frame.center[0]-(recording.frames[0].center[0]+recording.frames[1].center[0])/2)<1e-10);
 frame.positions[0]=999;assert.deepEqual(s.positions,before);assert.notEqual(recording.frames[0].positions[0],999);
 frame.velocities[0]=999;assert.deepEqual(s.velocities,velocityBefore);assert.notEqual(recording.frames[0].velocities[0],999);
 assert.equal(recording.sample(-1)!.time,0);assert.equal(recording.sample(99)!.time,end);
 recording.capture(s);assert.equal(recording.frames.length,2);
});

test('lift centroid retains signed opposing loads and hides at zero or cancellation',()=>{
 const panel=(x:number,lift:number):PanelLoad=>({center:[x,0,0],lift:[0,lift,0],force:[-100,lift,0],area:1,alpha:0,stalled:false});
 assert.deepEqual(centerOfLift([panel(0,1),panel(2,3)]),[1.5,0,0]);
 assert.deepEqual(centerOfLift([panel(0,3),panel(2,-1)]),[-1,0,0]);
 assert.deepEqual(centerOfLift([panel(0,-1),panel(2,-3)]),[1.5,0,0]);
 assert.equal(centerOfLift([panel(0,0)]),undefined);
 assert.equal(centerOfLift([panel(0,1),panel(2,-1)]),undefined);
 assert.equal(centerOfLift([panel(0,1),panel(2,-.9999)]),undefined);
});

test('lift location follows changing replay state and preserves layer exposure weights',()=>{
 const positions:V3[]=[[0,0,0],[1,0,0],[0,0,1],[2,0,0],[3,0,0],[2,0,1]];
 const faces:[number,number,number][]=[[0,1,2],[3,4,5]],velocities:V3[]=positions.map(()=>[6,-1,0]);
 const settings={enabled:true,density:1.225,wind:[0,0,0] as V3,areaWeights:[1,.5]};
 const loads=()=>aerodynamicLoads(positions,velocities,faces,settings,positions.map(()=>[0,0,0]));
 assert.ok(Math.abs(centerOfLift(loads())![0]-1)<1e-12);
 velocities.slice(3).forEach(v=>{v[0]*=2;v[1]*=2;});
 assert.ok(Math.abs(centerOfLift(loads())![0]-5/3)<1e-12,'faster outer panel shifts lift centroid');
 settings.enabled=false;assert.equal(centerOfLift(loads()),undefined);
});
