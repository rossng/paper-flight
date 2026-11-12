import test from 'node:test';
import assert from 'node:assert/strict';
import { GroundContactSolver } from '../src/physics/ground-contact';
import { Simulation, DEFAULTS } from '../src/physics/simulation';
import { designMesh } from '../src/folding/design';
import type { V3 } from '../src/physics/math';

const friction={groundStaticFriction:.55,groundKineticFriction:.38};
function particle(dt:number,speed=2,mu=.38,acceleration=0,duration=2){
 const solver=new GroundContactSolver(1),p:V3[]=[[0,0,0]],v:V3[]=[[speed,0,0]],settings={groundStaticFriction:mu===0?0:.55,groundKineticFriction:mu};
 let previousSpeed=speed;
 for(let t=0;t<duration-dt/2;t+=dt){
  const old=structuredClone(p);v[0][0]+=acceleration*dt;v[0][1]-=9.81*dt;
  for(let d=0;d<3;d++)p[0][d]+=dt*v[0][d];
  solver.reset();for(let i=0;i<12;i++)solver.solve(p,old,[1],0,settings);
  for(let d=0;d<3;d++)v[0][d]=(p[0][d]-old[0][d])/dt;
  solver.solveVelocity(p,v,0,settings);
  assert.ok(p[0][1]>=-1e-12);
  if(acceleration===0){assert.ok(v[0][0]>=-1e-12);assert.ok(v[0][0]<=previousSpeed+1e-10);}
  previousSpeed=v[0][0];
 }
 return {position:p[0],velocity:v[0]};
}

test('Coulomb sliding approaches analytic stopping distance and never reverses velocity',()=>{
 const expected=2**2/(2*.38*9.81),coarse=particle(1/360),fine=particle(1/720);
 assert.ok(Math.abs(coarse.position[0]-expected)<.01);
 assert.ok(Math.abs(fine.position[0]-expected)<Math.abs(coarse.position[0]-expected));
 assert.equal(fine.velocity[0],0);
 assert.ok(particle(1/360,2,.2).position[0]>coarse.position[0]*1.7);
});

test('static friction holds below its load limit, slips above it, and zero friction preserves translation',()=>{
 assert.ok(Math.abs(particle(1/360,0,.38,4).position[0])<1e-10);
 assert.ok(particle(1/360,0,.38,7).position[0]>1);
 const free=particle(1/360,2,0);assert.ok(Math.abs(free.position[0]-4)<1e-10);assert.ok(Math.abs(free.velocity[0]-2)<1e-10);
});

test('ground cannot pull an airborne vertex down or apply friction without normal pressure',()=>{
 const solver=new GroundContactSolver(1),old:V3[]=[[0,0,0]],p:V3[]=[[.1,.01,.2]],v:V3[]=[[1,1,2]];
 solver.solve(p,old,[1],0,friction);assert.equal(solver.solveVelocity(p,v,0,friction),false);
 assert.deepEqual(p,[[.1,.01,.2]]);assert.deepEqual(v,[[1,1,2]]);assert.equal(solver.touched,false);
});

test('impact removes residual closing velocity and uses only its additional friction budget',()=>{
 const solver=new GroundContactSolver(1),old:V3[]=[[0,.01,0]],p:V3[]=[[.01,-.01,0]],v:V3[]=[[1,-2,0]],dt=.01;
 solver.solve(p,old,[1],0,friction);
 for(let d=0;d<3;d++)v[0][d]=(p[0][d]-old[0][d])/dt;
 assert.ok(v[0][1]<0,'position correction alone still yields a downward velocity');
 solver.solveVelocity(p,v,0,friction);
 assert.equal(v[0][1],0);assert.ok(v[0][0]>=0&&v[0][0]<1);assert.equal(p[0][1],0);
});

for(const pitch of [-30,0,90])test(`pitch ${pitch}: impact deforms, slides, and reaches a supported rest`,()=>{
 const sim=new Simulation({aero:false,height:.3,speed:4,pitch,launchDuration:0});sim.launch();
 let sawSettling=false,peakDeformation=0,steps=0;
 while(sim.active){
  sim.step();assert.ok(++steps<2400);sawSettling ||= sim.status==='settling';
  peakDeformation=Math.max(peakDeformation,sim.deformation().rms);
  assert.ok(sim.positions.every(p=>p.every(Number.isFinite)&&p[1]>=sim.groundHeight-1e-9));
 }
 assert.ok(sawSettling);assert.equal(sim.status,'landed');assert.ok(sim.touchdown);
 assert.ok(sim.time>sim.touchdown.time+.5);assert.ok(sim.groundDistance>.1);assert.ok(peakDeformation>.001);
 assert.ok(sim.velocities.every(v=>v.every(x=>x===0)));
 assert.equal(sim.metrics().airTime,sim.touchdown.time);assert.equal(sim.metrics().flightDistance,sim.touchdown.distance);
 const final=structuredClone(sim.positions);sim.step();assert.deepEqual(sim.positions,final);
 sim.launch();assert.equal(sim.status,'flying');assert.equal(sim.touchdown,undefined);assert.equal(sim.groundDistance,0);
 assert.ok(sim.deformation().rms<1e-12);assert.ok(sim.mesh.hinges.every(h=>h.plasticOffset===0));
});

test('a frictionless moving sheet times out without being mislabeled as resting',()=>{
 const sim=new Simulation({aero:false,height:.05,speed:1,pitch:0,launchDuration:0,groundStaticFriction:0,groundKineticFriction:0});sim.launch();
 while(sim.active)sim.step();
 assert.equal(sim.status,'finished');assert.ok(sim.touchdown);assert.ok(sim.time-sim.touchdown.time>=8);
 assert.ok(Math.abs(sim.center(sim.velocities)[0]-1)<1e-8);
});

test('sleep uses world velocity even when the airspeed readout is nonzero',()=>{
 const sim=new Simulation({aero:false,height:.1,speed:0,wind:[3,0,0]});sim.launch();
 while(sim.active)sim.step();
 assert.equal(sim.status,'landed');assert.equal(sim.metrics().speed,3);
 assert.ok(sim.velocities.every(v=>v.every(x=>x===0)));
});

test('a spinning sheet with almost stationary centre of mass stays awake',()=>{
 const sim=new Simulation({aero:false,height:.05,speed:0,pitch:0,launchDuration:0,groundStaticFriction:0,groundKineticFriction:0});sim.launch();
 const centre=sim.center();
 sim.velocities=sim.positions.map(p=>[-10*(p[2]-centre[2]),0,10*(p[0]-centre[0])]);
 while(sim.active&&sim.time<2)sim.step();
 assert.equal(sim.status,'settling');assert.ok(sim.touchdown);
 assert.ok(Math.hypot(sim.center(sim.velocities)[0],sim.center(sim.velocities)[2])<1e-8);
 assert.ok(sim.velocities.some(v=>Math.hypot(...v)>.1));
});

test('a compiled folded dart settles its overlapping layers after touchdown',()=>{
 const parameters={...DEFAULTS,speed:7,pitch:15,elevation:8};
 const sim=new Simulation(parameters,designMesh('dart',parameters));sim.launch();
 while(sim.active)sim.step();
 assert.equal(sim.status,'landed');assert.ok(sim.touchdown);assert.ok(sim.groundDistance>.1);
 assert.ok(sim.positions.every(p=>p.every(Number.isFinite)&&p[1]>=sim.groundHeight-1e-9));
 assert.ok(sim.metrics().maxStrain<.1);assert.ok(sim.velocities.every(v=>v.every(x=>x===0)));
});
