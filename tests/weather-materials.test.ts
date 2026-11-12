import test from 'node:test';
import assert from 'node:assert/strict';
import {airVelocity} from '../src/physics/weather';
import {materials,materialSettings} from '../src/physics/materials';
import {Simulation,DEFAULTS} from '../src/physics/simulation';
import {aerodynamicLoads} from '../src/physics/aerodynamics';
import {designMesh} from '../src/folding/design';
import {launchSpeed} from '../src/physics/launcher';
import {length,sub,type V3} from '../src/physics/math';

test('weather is deterministic, spatially varying, continuous and calm by default',()=>{
 const point:V3=[2,1.8,.2];assert.deepEqual(airVelocity(DEFAULTS,4,point),[0,0,0]);
 const weather={wind:[0,0,2] as V3,gusts:1.5,turbulence:1};
 assert.deepEqual(airVelocity(weather,.7,point),airVelocity(weather,.7,point));
 assert.ok(length(sub(airVelocity(weather,.7,point),airVelocity(weather,1.7,point)))>.1);
 assert.ok(length(sub(airVelocity(weather,.7,point),airVelocity(weather,.7,[2,1.8,.4])))>.01);
 assert.ok(length(sub(airVelocity(weather,.7,point),airVelocity(weather,.700001,point)))<.0001);
 const calmGust=airVelocity({wind:[0,0,0],windHeading:90,gusts:1,turbulence:0},1,point);assert.ok(calmGust[2]>.5);assert.ok(Math.abs(calmGust[0])<1e-8);
 let divergence=0;for(let d=0;d<3;d++){const a=[...point] as V3,b=[...point] as V3;a[d]+=.001;b[d]-=.001;divergence+=(airVelocity(weather,.7,a)[d]-airVelocity(weather,.7,b)[d])/.002;}assert.ok(Math.abs(divergence)<1e-10);
});

test('panel aerodynamics samples local weather and replay samples the requested time',()=>{
 const positions:V3[]=[[0,0,0],[.2,0,0],[0,0,.2]],velocities:V3[]=positions.map(()=>[6,-1,0]),faces:[number,number,number][]=[[0,1,2]];
 const wind:V3=[1,.2,-.5],zeros=()=>positions.map(()=>[0,0,0] as V3);
 const settings={enabled:true,density:1.225,wind};
 const baseline=aerodynamicLoads(positions,velocities,faces,settings,zeros())[0];
 const local=aerodynamicLoads(positions,velocities,faces,{...settings,wind:[0,0,0],windAt:()=>wind},zeros())[0];assert.deepEqual(local,baseline);
 const sim=new Simulation({wind,gusts:1.5,turbulence:1});sim.time=8;
 assert.deepEqual(sim.aeroSettings(.3).windAt!(positions[0]),airVelocity(sim.parameters,.3,positions[0]));
 assert.notDeepEqual(sim.aeroSettings(.3).windAt!(positions[0]),sim.aeroSettings().windAt!(positions[0]));
});

test('materials change real mass and bending properties, including folded layers',()=>{
 const baseline=designMesh('dart',DEFAULTS).masses.reduce((a,b)=>a+b,0);
 for(const material of ['tissue','foil','steel'] as const){
  const parameters={...DEFAULTS,...materialSettings(material)},mesh=designMesh('dart',parameters),mass=mesh.masses.reduce((a,b)=>a+b,0);
  assert.ok(Math.abs(mass/baseline-materials[material].gsm/80)<1e-10);
  assert.equal(parameters.stiffness,materials[material].stiffness);assert.equal(parameters.crease,materials[material].crease);
 }
 assert.deepEqual(materialSettings('paper',60),{material:'paper',paperWeight:60,stiffness:(60/80)**3,crease:1});
});

for(const material of ['tissue','foil','steel'] as const)for(const design of ['legacy','basic','dart','stable'])test(`${material}/${design}: full-speed launch in gusty crosswind stays finite`,()=>{
 const parameters={...DEFAULTS,...materialSettings(material),wind:[0,0,3] as V3,gusts:1.5,turbulence:1};
 const sim=new Simulation(parameters,design==='legacy'?undefined:designMesh(design,parameters));sim.parameters.speed=launchSpeed(100);sim.launch();
 assert.ok(sim.velocities.every(v=>length(v)===0),'elastic launch starts from rest');
 let strain=0;while(sim.status==='flying'){sim.step();strain=Math.max(strain,sim.metrics().maxStrain);}
 assert.equal(sim.status,'landed');assert.ok(sim.positions.flat().every(Number.isFinite));assert.ok(strain<.2,`maximum edge strain ${strain}`);
});

for(const design of ['legacy','basic','dart','stable'])test(`tissue/${design}: holds its folds through an ordinary launch while remaining flexible`,()=>{
 const parameters={...DEFAULTS,...materialSettings('tissue'),speed:7.15,pitch:15,elevation:11.8};
 const sim=new Simulation(parameters,design==='legacy'?undefined:designMesh(design,parameters));sim.launch();
 let peak=0;while(sim.status==='flying'&&sim.time<.35){sim.step();peak=Math.max(peak,sim.deformation().rms);}
 assert.equal(sim.status,'flying');assert.ok(peak<.035,`early collapse: ${peak*1000} mm RMS`);
 assert.ok(peak>.001,`expected visible flex, got ${peak*1000} mm RMS`);
 assert.ok(Math.max(...sim.mesh.hinges.map(h=>Math.abs(h.plasticOffset??0)))<.1,'launch must not permanently crumple the creases');
});
