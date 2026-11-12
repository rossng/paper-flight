import test from 'node:test';
import assert from 'node:assert/strict';
import {Simulation,DEFAULTS} from '../../src/physics/simulation';
import {materialSettings} from '../../src/physics/materials';
import {designMesh} from '../../src/folding/design';
import {launchSpeed} from '../../src/physics/launcher';
import {length,type V3} from '../../src/physics/math';

// Separate files let Node distribute the expensive design sweeps across its
// existing two workers. Each case still uses the production solver settings.
export function materialFlights(design:string){
for(const material of ['tissue','foil','steel'] as const)test(`${material}/${design}: full-speed launch in gusty crosswind stays finite`,()=>{
 const parameters={...DEFAULTS,...materialSettings(material),wind:[0,0,3] as V3,gusts:1.5,turbulence:1};
 const sim=new Simulation(parameters,design==='legacy'?undefined:designMesh(design,parameters));sim.parameters.speed=launchSpeed(100);sim.launch();
 assert.ok(sim.velocities.every(v=>length(v)===0),'elastic launch starts from rest');
 let strain=0;while(sim.status==='flying'){sim.step();strain=Math.max(strain,sim.metrics().maxStrain);}
 assert.equal(sim.status,'landed');assert.ok(sim.positions.flat().every(Number.isFinite));assert.ok(strain<.2,`maximum edge strain ${strain}`);
});
}
