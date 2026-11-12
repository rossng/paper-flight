import test from 'node:test';
import assert from 'node:assert/strict';
import {designMesh} from '../../src/folding/design';
import {recipes} from '../../src/folding/recipes';
import {DEFAULTS,Simulation} from '../../src/physics/simulation';

for(const {id} of recipes)test(`${id}: compiled sheet reaches touchdown`,()=>{
 const p={...DEFAULTS,speed:7,pitch:15,elevation:8},mesh=designMesh(id,p);
 const s=new Simulation(p,mesh);s.launch();while(s.status==='flying')s.step();assert.equal(s.status,'settling');assert.ok(s.touchdown);assert.ok(s.metrics().distance>1);assert.ok(s.metrics().maxStrain<.1);assert.ok(s.peakDeformation>1e-6);
});
test('folded material options retain area scaling and reach touchdown after an extreme launch',()=>{
 const p={...DEFAULTS,size:.85,span:1.35,paperWeight:60,stiffness:(60/80)**3,dihedral:16,pitch:35,elevation:16,heading:-70,speed:9},mesh=designMesh('stable',p),s=new Simulation(p,mesh);
 assert.ok(Math.abs(mesh.masses.reduce((a,b)=>a+b,0)-recipes[2].width*recipes[2].height*.85**2*.060)<1e-12);s.launch();while(s.status==='flying')s.step();assert.equal(s.status,'settling');assert.ok(s.touchdown);assert.ok(s.positions.flat().every(Number.isFinite));
});
