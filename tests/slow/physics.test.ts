import test from 'node:test';
import assert from 'node:assert/strict';
import {Simulation} from '../../src/physics/simulation';

test('representative parameter extremes stay finite through contact, including high-angle stalls',()=>{
 const cases=[
 {speed:3,pitch:-5,stiffness:0.2,crease:0.25,dihedral:0},
 {speed:9,pitch:25,stiffness:0.2,crease:0.25,dihedral:18},
 {speed:9,pitch:-5,stiffness:5,crease:4,dihedral:0},
 {speed:3,pitch:25,stiffness:5,crease:4,dihedral:18},
 {speed:9,pitch:8,stiffness:0.2,crease:4,dihedral:0},
 {speed:9,pitch:25,stiffness:5,crease:0.25,dihedral:18},
 ];
 let sawStall=false;
 for(const parameters of cases){const s=new Simulation(parameters);s.launch();let steps=0;
 while(s.active&&steps++<2402){s.step();const m=s.metrics();assert.ok(Number.isFinite(m.deformation));assert.ok(m.maxStrain<0.12);assert.ok(m.speed<25);sawStall ||=m.stallFraction>0.5;}
 assert.ok(['landed','finished'].includes(s.status));
 }
 assert.ok(sawStall);
});

test('workbench extremes complete finite directional flights',()=>{
 for(const [size,paperWeight,span,heading] of [[0.85,60,0.7,-70],[1.15,110,1.35,70]]){
  const s=new Simulation({size,paperWeight,span,heading,stiffness:(paperWeight/80)**3,speed:8,elevation:16,pitch:23});s.launch();
  while(s.active)s.step();
  assert.equal(s.status,'landed');assert.ok(s.metrics().distance>1);assert.ok(s.positions.flat().every(Number.isFinite));
 }
});

test('vertical, backward and inverted pitches complete finite flights',()=>{
 for(const pitch of [-180,-135,-90,90,135,180]){
  const s=new Simulation({pitch,speed:6});s.launch();
  while(s.active)s.step();
  assert.ok(['landed','finished'].includes(s.status),`pitch ${pitch}: ${s.status}`);
  assert.ok(s.positions.flat().every(Number.isFinite));
 }
});
