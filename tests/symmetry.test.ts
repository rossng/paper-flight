import test from 'node:test';
import assert from 'node:assert/strict';
import {recipes} from '../src/folding/recipes';
import {designMesh} from '../src/folding/design';
import {DEFAULTS,Simulation} from '../src/physics/simulation';
import {aerodynamicLoads} from '../src/physics/aerodynamics';
import type {V3} from '../src/physics/math';

/** Largest deviation from mirror symmetry about the launch plane z = z0. */
function asymmetry(values:V3[],mirror:Int32Array,z0=0){
  let worst=0;
  values.forEach((p,i)=>{const q=values[mirror[i]];worst=Math.max(worst,Math.abs(p[0]-q[0]),Math.abs(p[1]-q[1]),Math.abs(p[2]-z0+q[2]-z0));});
  return worst;
}
// The Buzz's folded nose is one-sided, so it has no mirror image.
const symmetric=recipes.map(r=>r.id).filter(id=>id!=='buzz');

for(const id of symmetric)test(`${id}: compiled flight mesh is mirror-symmetric, layer by layer`,()=>{
  assert.ok(new Simulation(DEFAULTS,designMesh(id,DEFAULTS)).mirror,'no mirror map');
});
test('legacy dart has a mirror map',()=>assert.ok(new Simulation(DEFAULTS).mirror));

test('dart: coupled and panel air loads on the folded rest shape are mirror-symmetric',()=>{
  const s=new Simulation(DEFAULTS,designMesh('dart',DEFAULTS)),mirror=s.mirror!;
  const positions=s.mesh.rest.map(p=>[...p] as V3),velocities=positions.map(()=>[6,-.6,0] as V3);
  for(const model of ['coupled','panels'] as const){
    const forces=positions.map(()=>[0,0,0] as V3);aerodynamicLoads(positions,velocities,s.mesh.faces,{...s.aeroSettings(),model},forces);
    const scale=Math.max(...forces.flat().map(Math.abs));
    assert.ok(asymmetry(forces,mirror)<1e-12*scale,`${model} loads favour one wing`);
  }
});

for(const layerModel of ['contact','bonded'] as const)
test(`dart (${layerModel}): a straight launch stays mirror-symmetric`,()=>{
  // Gauss-Seidel order, contact side choice and aero patching used to seed a
  // one-sided roll of ~1e-7 m per step, enough to curl every flight one way.
  const p={...DEFAULTS,speed:9,pitch:15,elevation:6,layerModel},s=new Simulation(p,designMesh('dart',p)),mirror=s.mirror!;
  s.launch();for(let i=0;i<12;i++)s.step();
  assert.ok(asymmetry(s.positions,mirror,s.start[2])<1e-10,`asymmetry ${asymmetry(s.positions,mirror,s.start[2])} m`);
});
