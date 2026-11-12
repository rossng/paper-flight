import test from 'node:test';
import assert from 'node:assert/strict';
import {designMesh} from '../src/folding/design';
import {DEFAULTS} from '../src/physics/simulation';
test('fixed-width Folded Dart ignores span carried over from another design',()=>{
 const baseline=designMesh('dart',DEFAULTS);
 for(const span of [.7,1.35])assert.deepEqual(designMesh('dart',{...DEFAULTS,span}),baseline);
});
test('adjustable recipes still change actual deployed wing span',()=>{
 for(const id of ['basic','stable']){
  const width=(span:number)=>{const mesh=designMesh(id,{...DEFAULTS,span});return Math.max(...mesh.rest.map(p=>p[2]))-Math.min(...mesh.rest.map(p=>p[2]));};
  assert.ok(width(1.35)>width(.7)+.1,id);
 }
});
