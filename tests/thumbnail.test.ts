import {test} from 'node:test';
import assert from 'node:assert/strict';
import {designs} from '../src/folding/library';
import {designThumbnail} from '../src/folding/thumbnail';
import {DEFAULTS} from '../src/physics/simulation';

for(const {id,adjustableSpan} of designs){
 test(`${id} preview is finite across wing settings and ignores nonvisual parameters`,()=>{
  for(const dihedral of [0,8,16])for(const span of [.7,1,1.35]){
   const parameters={...DEFAULTS,dihedral,span};
   const svg=designThumbnail(id,parameters);
   assert.match(svg,/<polygon/);
   assert.doesNotMatch(svg,/NaN|Infinity/);
   assert.equal(designThumbnail(id,{...parameters,size:1.15,paperWeight:110}),svg);
  }
  const original=designThumbnail(id,DEFAULTS);
  assert.notEqual(designThumbnail(id,{...DEFAULTS,dihedral:16}),original);
  const wider=designThumbnail(id,{...DEFAULTS,span:1.35});
  if(!adjustableSpan)assert.equal(wider,original);
  else assert.notEqual(wider,original);
  assert.equal(designThumbnail(id,DEFAULTS),original);
 });
}
