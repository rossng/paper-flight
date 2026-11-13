import test from 'node:test';
import assert from 'node:assert/strict';
import { designMesh, supportsElevator } from '../src/folding/design';
import { designs } from '../src/folding/library';
import { DEFAULTS, Simulation } from '../src/physics/simulation';
import { aerodynamicLoads } from '../src/physics/aerodynamics';
import { createDart, LEGACY_REFLEX_DEGREES } from '../src/geometry/dart';
import { readDesignSettings } from '../src/folding/settings';
import { cross, dot, sub, type V3 } from '../src/physics/math';

const folded=designs.filter(d=>d.recipe).map(d=>d.id);
test('elevator flap is available on every folded wing that can take a straight trailing-edge crease',()=>{
 assert.deepEqual(folded.filter(id=>!supportsElevator(id)),['hunting']);
 assert.ok(supportsElevator('legacy'));
 // Winglets would have to tear for the flap to rotate; the design ignores the setting.
 assert.deepEqual(designMesh('hunting',{...DEFAULTS,elevator:12}),designMesh('hunting',DEFAULTS));
});
for(const id of folded.filter(supportsElevator))test(`${id}: elevator folds the trailing edge without moving the keel or mass`,()=>{
 const flat=designMesh(id,DEFAULTS),up=designMesh(id,{...DEFAULTS,elevator:12}),down=designMesh(id,{...DEFAULTS,elevator:-12});
 const mass=(m:typeof flat)=>m.masses.reduce((a,b)=>a+b,0);
 assert.ok(Math.abs(mass(up)-mass(flat))<1e-12);
 // Keel (centre-plane) vertices are untouched.
 for(const p of flat.rest.filter(p=>Math.abs(p[2])<1e-9))assert.ok(up.rest.some(q=>Math.hypot(q[0]-p[0],q[1]-p[1],q[2]-p[2])<1e-9));
 // The outer trailing corner rises for a positive angle and drops for a negative one.
 const corner=(m:typeof flat)=>m.rest.reduce((best,p)=>Math.abs(p[2])-p[0]>Math.abs(best[2])-best[0]?p:best);
 assert.ok(corner(up)[1]>corner(flat)[1]+.003&&corner(down)[1]<corner(flat)[1]-.003);
 assert.ok(up.hinges.some(h=>h.crease&&Math.abs(Math.abs(h.rest)-12*Math.PI/180)<1e-6));
});
for(const id of ['legacy',...folded.filter(supportsElevator)])test(`${id}: raising the elevator adds nose-up pitching moment`,()=>{
 const moment=(elevator:number)=>{
  const p={...DEFAULTS,elevator},s=id==='legacy'?new Simulation(p):new Simulation(p,designMesh(id,p)),x=s.mesh.rest,cg=s.center(x),v:V3=[5,-.6,0];
  const panels=aerodynamicLoads(x,x.map(()=>v),s.mesh.faces,s.aeroSettings(0),x.map(()=>[0,0,0] as V3));
  return panels.reduce((sum,panel)=>sum+dot(cross(sub(panel.center,cg),panel.force),[0,0,1]),0);
 };
 assert.ok(moment(12)>moment(0)+1e-4,`${moment(12)} vs ${moment(0)}`);
});
test('the original dart keeps its authored reflex and maps elevator to the same rear ramp',()=>{
 const tail=(m:ReturnType<typeof createDart>)=>Math.max(...m.rest.filter(p=>p[0]<-.1).map(p=>p[1]));
 assert.deepEqual(createDart(),createDart(8,8,5,1,1,.005,LEGACY_REFLEX_DEGREES));
 assert.ok(Math.abs(Math.tan(LEGACY_REFLEX_DEGREES*Math.PI/180)*.09-.006)<1e-12);
 assert.ok(tail(createDart(8,8,5,1,1,.005,10))>tail(createDart(8,8,5,1,1,.005,0)));
});
test('elevator is read from game URLs within its range',()=>{
 assert.equal(readDesignSettings(new URLSearchParams('elevator=8')).elevator,8);
 assert.equal(readDesignSettings(new URLSearchParams('elevator=90')).elevator,DEFAULTS.elevator);
});
