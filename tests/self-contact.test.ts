import test from 'node:test';
import assert from 'node:assert/strict';
import type { PaperMesh, Face } from '../src/geometry/dart';
import { add, cross, length, scale, sub, type V3 } from '../src/physics/math';
import { SelfContactSolver, solveSelfContact, coplanarityRoots } from '../src/physics/self-contact';
import { buildContactCandidates, detectLayerContacts } from '../src/physics/layer-contact';
import { Simulation, DEFAULTS } from '../src/physics/simulation';
import { intersections } from './fixtures/intersections';
import { designMesh } from '../src/folding/design';
import { materialSettings } from '../src/physics/materials';
import { pathToFileURL } from 'node:url';

// Optional baseline constructor for demonstrating that the high-load bounds
// fail against the dispatched solver, without changing the checkout's code.
const FlightSimulation:typeof Simulation=process.env.PAPER_CONTACT_BASELINE
  ?(await import(pathToFileURL(process.env.PAPER_CONTACT_BASELINE).href)).Simulation:Simulation;

function mesh(rest:V3[],faces:Face[]=[[0,1,2],[3,4,5]]):PaperMesh {
  return {rest,faces,edges:[],hinges:[],masses:rest.map(()=>1),regions:rest.map(()=>0),nose:0,tail:1,leftRoot:2,rightRoot:3};
}
const copy=(p:V3[])=>structuredClone(p);
const total=(p:V3[])=>p.reduce(add,[0,0,0]);
const close=(a:V3,b:V3,tol=1e-10)=>assert.ok(length(sub(a,b))<tol,`${a} != ${b}`);
const old:V3[]=[[0,0,0],[0,0,1],[1,0,0],[.2,.05,.2],[.3,.05,.2],[.2,.05,.3]];

test('swept vertex/face stops a fast patch even when its final projection misses the face',()=>{
  const m=mesh(old),p=old.map((v,i)=>i<3?[...v] as V3:add(v,[1,-.2,0]));
  assert.equal(detectLayerContacts(p,old,buildContactCandidates(m)).length,0,'baseline has no candidates outside rest-coplanar groups');
  const solver=new SelfContactSolver(m);solver.beginStep();solver.detect(old,p);
  assert.ok(solver.contacts.some(c=>c.kind==='vertex-face'));
  for(let i=0;i<20;i++)solver.solve(p,[0,0,0,1,1,1]);
  assert.ok(p.slice(3).every(v=>v[1]>=-1e-10));
  assert.ok(p[3][0]>1,'tangential motion is not frozen');
});

test('edge/edge sweep catches crossing strips with no vertex/face impact',()=>{
  const start:V3[]=[[-1,0,-.01],[1,0,-.01],[1,0,.01],[-.01,.05,-1],[-.01,.05,1],[.01,.05,1]];
  const p=start.map((v,i)=>add(v,[0,i<3?0:-.1,0])),solver=new SelfContactSolver(mesh(start));
  solver.beginStep();solver.detect(start,p);
  assert.ok(solver.contacts.length>0);assert.ok(solver.contacts.every(c=>c.kind==='edge-edge'));
  for(let i=0;i<40;i++)solver.solve(p,[0,0,0,1,1,1]);
  // Free endpoints can tilt below the plane outside the opposing strip.
  assert.ok(Math.max(...p.slice(3).map(v=>v[1]))>.01);
  assert.equal(intersections(p,[[0,1,2],[3,4,5]]).pairs,0);
});

test('root isolation includes two crossings with equal endpoint signs and a tangent root',()=>{
  const roots=coplanarityRoots([.1875,-1,1,0]);
  assert.equal(roots.length,2);assert.ok(Math.abs(roots[0]-.25)<1e-8);assert.ok(Math.abs(roots[1]-.75)<1e-8);
  assert.ok(Math.abs(coplanarityRoots([.25,-1,1,0])[0]-.5)<1e-8);
  assert.deepEqual(coplanarityRoots([0,0,0,0]),[]);
});

test('normal projections conserve mass centre and instantaneous linear/angular impulse',()=>{
  for(const kind of ['vertex-face','edge-edge'] as const){
    const p=copy(old);p[3][1]=-.02;
    const ids:[number,number,number,number]=kind==='vertex-face'?[3,0,1,2]:[0,2,3,5];
    const contact={ids,kind,sign:1,key:0},before=copy(p),w=[1,2,3,4,5,6];
    solveSelfContact(p,w,contact);
    const impulses=p.map((v,i)=>scale(sub(v,before[i]),1/w[i]));
    assert.ok(length(total(impulses.map(v=>v.map(Math.abs) as V3)))>1e-5,'must exercise a correction');
    close(total(impulses),[0,0,0]);close(total(impulses.map((v,i)=>cross(before[i],v))),[0,0,0]);
  }
});

test('unilateral contact allows separation, retains touching side, and catches recontact',()=>{
  const solver=new SelfContactSolver(mesh(old)),p=copy(old);p.slice(3).forEach(v=>v[1]=-.02);
  solver.beginStep();solver.detect(old,p);solver.solve(p,[0,0,0,1,1,1]);
  const touching=copy(p);p.slice(3).forEach(v=>v[1]=-.02);
  solver.beginStep();solver.detect(touching,p);solver.solve(p,[0,0,0,1,1,1]);
  assert.ok(p.slice(3).every(v=>v[1]>=-1e-10));
  const opened=copy(p);opened.slice(3).forEach(v=>v[1]=.05);
  solver.beginStep();solver.detect(p,opened);solver.solve(opened,[0,0,0,1,1,1]);assert.equal(opened[3][1],.05);
  const returned=copy(opened);returned.slice(3).forEach(v=>v[1]=-.1);
  solver.beginStep();solver.detect(opened,returned);solver.solve(returned,[0,0,0,1,1,1]);assert.ok(returned[3][1]>=-1e-10);
});

test('initial coincidence opens freely, material neighbors are excluded, and reset clears history',()=>{
  const flat=copy(old);flat.slice(3).forEach(v=>v[1]=0);
  const solver=new SelfContactSolver(mesh(flat)),p=copy(flat);p.slice(3).forEach(v=>v[1]=-.05);
  solver.beginStep();solver.detect(flat,p);assert.equal(solver.contacts.length,0);
  const adjacent=new SelfContactSolver(mesh(old,[[0,1,2],[0,3,4],[3,4,5]]));adjacent.beginStep();adjacent.detect(old,p);
  assert.ok(adjacent.contacts.every(c=>!c.ids.includes(0)||!c.ids.includes(3)));
  solver.reset();solver.beginStep();solver.detect(flat,p);assert.equal(solver.contacts.length,0);
});

test('swept contacts are invariant under rigid transforms and face winding',()=>{
  const p=old.map((v,i)=>add(v,[0,i<3?0:-.1,0])),m=mesh(old),solver=new SelfContactSolver(m);
  solver.beginStep();solver.detect(old,p);const w=[0,0,0,1,1,1];solver.solve(p,w);
  const transform=([x,y,z]:V3):V3=>[3+z,-2+x,7+y];
  const start=old.map(transform),q=old.map((v,i)=>transform(add(v,[0,i<3?0:-.1,0])));
  const rotated=new SelfContactSolver(mesh(start,m.faces.map(([a,b,c])=>[a,c,b])));
  rotated.beginStep();rotated.detect(start,q);rotated.solve(q,w);q.forEach((v,i)=>close(v,transform(p[i])));
});

test('moving-frame broad phase culls separated patches even in a fast rigid translation',()=>{
  const rest:V3[]=[],faces:Face[]=[];
  for(let i=0;i<20;i++){const n=rest.length;rest.push([i*10,0,0],[i*10+1,0,0],[i*10,0,1]);faces.push([n,n+1,n+2]);}
  const solver=new SelfContactSolver(mesh(rest,faces)),p=rest.map(v=>add(v,[100,20,-30]));
  solver.beginStep();solver.detect(rest,p);
  assert.equal(solver.contacts.length,0);assert.equal(solver.stats.narrowTests,0);
  solver.solve(p,p.map(()=>1));assert.deepEqual(p,rest.map(v=>add(v,[100,20,-30])));
});

test('full-power gusty Original dart bounds the previously reproduced high-load crossing',()=>{
  const sim=new FlightSimulation({...DEFAULTS,speed:11,wind:[0,0,3],gusts:1.5,turbulence:1});sim.launch();
  let depth=0;
  for(let i=0;i<42;i++){sim.step();depth=Math.max(depth,intersections(sim.positions,sim.mesh.faces).depth);}
  assert.ok(depth<.0001,`peak transverse depth ${1000*depth} mm (baseline 13.66 mm)`);
  assert.ok(sim.positions.flat().every(Number.isFinite));assert.equal(sim.status,'flying');
  const end=copy(sim.positions);sim.launch();for(let i=0;i<42;i++)sim.step();assert.deepEqual(sim.positions,end);
});

for(const [design,material,limitMm] of [['basic','paper',6],['dart','paper',2],['stable','paper',12],['dart','tissue',2],['stable','tissue',15]] as const)
test(`${material}/${design}: bounds high-load transverse depth in compiled zero-thickness layers`,()=>{
  const parameters={...DEFAULTS,...materialSettings(material),speed:11,wind:[0,0,3] as V3,gusts:1.5,turbulence:1};
  const sim=new FlightSimulation(parameters,designMesh(design,parameters));sim.launch();let depth=0;
  for(let i=0;i<42;i++){sim.step();depth=Math.max(depth,intersections(sim.positions,sim.mesh.faces).depth);}
  // These bounds fail on the measured baseline; they deliberately do not assert
  // zero intersections in an unordered, initially coincident recipe stack.
  assert.ok(depth*1000<limitMm,`peak transverse depth ${1000*depth} mm`);
  assert.ok(sim.positions.flat().every(Number.isFinite));assert.equal(sim.status,'flying');
});
