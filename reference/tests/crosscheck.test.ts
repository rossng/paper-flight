/** Checks that the reference solver integrates the same model as the game, and that it does
 * so correctly. Needs a Rust toolchain; run with `npm run test:reference` (not part of CI). */
import test from 'node:test';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { homedir } from 'node:os';
import { join } from 'node:path';
import { caseFor, gameSimulation, type Case } from '../export';
import { caseLabel, solve, solveAll, solverDirectory, staticLoads } from '../run';
import { flyGame, compare } from '../compare';
import { foldHistory, stackingSides } from '../stack-order';
import { aerodynamicLoads } from '../../src/physics/aerodynamics';
import { buildMembranes } from '../../src/physics/membrane';
import { hingeStiffness } from '../../src/physics/sheet-material';
import { hingeAngle } from '../../src/geometry/dart';
import { flightRecipe } from '../../src/folding/design';
import { build } from '../../src/folding/model';
import { cross, dot, sub, wrapAngle, type V3 } from '../../src/physics/math';
import type { Parameters } from '../../src/physics/simulation';

/** A mid-flight state from the game: deformed, moving, layers partly open. */
function flightState(test:Case,frames=40){
  const sim=gameSimulation(test);sim.launch();
  for(let i=0;i<frames;i++)sim.step();
  return sim;
}
const relative=(a:number[],b:number[])=>Math.sqrt(a.reduce((sum,v,i)=>sum+(v-b[i])**2,0)/Math.max(b.reduce((sum,v)=>sum+v*v,0),1e-30));

test('solver unit tests pass',()=>{
  const result=spawnSync('cargo',['test','--release','--quiet'],{cwd:solverDirectory,encoding:'utf8',env:{...process.env,PATH:`${process.env.PATH}:${join(homedir(),'.cargo','bin')}`}});
  assert.equal(result.status,0,result.stdout+result.stderr);
});

for(const [design,overrides] of [['legacy',{}],['basic',{}],['stable',{}],['raven',{}],['dart',{material:'tissue',wind:[0,0,3],gusts:1.5,turbulence:1}]] as [string,Partial<Parameters>][])
test(`air loads match the game's kernel on ${caseLabel(design,overrides)}`,()=>{
  const flight=caseFor(design,overrides),sim=flightState(flight),forces=sim.positions.map(()=>[0,0,0] as V3);
  aerodynamicLoads(sim.positions,sim.velocities,sim.mesh.faces,sim.aeroSettings(),forces);
  // Same lattice and one patch per clipped polygon: the two implementations must agree.
  const same=staticLoads(flight,sim.positions,sim.velocities,{patchArea:0},sim.time);
  assert.ok(relative(same.air,forces.flat())<1e-9,`nodal air loads differ by ${relative(same.air,forces.flat())}`);
  // Finer quadrature is a small correction, not a different answer.
  const fine=staticLoads(flight,sim.positions,sim.velocities,{},sim.time),total=forces.reduce((sum,f)=>sum.map((v,d)=>v+f[d]) as V3,[0,0,0] as V3);
  const error=Math.hypot(...fine.airForce.map((v,d)=>v-total[d]))/Math.hypot(...total);
  assert.ok(fine.patches>2*sim.surface.patches.length&&error<.05,`fine quadrature changes the net air load by ${error}`);
});

for(const design of ['legacy','basic','heavy-nose'])
test(`elastic energy matches the game's constraint definitions on ${design}`,()=>{
  const flight=caseFor(design),sim=flightState(flight),material=sim.material,p=sim.positions;
  let membrane=0,bending=0;
  for(const t of buildMembranes(sim.mesh)){
    const [a,b,c]=t.face.map(i=>p[i]),u=[0,1,2].map(d=>t.md[0]*a[d]+t.md[1]*b[d]+t.md[2]*c[d]) as V3,v=[0,1,2].map(d=>t.cd[0]*a[d]+t.cd[1]*b[d]+t.cd[2]*c[d]) as V3;
    membrane+=material.thickness*t.area/2*(material.youngMD*((dot(u,u)-1)/2)**2+material.youngCD*((dot(v,v)-1)/2)**2+material.shear*dot(u,v)**2);
  }
  for(const h of sim.mesh.hinges){
    const stiffness=hingeStiffness(material,h)*(h.layers??1)**(2*sim.parameters.stackCoupling);
    bending+=stiffness/2*wrapAngle(hingeAngle(...h.ids.map(i=>p[i]) as [V3,V3,V3,V3])-h.rest)**2;
  }
  const reference=staticLoads(flight,p,sim.velocities);
  assert.ok(Math.abs(reference.membraneEnergy/membrane-1)<1e-9,`membrane ${reference.membraneEnergy} vs ${membrane}`);
  assert.ok(Math.abs(reference.bendingEnergy/bending-1)<1e-9,`bending ${reference.bendingEnergy} vs ${bending}`);
  // Forces are the energy gradient: check by a directional finite difference. In flight the
  // membrane and bending parts nearly cancel, so the tolerance is set by their gross size.
  const direction=p.map((_,i)=>[Math.sin(i),Math.cos(2*i),Math.sin(3*i+1)]),step=1e-8;
  const moved=(sign:number)=>staticLoads(flight,p.map((q,i)=>q.map((v,d)=>v+sign*step*direction[i][d])),sim.velocities);
  const plus=moved(1),minus=moved(-1),stretch=(plus.membraneEnergy-minus.membraneEnergy)/(2*step),bend=(plus.bendingEnergy-minus.bendingEnergy)/(2*step);
  const work=-reference.elastic.reduce((sum,f,k)=>sum+f*direction[Math.floor(k/3)][k%3],0);
  assert.ok(Math.abs(stretch+bend-work)<1e-3*(Math.abs(stretch)+Math.abs(bend)),`energy slope ${stretch+bend} vs force work ${work}`);
});

test('a corner folded over the sheet lies above it',()=>{
  // The Basic's first corner fold turns the corner over the top (+z) of the flat sheet.
  const operations=flightRecipe('basic',caseFor('basic').parameters).operations.slice(0,2);
  assert.equal(operations[1].id,'left-corner');
  const flap=foldHistory([-.05,.01],operations),under=foldHistory([flap.position[0],flap.position[1]],operations);
  assert.ok(Math.hypot(...flap.position.map((v,d)=>v-under.position[d]))<1e-12&&under.stack.every(v=>v===0));
  assert.ok(Math.abs(dot(flap.up,under.up)+1)<1e-12,'the flap is turned over');
  assert.ok(flap.stack[1]*Math.sign(dot(flap.up,under.up))>0&&under.up[2]===1,'and rests on the +z side of the sheet beneath');
});

test('fold order is the same for every vertex of a pair of panels, and opposite seen from the other panel',()=>{
  for(const design of ['basic','stable','buzz']){
    const flight=caseFor(design),sim=gameSimulation(flight),mesh=sim.mesh,owners=mesh.faceOwners!;
    const sides=stackingSides(build(flightRecipe(design,flight.parameters)),mesh);
    assert.ok(sides.length>100);
    const normal=new Map<number,V3>();
    mesh.faces.forEach((f,i)=>{if(!normal.has(owners[i]))normal.set(owners[i],cross(sub(mesh.rest[f[1]],mesh.rest[f[0]]),sub(mesh.rest[f[2]],mesh.rest[f[0]])));});
    // World direction from panel to layer, per ordered pair of panels: uniform, and opposite for the reverse pair.
    const direction=new Map<string,V3>();
    for(const side of sides){
      const key=`${side.layer}>${side.owner}`,d=normal.get(side.owner)!.map(v=>v*side.sign) as V3,seen=direction.get(key);
      if(seen)assert.ok(dot(seen,d)>0,`${design}: panels ${key} are ordered both ways`);else direction.set(key,d);
    }
    for(const [key,d] of direction){const reverse=direction.get(key.split('>').reverse().join('>'));if(reverse)assert.ok(dot(reverse,d)<0,`${design}: ${key} and its reverse agree`);}
  }
});

test('the sweep solver converges towards the reference as its sweeps and substeps increase',async()=>{
  const flight=caseFor('legacy'),reference=await solve({test:flight,overrides:{},options:{maxTime:.5}},undefined,false);
  const coarse=compare(reference,flyGame(flight,{elasticSolve:'sweeps'})),fine=compare(reference,flyGame(flight,{elasticSolve:'sweeps',substeps:48,iterations:100}));
  assert.ok(fine.separation[.25]!<.01,`converged game is ${fine.separation[.25]} m from the reference at 0.25 s`);
  assert.ok(fine.separation[.25]!<.5*coarse.separation[.25]!,`default ${coarse.separation[.25]} m, converged ${fine.separation[.25]} m`);
});

test('the game flies the default launch within a few percent of the reference',async()=>{
  // The two flights where under-converged sweeps cost half the range, and a layered glider.
  const flights=['legacy','v-wing','hunting'].map(design=>({test:caseFor(design),overrides:{}}));
  const references=await solveAll(flights,3);
  references.forEach((reference,i)=>{
    const game=flyGame(flights[i].test),error=game.flightDistance/reference.flightDistance-1;
    assert.ok(Math.abs(error)<.05,`${reference.case.label}: game ${game.flightDistance.toFixed(2)} m, reference ${reference.flightDistance.toFixed(2)} m`);
    const sweeps=flyGame(flights[i].test,{elasticSolve:'sweeps'});
    assert.ok(Math.abs(sweeps.flightDistance/reference.flightDistance-1)>2*Math.abs(error),`${reference.case.label}: sweeps alone are no worse`);
  });
});

test('energy is accounted for through a launch and free flight',async()=>{
  const flight=caseFor('stable'),reference=await solve({test:flight,overrides:{},options:{maxTime:.4}},undefined,false);
  // Launch work in, air work out; what is left over is integration error and contact losses.
  assert.ok(Math.abs(reference.energy.residual)<.01*reference.energy.launchWork,`residual ${reference.energy.residual} J of ${reference.energy.launchWork} J`);
  assert.ok(reference.solver.worstNewton>0&&reference.peakPenetration<2e-5&&reference.peakStrain<1e-3);
});

test('halving the time step barely moves the reference',async()=>{
  const flight=caseFor('basic'),column=(r:{columns:string[]},name:string)=>r.columns.indexOf(name);
  const [coarse,fine]=await Promise.all([32,128].map(stepsPerFrame=>solve({test:flight,overrides:{},options:{maxTime:.5,stepsPerFrame}},undefined,false)));
  const x=column(fine,'x'),a=coarse.frames[48].slice(x,x+3),b=fine.frames[48].slice(x,x+3);
  assert.ok(Math.hypot(...a.map((v,d)=>v-b[d]))<.01,`0.4 s centres differ by ${Math.hypot(...a.map((v,d)=>v-b[d]))} m between 32 and 128 steps per frame`);
});
