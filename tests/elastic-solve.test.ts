import test from 'node:test';
import assert from 'node:assert/strict';
import { ElasticSolver } from '../src/physics/elastic-solve';
import { Simulation } from '../src/physics/simulation';
import { buildMembranes, membraneBatch, solveMembranes } from '../src/physics/membrane';
import { hingeBatch, setHingeOffset, solveHinges } from '../src/physics/constraints';
import { hingeStiffness, prepareMaterialGeometry, sheetMaterial } from '../src/physics/sheet-material';
import { createDart } from '../src/geometry/dart';
import { length, sub } from '../src/physics/math';

/** A dart with its batches, a bent and stretched prediction, and compliances for step dt. */
function bentDart(dt:number){
  const mesh=createDart(),material=sheetMaterial('paper',80);prepareMaterialGeometry(mesh);
  const membranes=buildMembranes(mesh),batch=membraneBatch(membranes),hinges=hingeBatch(mesh.hinges);
  membranes.forEach((t,j)=>{const scale=material.thickness*t.area*dt*dt;batch.alpha[3*j]=1/(material.youngMD*scale);batch.alpha[3*j+1]=1/(material.youngCD*scale);batch.alpha[3*j+2]=1/(material.shear*scale);});
  mesh.hinges.forEach((h,j)=>{hinges.alpha[j]=1/hingeStiffness(material,h)/(dt*dt);setHingeOffset(hinges,j,0);});
  const x=Float64Array.from(mesh.rest.flatMap(([px,py,pz])=>[px*1.0005,py+.4*pz*pz+.002*Math.sin(40*px),pz]));
  return {mesh,batch,hinges,x};
}

test('direct elastic solve leaves nothing for the constraint sweeps to do',()=>{
  const dt=1/360,{mesh,batch,hinges,x}=bentDart(dt),solver=new ElasticSolver(mesh.rest.length,mesh.faces,mesh.hinges);
  assert.equal(solver.solve(x,mesh.masses,dt,batch,hinges,50,1e-12),true);
  const solved=x.slice(),w=mesh.masses.map(m=>1/m);
  for(let sweep=0;sweep<4;sweep++){solveMembranes(x,w,batch,sweep%2===0);solveHinges(x,w,hinges,sweep%2===0);}
  let moved=0;for(let k=0;k<x.length;k++)moved=Math.max(moved,Math.abs(x[k]-solved[k]));
  assert.ok(moved<1e-11,`sweeps moved a solved sheet by ${moved} m`);
});

test('direct elastic solve reaches the state many sweeps only approach',()=>{
  const dt=1/360,direct=bentDart(dt),swept=bentDart(dt),predicted=direct.x.slice(),w=direct.mesh.masses.map(m=>1/m);
  new ElasticSolver(direct.mesh.rest.length,direct.mesh.faces,direct.mesh.hinges).solve(direct.x,direct.mesh.masses,dt,direct.batch,direct.hinges,50,1e-12);
  const distance=(sweeps:number)=>{
    const x=predicted.slice();swept.batch.lambda.fill(0);swept.hinges.lambda.fill(0);
    for(let sweep=0;sweep<sweeps;sweep++){solveMembranes(x,w,swept.batch,sweep%2===0);solveHinges(x,w,swept.hinges,sweep%2===0);}
    let sum=0;for(let k=0;k<x.length;k++)sum+=(x[k]-direct.x[k])**2;return Math.sqrt(sum/x.length);
  };
  const few=distance(24),many=distance(2000);
  assert.ok(many<.25*few,`RMS distance to the direct solution: ${few} m after 24 sweeps, ${many} m after 2000`);
});

test('a solve that cannot converge hands back the prediction',()=>{
  const dt=1/360,{mesh,batch,hinges,x}=bentDart(dt),predicted=x.slice();
  assert.equal(new ElasticSolver(mesh.rest.length,mesh.faces,mesh.hinges).solve(x,mesh.masses,dt,batch,hinges,1,1e-14),false);
  assert.deepEqual(x,predicted);assert.ok(batch.lambda.every(v=>v===0)&&hinges.lambda.every(v=>v===0));
});

test('default flight agrees with the sweep solver taken to many substeps and sweeps',()=>{
  const fly=(parameters:object)=>{const sim=new Simulation(parameters);sim.launch();for(let i=0;i<36;i++)sim.step();return sim.center();};
  const direct=fly({}),sweeps=fly({elasticSolve:'sweeps'}),converged=fly({elasticSolve:'sweeps',substeps:48,iterations:100});
  assert.ok(length(sub(direct,converged))<.02,`direct solve is ${length(sub(direct,converged))} m from the converged sweeps at 0.3 s`);
  assert.ok(length(sub(sweeps,converged))>2*length(sub(direct,converged)),'and closer than default sweeps alone');
});
