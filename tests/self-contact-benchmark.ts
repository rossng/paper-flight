import { Simulation, DEFAULTS } from '../src/physics/simulation';
import { materialSettings } from '../src/physics/materials';
import { designMesh } from '../src/folding/design';
import { type V3 } from '../src/physics/math';
import { intersections } from './fixtures/intersections';
import { pathToFileURL } from 'node:url';

// Deliberately serial. Simulation timing excludes the independent intersection
// diagnostic and mesh compilation. Both baseline and changed solver use this file.
const results=[];
// Optional baseline Simulation module, with its relative imports resolved to the
// unchanged support modules. Lets both implementations run on the same machine.
const solvers:[string,typeof Simulation][]=[['current',Simulation]];
if(process.argv[2])solvers.unshift(['baseline',(await import(pathToFileURL(process.argv[2]).href)).Simulation]);
for(const material of ['paper','tissue'] as const)for(const design of ['legacy','basic','dart','stable']){
 for(const [solver,Constructor] of solvers){
  const parameters={...DEFAULTS,...materialSettings(material),speed:11,wind:[0,0,3] as V3,gusts:1.5,turbulence:1};
  const sim=new Constructor(parameters,design==='legacy'?undefined:designMesh(design,parameters));sim.launch();
  let milliseconds=0,peakPairs=0,peakDepth=0,firstIntersection:number|null=null,peakStrain=0;
  const steps=42;
  for(let step=0;step<steps;step++){
    const start=performance.now();sim.step();milliseconds+=performance.now()-start;
    const hit=intersections(sim.positions,sim.mesh.faces);
    if(hit.pairs&&firstIntersection===null)firstIntersection=step+1;
    peakPairs=Math.max(peakPairs,hit.pairs);peakDepth=Math.max(peakDepth,hit.depth);peakStrain=Math.max(peakStrain,sim.metrics().maxStrain);
  }
  results.push({solver,material,design,vertices:sim.positions.length,faces:sim.mesh.faces.length,steps,time:sim.time,status:sim.status,firstIntersection,peakPairs,peakDepthMm:1000*peakDepth,peakStrain,meanStepMs:milliseconds/steps,flexMm:1000*sim.deformation().rms});
 }
}
console.log(JSON.stringify(results,null,2));
