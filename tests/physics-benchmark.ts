import { Simulation } from '../src/physics/simulation';
import { solveLiftingLine, buildLiftingSurface } from '../src/physics/lifting-surface';
import { aerodynamicLoads } from '../src/physics/aerodynamics';
import { createDart } from '../src/geometry/dart';
import { add, length, sub, type V3 } from '../src/physics/math';
import { designMesh } from '../src/folding/design';
import { DEFAULTS } from '../src/physics/simulation';
import { rectangle } from './fixtures/lifting-wing';

console.log('Elliptic wing, span 2 m, root chord .4 m, 8 m/s, incidence .05 rad; incompressible lifting-line reference.');
for(const n of [8,16,32,64]){
 const b=2,c0=.4,V=8,alpha=.05,bounds=Array.from({length:n+1},(_,i)=>-b/2+b*(1-Math.cos(Math.PI*i/n))/2);
 const chords=Array.from({length:n},(_,i)=>c0*Math.sqrt(1-((bounds[i]+bounds[i+1])/b)**2));
 const r=solveLiftingLine(bounds,chords,chords.map(()=>V),chords.map(()=>alpha));let area=0,lift=0,drag=0;
 for(let i=0;i<n;i++){const dz=bounds[i+1]-bounds[i];area+=chords[i]*dz;lift+=V*r.circulation[i]*dz;drag+=r.circulation[i]*r.downwash[i]*dz;}
 const ar=b*b/area,cl=2*lift/(V*V*area),cd=2*drag/(V*V*area),exact=2*Math.PI*alpha/(1+2/ar);
 console.log(JSON.stringify({stations:n,cl,cd,analyticCL:exact,liftErrorPercent:100*(cl/exact-1),inducedDragErrorPercent:100*(cd/(cl*cl/(Math.PI*ar))-1)}));
}
console.log('Fixed rectangular surface, 12 aerodynamic stations; structural tessellation and baseline comparison.');
for(const segments of [6,12,24,48])for(const model of ['panels','coupled'] as const){
 const mesh=rectangle(1,.2,segments),force=mesh.rest.map(()=>[0,0,0] as V3);
 const panels=aerodynamicLoads(mesh.rest,mesh.rest.map(()=>[8,-.6,0]),mesh.faces,{density:1.225,enabled:true,wind:[0,0,0],surface:buildLiftingSurface(mesh),model},force);
 console.log(JSON.stringify({segments,model,force:panels.map(p=>p.force).reduce(add,[0,0,0])}));
}
console.log('Matched .4-second flexible flight, original dart: temporal/iteration resolution, then mesh resolution.');
const runs:{substeps:number;iterations:number;rows:number;columns:number}[]=[
 {substeps:0,iterations:24,rows:8,columns:5},{substeps:3,iterations:24,rows:8,columns:5},{substeps:6,iterations:24,rows:8,columns:5},
 {substeps:9,iterations:24,rows:8,columns:5},{substeps:9,iterations:48,rows:8,columns:5},
 {substeps:9,iterations:48,rows:12,columns:7},{substeps:9,iterations:48,rows:16,columns:9},
];
let previous:V3|undefined;
for(const run of runs){
 const sim=new Simulation(run,createDart(8,run.rows,run.columns));sim.launch();const start=performance.now();let maxStrain=0;
 for(let i=0;i<48;i++){sim.step();maxStrain=Math.max(maxStrain,sim.metrics().maxStrain);}
 const m=sim.metrics();console.log(JSON.stringify({...run,center:m.center,deformationMm:1000*m.deformation,maxStrain,centerDeltaFromPrevious:previous?length(sub(m.center,previous)):null,msPerStep:(performance.now()-start)/48}));previous=m.center;
}
console.log('Supported folded designs, matched .4-second flight, contact vs legacy bonded layers.');
for(const id of ['basic','dart','stable'])for(const layerModel of ['contact','bonded'] as const){
 const sim=new Simulation({layerModel},designMesh(id,DEFAULTS));sim.launch();const start=performance.now();let maxStrain=0;
 for(let i=0;i<48;i++){sim.step();maxStrain=Math.max(maxStrain,sim.metrics().maxStrain);}
 console.log(JSON.stringify({id,layerModel,center:sim.center(),deformationMm:1000*sim.deformation().rms,maxStrain,msPerStep:(performance.now()-start)/48}));
}
