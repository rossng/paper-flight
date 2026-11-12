/** Serial, warmed solver benchmark. --source also runs this harness against an
 * archived baseline without checking out or changing another worktree. */
import { writeFileSync } from 'node:fs';
import { gzipSync } from 'node:zlib';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { cpus, platform, arch, totalmem, loadavg } from 'node:os';
import { execFileSync } from 'node:child_process';
import { Session } from 'node:inspector';
import type { Simulation as SimulationType, Parameters } from '../src/physics/simulation';

const option=(name:string,fallback:string)=>process.argv.includes(name)?process.argv[process.argv.indexOf(name)+1]:fallback;
const source=resolve(option('--source','.'));
const sources=[source,...(process.argv.includes('--compare-source')?[resolve(option('--compare-source','.'))]:[])];
const implementations=await Promise.all(sources.map(async(root,index)=>{
 const moduleAt=async(path:string)=>import(pathToFileURL(resolve(root,path)).href);
 return {label:sources.length===1?'current':index===0?'baseline':'candidate',root,
  ...await moduleAt('src/physics/simulation.ts'),...await moduleAt('src/folding/design.ts'),
  ...await moduleAt('src/physics/materials.ts'),...await moduleAt('src/rendering/flight-recording.ts'),
  ...await moduleAt('src/physics/layer-contact.ts')};
}));
const runs=Number(option('--runs','5')),steps=Number(option('--steps','48')),warmups=Number(option('--warmups','2'));
const stats=(values:number[])=>{const sorted=[...values].sort((a,b)=>a-b);return {median:sorted[Math.floor(sorted.length*.5)],p95:sorted[Math.min(sorted.length-1,Math.floor(sorted.length*.95))],max:sorted.at(-1),mean:values.reduce((a,b)=>a+b,0)/values.length};};
const cases=['legacy-paper','basic-paper','dart-paper','stable-paper','legacy-tissue','dart-tissue',...(option('--suite','default')==='contact'?['basic-tissue','stable-tissue']:[])];
const selected=option('--case','all');
const report:any={context:{date:new Date().toISOString(),source,sources,order:'alternating baseline/candidate within each repeat, serial',baseline:option('--baseline','26cc85fe47f0d5283e311af8a4e7ef657a6abebe'),revision:execFileSync('git',['rev-parse','HEAD']).toString().trim(),node:process.version,platform:platform(),arch:arch(),cpu:cpus()[0]?.model,logicalCPUs:cpus().length,memoryBytes:totalmem(),loadBefore:loadavg(),runs,steps,warmups,workers:1},cases:[]};
const profiler=new Session();
const post=(method:string)=>new Promise<any>((resolve,reject)=>profiler.post(method,(error,result)=>error?reject(error):resolve(result)));
const profile=process.argv.includes('--profile');
if(profile){profiler.connect();await post('Profiler.enable');await post('Profiler.start');}
for(const name of cases.filter(c=>selected==='all'||c===selected)){
 const [design,material]=name.split('-');
 const entries=implementations.map(implementation=>{
  const {DEFAULTS,materialSettings,designMesh}=implementation;
  const parameters:Parameters={...DEFAULTS,...materialSettings(material),...(process.argv.includes('--launch-duration')?{launchDuration:Number(option('--launch-duration','0'))}:{}),...((material==='tissue'||option('--load','default')==='high')?{speed:11,wind:[0,0,3],gusts:1.5,turbulence:1}:{})};
  return {implementation,parameters,mesh:design==='legacy'?undefined:designMesh(design,parameters),
   timings:[] as number[],totals:[] as number[],substeps:[] as number[],metricsMs:[] as number[],deformationMs:[] as number[],recordingMs:[] as number[],snapshots:[] as any[],metadata:undefined as any};
 });
 for(let run=-warmups;run<runs;run++)for(const entry of run%2===0?entries:[...entries].reverse()){
  const {implementation,parameters,mesh,timings,totals,substeps,metricsMs,deformationMs,recordingMs}=entry;
  const {Simulation,FlightRecording,detectLayerContacts}=implementation;
  const sim:SimulationType=new Simulation(parameters,mesh),recording=new FlightRecording();sim.launch();
  let inner=0;const aeroSettings=sim.aeroSettings.bind(sim);sim.aeroSettings=(time?:number)=>{inner++;return aeroSettings(time);};
  let total=0;const samples:any[]=[];
  for(let step=0;step<steps&&sim.active;step++){
   inner=0;const start=performance.now();sim.step();const ms=performance.now()-start;total+=ms;
   if(run>=0){timings.push(ms);substeps.push(inner);}
   // Diagnostics and recording timed separately from the solver.
   let t=performance.now();const metrics=sim.metrics();if(run>=0)metricsMs.push(performance.now()-t);
   t=performance.now();sim.deformation();if(run>=0)deformationMs.push(performance.now()-t);
   t=performance.now();recording.capture(sim);if(run>=0)recordingMs.push(performance.now()-t);
   if(run===0&&((step+1)%12===0||step===steps-1||!sim.active)){
    const candidates=structuredClone(sim.contactCandidates);
    const contacts=detectLayerContacts(sim.positions,sim.positions,candidates);
    samples.push({step:step+1,status:sim.status,substeps:inner,metrics,peakDeformation:sim.peakDeformation,selfContact:sim.selfContact?{stats:{...sim.selfContact.stats},contacts:structuredClone(sim.selfContact.contacts)}:undefined,positions:sim.positions.map(p=>[...p]),velocities:sim.velocities.map(p=>[...p]),forces:sim.forces.map(p=>[...p]),plastic:sim.mesh.hinges.map(h=>h.plasticOffset??0),contactCount:contacts.length,contactSigns:candidates.map((c:any)=>c.sign)});
   }
  }
  if(run>=0)totals.push(total);
  if(run===0){entry.snapshots=samples;entry.metadata={vertices:sim.positions.length,faces:sim.mesh.faces.length,hinges:sim.mesh.hinges.length,edges:sim.mesh.edges.length,patches:sim.surface.patches.length,candidates:sim.contactCandidates.length,iterations:sim.parameters.iterations,simulatedSeconds:sim.time,status:sim.status};}
 }
 for(const {implementation,parameters,metadata,timings,totals,substeps,metricsMs,deformationMs,recordingMs,snapshots} of entries){
 const result={name,variant:implementation.label,parameters,...metadata,stepMs:stats(timings),runMs:totals,runStatsMs:stats(totals),realTimeRatio:metadata.simulatedSeconds*1000/stats(totals).median,substeps:stats(substeps),metricsMs:stats(metricsMs),deformationMs:stats(deformationMs),recordingMs:stats(recordingMs),snapshots};
 report.cases.push(result);console.log(JSON.stringify({...result,snapshots:undefined}));
 }
}
report.context.loadAfter=loadavg();
const output=option('--output','/private/tmp/physics-performance.json');writeFileSync(output+'.states.json.gz',gzipSync(JSON.stringify(report.cases.map((c:any)=>({name:c.name,variant:c.variant,snapshots:c.snapshots})))));
writeFileSync(output,JSON.stringify({...report,cases:report.cases.map((c:any)=>({...c,snapshots:undefined}))},null,2)+'\n');
if(profile){const {profile}=await post('Profiler.stop');writeFileSync(output+'.cpuprofile',JSON.stringify(profile));profiler.disconnect();}
