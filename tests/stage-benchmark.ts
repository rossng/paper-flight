/** Where the flight solver's time goes, by stage, over complete default-launch flights:
 * milliseconds of solver time per simulated second, timed inside the solver (the host
 * clock is read at stage boundaries, which adds a few per cent). "Outside the solver" is
 * what `Simulation.step` spends around it, copying state in and out.
 * Usage: npm run benchmark:stages -- [design ...] */
import { Simulation, DEFAULTS } from '../src/physics/simulation';
import { STAGES, setProfiling } from '../src/physics/solver';
import { designMesh } from '../src/folding/design';
import { designs } from '../src/folding/library';

const only=process.argv.slice(2);
const GROUPS=['air loads','elastic solve','sweeps','layer contact','self-contact','ground','other'] as const;
const group:(typeof GROUPS[number])[]=['other','air loads','elastic solve','sweeps','layer contact','layer contact','self-contact','self-contact','ground','sweeps','other'];
if(group.length!==STAGES.length)throw Error('Stage map out of date');
const fly=(id:string)=>{
  const sim=new Simulation(DEFAULTS,id==='legacy'?undefined:designMesh(id,DEFAULTS));sim.launch();
  let wall=0,times:number[]=[];
  while(sim.active){const start=performance.now();sim.step();wall+=performance.now()-start;times=sim.stageTimes??times;}
  return {sim,wall,times};
};
for(const design of designs){
  if(only.length&&!only.includes(design.id))continue;
  fly(design.id); // Warm up; flights are deterministic.
  setProfiling(true);const {sim,wall,times}=fly(design.id);setProfiling(false);
  const totals=Object.fromEntries(GROUPS.map(name=>[name,0])) as Record<typeof GROUPS[number],number>;
  times.forEach((ms,stage)=>{totals[group[stage]]+=ms;});
  const inside=times.reduce((sum,ms)=>sum+ms,0),per=(ms:number)=>Number((ms/sim.time).toFixed(1));
  console.log(JSON.stringify({design:design.id,simulatedSeconds:Number(sim.time.toFixed(2)),msPerSimulatedSecond:{...Object.fromEntries(GROUPS.map(name=>[name,per(totals[name])])),
    'outside the solver':per(wall-inside),total:per(wall)}}));
}
