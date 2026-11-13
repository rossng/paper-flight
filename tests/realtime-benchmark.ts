/** Complete default-launch flights for every design: simulated seconds per
 * wall-clock second of solver time, split at touchdown. A ratio below one means
 * the solver alone cannot keep pace with real time on this machine.
 * Usage: npm run benchmark:realtime -- [design] [--material tissue] */
import { Simulation, DEFAULTS, FIXED_DT } from '../src/physics/simulation';
import { designMesh } from '../src/folding/design';
import { materialSettings, type MaterialId } from '../src/physics/materials';
import { designs } from '../src/folding/library';

const args=process.argv.slice(2),flag=args.indexOf('--material');
const material=(flag>=0?args[flag+1]:'paper') as MaterialId,only=args.find((arg,i)=>!arg.startsWith('--')&&i!==flag+1);
const ratio=(steps:number,ms:number)=>ms?Number((steps*FIXED_DT*1000/ms).toFixed(2)):null;
for(const design of designs){
  if(only&&design.id!==only)continue;
  const parameters={...DEFAULTS,...materialSettings(material)};
  const run=()=>{
    const sim=new Simulation(parameters,design.id==='legacy'?undefined:designMesh(design.id,parameters));sim.launch();
    const phase={air:{steps:0,ms:0},ground:{steps:0,ms:0}};let worst=0;
    while(sim.active){
      const bucket=sim.touchdown?phase.ground:phase.air,start=performance.now();sim.step();
      const ms=performance.now()-start;bucket.steps++;bucket.ms+=ms;worst=Math.max(worst,ms);
    }
    return {sim,phase,worst};
  };
  run(); // Warm the JIT; flights are deterministic.
  const {sim,phase,worst}=run(),metrics=sim.metrics();
  console.log(JSON.stringify({design:design.id,material,vertices:sim.positions.length,faces:sim.mesh.faces.length,status:sim.status,
    simulatedSeconds:Number(sim.time.toFixed(2)),realTimeRatio:ratio(phase.air.steps+phase.ground.steps,phase.air.ms+phase.ground.ms),
    airRatio:ratio(phase.air.steps,phase.air.ms),groundRatio:ratio(phase.ground.steps,phase.ground.ms),worstStepMs:Number(worst.toFixed(1)),
    flightMetres:Number(metrics.flightDistance.toFixed(2)),peakDeformationMm:Number((sim.peakDeformation*1000).toFixed(1))}));
}
