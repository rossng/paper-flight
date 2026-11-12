import {writeFileSync} from 'node:fs';
import {Simulation,DEFAULTS} from '../src/physics/simulation';
import {designMesh} from '../src/folding/design';
import {materialSettings} from '../src/physics/materials';

// Compare the same nominal launch impulse; aerodynamic and gravity losses
// during the spring stroke are deliberately not compensated at detachment.
const results=[];
for(const design of ['legacy','dart'])for(const material of ['paper','tissue'] as const){
 for(const launchDuration of [0,DEFAULTS.launchDuration]){
  const parameters={...DEFAULTS,...materialSettings(material),speed:7.15,pitch:15,elevation:11.8,launchDuration};
  const sim=new Simulation(parameters,design==='legacy'?undefined:designMesh(design,parameters));sim.launch();
  let firstStepAirForce=0,earlyDeformation=0;
  while(sim.status==='flying'){
   sim.step();if(sim.trail.length===2)firstStepAirForce=Math.hypot(...sim.metrics().force);
   if(sim.time<=.35+1e-12)earlyDeformation=Math.max(earlyDeformation,sim.deformation().rms);
  }
  const result={design,material,launchDuration,firstStepAirForceN:firstStepAirForce,earlyDeformationMm:earlyDeformation*1000,
   peakDeformationMm:sim.peakDeformation*1000,distanceM:sim.metrics().distance,timeS:sim.time,status:sim.status};
  results.push(result);console.log(JSON.stringify(result));
 }
}
writeFileSync('docs/launcher-benchmark-results.json',JSON.stringify({
 settings:{speed:7.15,pitch:15,elevation:11.8,height:DEFAULTS.height,earlyWindowS:.35,wind:DEFAULTS.wind},
 note:'Nominal speed/impulse matched; time and distance include the spring stroke. Illustrative simulation, not measured throws.',results,
},null,2)+'\n');
