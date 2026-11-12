import { Simulation } from '../src/physics/simulation';
const now=()=>performance.now();
for(const stiffness of [0.2,1,5]){
 const simulation=new Simulation({stiffness});simulation.launch();let steps=0,maxStrain=0,sumMs=0;const start=now();
 while(simulation.active){
   const t=now();simulation.step();sumMs+=now()-t;steps++;
   maxStrain=Math.max(maxStrain,simulation.metrics().maxStrain);
 }
 const metrics=simulation.metrics();
 console.log(JSON.stringify({stiffness,status:simulation.status,flightSeconds:metrics.time,rangeMetres:metrics.distance,peakDeformationMm:simulation.peakDeformation*1000,maxEdgeStrainPercent:100*maxStrain,meanPhysicsMsPerStep:sumMs/steps,wallSeconds:(now()-start)/1000},null,2));
}
