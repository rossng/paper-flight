import { createDart, hingeAngle, type PaperMesh } from '../geometry/dart';
import { aerodynamicLoads, type PanelLoad, type AeroSettings } from './aerodynamics';
import { airVelocity } from './weather';
import type { MaterialId } from './materials';
import { dampEdges, solveEdge, solveHinge, solveLayerTie } from './constraints';
import { cross, dot, length, radians, sub, unit, wrapAngle, type V3 } from './math';
export const FIXED_DT=1/120;
export const SUBSTEPS=3;
export interface Parameters { speed:number; pitch:number; heading:number; elevation:number; span:number; size:number; paperWeight:number; stiffness:number; crease:number; dihedral:number; height:number; gravity:number; density:number; wind:V3; windHeading:number; gusts:number; turbulence:number; material:MaterialId; aero:boolean }
export const DEFAULTS: Parameters={speed:6,pitch:8,heading:0,elevation:0,span:1,size:1,paperWeight:80,stiffness:1,crease:1,dihedral:8,height:1.8,gravity:9.81,density:1.225,wind:[0,0,0],windHeading:0,gusts:0,turbulence:0,material:'paper',aero:true};
export type Status='ready'|'flying'|'landed'|'finished'|'error';
export interface Metrics { time:number; speed:number; altitude:number; distance:number; pathLength:number; deformation:number; bend:number; maxStrain:number; force:V3; stallFraction:number; center:V3 }
export class Simulation {
  mesh:PaperMesh; positions:V3[]=[]; velocities:V3[]=[]; forces:V3[]=[]; invMass:number[];
  panels:PanelLoad[]=[]; parameters:Parameters; status:Status='ready'; time=0; pathLength=0;
  start:V3=[0,0,0]; trail:V3[]=[]; peakDeformation=0; lastCenter:V3=[0,0,0];
  constructor(parameters:Partial<Parameters>={},mesh?:PaperMesh) {
    this.parameters={...DEFAULTS,...parameters,wind:[...(parameters.wind??DEFAULTS.wind)]};
    this.mesh=mesh?structuredClone(mesh):createDart(this.parameters.dihedral,8,5,this.parameters.span,this.parameters.size,0.005*this.parameters.size**2*this.parameters.paperWeight/80);this.invMass=this.mesh.masses.map(m=>1/m);this.reset();
  }
  center(array=this.positions):V3 {
    const c:V3=[0,0,0],mass=this.mesh.masses.reduce((a,b)=>a+b,0);
    for(let i=0;i<array.length;i++)for(let d=0;d<3;d++)c[d]+=array[i][d]*this.mesh.masses[i]/mass;
    return c;
  }
  reset() {
    const angle=radians(this.parameters.pitch),c=Math.cos(angle),s=Math.sin(angle);
    const yaw=radians(this.parameters.heading),cy=Math.cos(yaw),sy=Math.sin(yaw);
    this.positions=this.mesh.rest.map(([x,y,z])=>{const px=c*x-s*y;return [cy*px-sy*z,s*x+c*y,sy*px+cy*z];});
    const center=this.center();
    for(const p of this.positions){p[0]-=center[0];p[1]+=this.parameters.height-center[1];p[2]-=center[2];}
    this.velocities=this.positions.map(()=>[0,0,0]);this.forces=this.positions.map(()=>[0,0,0]);
    this.status='ready';this.time=0;this.pathLength=0;this.peakDeformation=0;this.panels=[];
    this.start=this.center();this.lastCenter=[...this.start];this.trail=[[...this.start]];
  }
  launch(){this.reset();const yaw=radians(this.parameters.heading),e=radians(this.parameters.elevation),v=this.parameters.speed;this.velocities=this.positions.map(()=>[v*Math.cos(e)*Math.cos(yaw),v*Math.sin(e),v*Math.cos(e)*Math.sin(yaw)]);this.status='flying';}
  aeroSettings(time=this.time):AeroSettings {
    return {enabled:this.parameters.aero,density:this.parameters.density,wind:this.parameters.wind,areaWeights:this.mesh.aeroWeights,
      windAt:this.parameters.gusts||this.parameters.turbulence?position=>airVelocity(this.parameters,time,position):undefined};
  }
  step() {
    if(this.status!=='flying')return;
    // Very light sheets accelerate much faster under panel loads. Resolve them
    // with smaller substeps while keeping the recorded 120 Hz clock unchanged.
    const substeps=this.parameters.paperWeight<30?SUBSTEPS*3:SUBSTEPS,dt=FIXED_DT/substeps;
    for(let substep=0;substep<substeps;substep++){
      const old=this.positions.map(p=>[...p] as V3);
      for(let i=0;i<this.forces.length;i++){this.forces[i][0]=0;this.forces[i][1]=-this.parameters.gravity*this.mesh.masses[i];this.forces[i][2]=0;}
      this.panels=aerodynamicLoads(this.positions,this.velocities,this.mesh.faces,this.aeroSettings(),this.forces);
      for(let i=0;i<this.positions.length;i++)for(let d=0;d<3;d++){
        this.velocities[i][d]+=dt*this.forces[i][d]*this.invMass[i];
        this.positions[i][d]+=dt*this.velocities[i][d];
      }
      for(const e of this.mesh.edges)e.lambda=0;
      for(const h of this.mesh.hinges)h.lambda=0;
      for(let iteration=0;iteration<24;iteration++){
        // Alternating sweep direction reduces directional Gauss-Seidel bias.
        const forward=iteration%2===0;
        for(let j=0;j<this.mesh.edges.length;j++)solveEdge(this.positions,this.invMass,this.mesh.edges[forward?j:this.mesh.edges.length-1-j],2e-8,dt);
        for(let j=0;j<this.mesh.hinges.length;j++){
          const h=this.mesh.hinges[forward?j:this.mesh.hinges.length-1-j];
          // Effective bending rigidity D; crease stiffness is an independent torsional spring.
          const k=(h.crease?0.0025*this.parameters.crease:0.00035*this.parameters.stiffness)*h.weight*(h.layerStiffness??1);
          solveHinge(this.positions,this.invMass,h,1/Math.max(k,1e-8),dt);
        }
        for(const tie of this.mesh.layerTies??[])solveLayerTie(this.positions,this.invMass,tie);
      }
      for(let i=0;i<this.positions.length;i++)for(let d=0;d<3;d++)this.velocities[i][d]=(this.positions[i][d]-old[i][d])/dt;
      dampEdges(this.positions,this.velocities,this.invMass,this.mesh.edges,1-Math.exp(-18*dt));
      this.time+=dt;
      if(this.positions.some(p=>p.some(v=>!Number.isFinite(v)||Math.abs(v)>1000))){this.positions=old;this.velocities=old.map(()=>[0,0,0]);this.status='error';return;}
      const center=this.center();this.pathLength+=length(sub(center,this.lastCenter));this.lastCenter=center;
      if(this.positions.some(p=>p[1]<=0)){this.status='landed';break;}
      if(this.time>=12){this.status='finished';break;}
    }
    this.trail.push(this.center());
    this.peakDeformation=Math.max(this.peakDeformation,this.deformation().rms);
  }
  /** Remove translation and body rotation before measuring shape change, in metres. */
  deformation(positions=this.positions){
    const frame=(p:V3[])=>{
      const origin=p[this.mesh.tail],x=unit(sub(p[this.mesh.nose],origin));
      const rawZ=sub(p[this.mesh.rightRoot],p[this.mesh.leftRoot]);
      const y=unit(cross(rawZ,x)),z=unit(cross(x,y));return {x,y,z,origin};
    };
    const rest=frame(this.mesh.rest),current=frame(positions);
    let sum=0;
    const vectors:V3[]=[],reference:V3[]=[];
    const perVertex=positions.map((p,i)=>{
      const a=sub(p,current.origin),b=sub(this.mesh.rest[i],rest.origin);
      const ideal:V3=[...current.origin];for(let k=0;k<3;k++)ideal[k]+=dot(b,rest.x)*current.x[k]+dot(b,rest.y)*current.y[k]+dot(b,rest.z)*current.z[k];reference.push(ideal);vectors.push(sub(p,ideal));
      const d=Math.hypot(dot(a,current.x)-dot(b,rest.x),dot(a,current.y)-dot(b,rest.y),dot(a,current.z)-dot(b,rest.z));sum+=d*d;return d;
    });
    return {rms:Math.sqrt(sum/positions.length),perVertex,vectors,reference};
  }
  metrics():Metrics {
    const center=this.center(),velocity=this.center(this.velocities),force:V3=[0,0,0];let area=0,stall=0,bend=0;
    for(const p of this.panels){for(let d=0;d<3;d++)force[d]+=p.force[d];area+=p.area;if(p.stalled)stall+=p.area;}
    let count=0;
    for(const h of this.mesh.hinges)if(!h.crease){const angle=hingeAngle(...h.ids.map(i=>this.positions[i]) as [V3,V3,V3,V3]);bend+=wrapAngle(angle-h.rest)**2;count++;}
    let maxStrain=0;for(const e of this.mesh.edges)maxStrain=Math.max(maxStrain,Math.abs(length(sub(this.positions[e.a],this.positions[e.b]))/e.rest-1));
    return {time:this.time,speed:length(sub(velocity,airVelocity(this.parameters,this.time,center))),altitude:center[1],distance:Math.hypot(center[0]-this.start[0],center[2]-this.start[2]),pathLength:this.pathLength,deformation:this.deformation().rms,bend:Math.sqrt(bend/Math.max(count,1))*180/Math.PI,maxStrain,force,stallFraction:area?stall/area:0,center};
  }
}
/** Rendering can slow down under load, but never takes a larger physical timestep. */
export class FixedClock {
  accumulator=0; droppedTime=0;
  advance(elapsed:number,step:()=>void){
    const accepted=Math.min(Math.max(elapsed,0),0.1);this.droppedTime+=Math.max(0,elapsed-accepted);this.accumulator+=accepted;
    let steps=0;while(this.accumulator>=FIXED_DT-1e-12 && steps<12){step();this.accumulator-=FIXED_DT;steps++;}return steps;
  }
  reset(){this.accumulator=0;this.droppedTime=0;}
}
