import { buildMembranes, solveMembrane, membraneAlphas, type Membrane } from './membrane';
import { buildLiftingSurface, type LiftingSurface } from './lifting-surface';
import { buildContactCandidates, detectLayerContacts, solveLayerContact, type ContactCandidate } from './layer-contact';
import { SelfContactSolver } from './self-contact';
import { sheetMaterial, prepareMaterialGeometry, hingeStiffness, updateCreasePlasticity, type SheetMaterial } from './sheet-material';
import { createDart, hingeAngle, type PaperMesh } from '../geometry/dart';
import { aerodynamicLoads, type PanelLoad, type AeroSettings } from './aerodynamics';
import { airVelocity } from './weather';
import { materialSettings, type MaterialId } from './materials';
import { RELEASE_DURATION_MS, springVelocityDelta } from './launcher';
import { dampEdges, limitEdgeStrain, solveHinge, solveLayerTie } from './constraints';
import { cross, dot, length, radians, sub, unit, wrapAngle, type V3 } from './math';
export const FIXED_DT=1/120;
export const SUBSTEPS=3;
/** Leave room for rendering/input; an indivisible substep can exceed this budget. */
export const FRAME_PHYSICS_BUDGET_MS=12;
export interface Parameters { aeroModel:'coupled'|'panels'; layerModel:'contact'|'bonded'; aeroStations:number; substeps:number; iterations:number; speed:number; launchDuration:number; pitch:number; heading:number; elevation:number; span:number; size:number; paperWeight:number; stiffness:number; crease:number; dihedral:number; height:number; gravity:number; density:number; wind:V3; windHeading:number; gusts:number; turbulence:number; material:MaterialId; aero:boolean }
export const DEFAULTS: Parameters={aeroModel:'coupled',layerModel:'contact',aeroStations:12,substeps:0,iterations:24,speed:6,launchDuration:RELEASE_DURATION_MS/1000,pitch:8,heading:0,elevation:0,span:1,size:1,paperWeight:80,stiffness:1,crease:1,dihedral:8,height:1.8,gravity:9.81,density:1.225,wind:[0,0,0],windHeading:0,gusts:0,turbulence:0,material:'paper',aero:true};
export type Status='ready'|'flying'|'landed'|'finished'|'error';
export interface Metrics { time:number; speed:number; altitude:number; distance:number; pathLength:number; deformation:number; bend:number; maxStrain:number; force:V3; stallFraction:number; center:V3 }
export class Simulation {
  tributaryArea:number[]; membranes:Membrane[];
  surface:LiftingSurface; material:SheetMaterial; contactCandidates:ContactCandidate[];
  selfContact:SelfContactSolver;
  mesh:PaperMesh; positions:V3[]=[]; velocities:V3[]=[]; forces:V3[]=[]; invMass:number[];
  panels:PanelLoad[]=[]; parameters:Parameters; status:Status='ready'; time=0; pathLength=0;
  start:V3=[0,0,0]; trail:V3[]=[]; peakDeformation=0; lastCenter:V3=[0,0,0];
  private pendingStep?:Generator<void,void,void>;
  private oldPositions:V3[]=[];
  constructor(parameters:Partial<Parameters>={},mesh?:PaperMesh) {
    this.parameters={...DEFAULTS,...materialSettings(parameters.material??'paper',parameters.paperWeight??80),...parameters,wind:[...(parameters.wind??DEFAULTS.wind)]};
    this.mesh=mesh?structuredClone(mesh):createDart(this.parameters.dihedral,8,5,this.parameters.span,this.parameters.size,0.005*this.parameters.size**2*this.parameters.paperWeight/80);this.invMass=this.mesh.masses.map(m=>1/m);
    this.tributaryArea=this.mesh.rest.map(()=>0);
    for(const [a,b,c] of this.mesh.faces){const area=length(cross(sub(this.mesh.rest[b],this.mesh.rest[a]),sub(this.mesh.rest[c],this.mesh.rest[a])))/6;for(const i of [a,b,c])this.tributaryArea[i]+=area;}
    this.membranes=buildMembranes(this.mesh);
    this.material=sheetMaterial(this.parameters.material,this.parameters.paperWeight);prepareMaterialGeometry(this.mesh);
    this.surface=buildLiftingSurface(this.mesh,this.parameters.aeroStations,Math.max(.0002,2*this.material.thickness));
    this.contactCandidates=buildContactCandidates(this.mesh);this.selfContact=new SelfContactSolver(this.mesh);this.reset();
  }
  center(array=this.positions):V3 {
    const c:V3=[0,0,0],mass=this.mesh.masses.reduce((a,b)=>a+b,0);
    for(let i=0;i<array.length;i++)for(let d=0;d<3;d++)c[d]+=array[i][d]*this.mesh.masses[i]/mass;
    return c;
  }
  reset() {
    this.pendingStep=undefined;
    this.selfContact.reset();
    for(const c of this.contactCandidates){c.sideChosen=false;if(c.sideState)c.sideState.side=undefined;}
    for(const h of this.mesh.hinges){h.plasticOffset=0;h.plasticAccumulated=0;h.lambda=0;}
    const angle=radians(this.parameters.pitch),c=Math.cos(angle),s=Math.sin(angle);
    const yaw=radians(this.parameters.heading),cy=Math.cos(yaw),sy=Math.sin(yaw);
    this.positions=this.mesh.rest.map(([x,y,z])=>{const px=c*x-s*y;return [cy*px-sy*z,s*x+c*y,sy*px+cy*z];});
    const center=this.center();
    for(const p of this.positions){p[0]-=center[0];p[1]+=this.parameters.height-center[1];p[2]-=center[2];}
    this.oldPositions=this.positions.map(p=>[...p]);
    this.velocities=this.positions.map(()=>[0,0,0]);this.forces=this.positions.map(()=>[0,0,0]);
    this.status='ready';this.time=0;this.pathLength=0;this.peakDeformation=0;this.panels=[];
    this.start=this.center();this.lastCenter=[...this.start];this.trail=[[...this.start]];
  }
  private launchDirection():V3 {
    const yaw=radians(this.parameters.heading),e=radians(this.parameters.elevation);
    return [Math.cos(e)*Math.cos(yaw),Math.sin(e),Math.cos(e)*Math.sin(yaw)];
  }
  launch(){
    this.reset();
    if(this.parameters.launchDuration<=0){const direction=this.launchDirection();this.velocities=this.positions.map(()=>direction.map(d=>d*this.parameters.speed) as V3);}
    this.status='flying';
  }
  aeroSettings(time=this.time):AeroSettings {
    return {model:this.parameters.aeroModel,surface:this.surface,enabled:this.parameters.aero,density:this.parameters.density,wind:this.parameters.wind,areaWeights:this.mesh.aeroWeights,
      windAt:this.parameters.gusts||this.parameters.turbulence?position=>airVelocity(this.parameters,time,position):undefined};
  }
  /** Synchronous API for tests/offline runs; finishes any partially advanced step. */
  step() {this.stepUntil(()=>false);}
  /** Yield only between complete internal substeps. True means an outer step
   * finished and may be recorded. Pausing keeps the iterator; reset discards it. */
  stepUntil(shouldYield:()=>boolean):boolean {
    if(this.status!=='flying'&&!this.pendingStep)return true;
    this.pendingStep??=this.integrateStep();
    do {
      if(this.pendingStep.next().done){this.pendingStep=undefined;return true;}
    } while(!shouldYield());
    return false;
  }
  private *integrateStep():Generator<void,void,void> {
    if(this.status!=='flying')return;
    // Very light sheets accelerate much faster under panel loads. Resolve them
    // with smaller substeps while keeping the recorded 120 Hz clock unchanged.
    // Explicit aerodynamic damping rate is ~ rho V pi A/m for a 2pi slope.
    // Resolve the fastest vertex mode; extra substeps preserve the 120 Hz outer clock.
    let rate=0;
    const launchDelta=Math.abs(springVelocityDelta(this.time,FIXED_DT,this.parameters.speed,this.parameters.launchDuration));
    if(this.parameters.aero)for(let i=0;i<this.positions.length;i++){
      // Include the upcoming launcher impulse when estimating the fastest mode.
      const speed=length(sub(this.velocities[i],airVelocity(this.parameters,this.time,this.positions[i])))+launchDelta;
      rate=Math.max(rate,this.parameters.density*Math.PI*speed*this.tributaryArea[i]*this.invMass[i]);
    }
    // Resolve at most one degree of spring phase per substep while loaded.
    // Low initial airspeed alone underestimates the resolution needed as the
    // load grows and coincident folded layers first open in crosswind.
    const launchSubsteps=this.parameters.speed!==0&&this.time<this.parameters.launchDuration
      ?Math.ceil(90*FIXED_DT/this.parameters.launchDuration):0;
    const minimum=Math.max(this.parameters.paperWeight<30?SUBSTEPS*3:SUBSTEPS,launchSubsteps);
    const substeps=this.parameters.substeps||Math.min(96,Math.max(minimum,Math.ceil(FIXED_DT*rate/.8))),dt=FIXED_DT/substeps;
    const preset=materialSettings(this.parameters.material,this.parameters.paperWeight);
    const flatScale=this.parameters.stiffness/preset.stiffness,creaseScale=this.parameters.crease/preset.crease;
    const rigidity=this.mesh.hinges.map(h=>hingeStiffness(this.material,h,flatScale,creaseScale)*(this.parameters.layerModel==='bonded'?(h.layerStiffness??1):1));
    const membraneAlpha=this.membranes.map(t=>membraneAlphas(t,this.material,dt));
    const direction=this.launchDirection();
    for(let substep=0;substep<substeps;substep++){
      const old=this.oldPositions;
      for(let i=0;i<this.positions.length;i++)for(let d=0;d<3;d++)old[i][d]=this.positions[i][d];
      for(let i=0;i<this.forces.length;i++){this.forces[i][0]=0;this.forces[i][1]=-this.parameters.gravity*this.mesh.masses[i];this.forces[i][2]=0;}
      this.panels=aerodynamicLoads(this.positions,this.velocities,this.mesh.faces,this.aeroSettings(),this.forces);
      // Distribute the launcher load by mass: a supported launch with no added
      // torque or artificial vertex-to-vertex velocity discontinuity. Local
      // hook/contact stresses are outside this ideal acceleration model.
      const acceleration=springVelocityDelta(this.time,dt,this.parameters.speed,this.parameters.launchDuration)/dt;
      if(acceleration)for(let i=0;i<this.forces.length;i++)for(let d=0;d<3;d++)this.forces[i][d]+=this.mesh.masses[i]*acceleration*direction[d];
      for(let i=0;i<this.positions.length;i++)for(let d=0;d<3;d++){
        this.velocities[i][d]+=dt*this.forces[i][d]*this.invMass[i];
        this.positions[i][d]+=dt*this.velocities[i][d];
      }
      for(const e of this.mesh.edges)e.lambda=0;
      for(const t of this.membranes){t.lambda[0]=0;t.lambda[1]=0;t.lambda[2]=0;}
      for(const h of this.mesh.hinges)h.lambda=0;
      const contacts=this.parameters.layerModel==='contact'?detectLayerContacts(this.positions,old,this.contactCandidates):[];
      this.selfContact.beginStep();
      for(let iteration=0;iteration<this.parameters.iterations;iteration++){
        // Alternating sweep direction reduces directional Gauss-Seidel bias.
        const forward=iteration%2===0;
        for(let j=0;j<this.membranes.length;j++)solveMembrane(this.positions,this.invMass,this.membranes[forward?j:this.membranes.length-1-j],this.material,dt,membraneAlpha[forward?j:this.membranes.length-1-j]);
        for(let j=0;j<this.mesh.hinges.length;j++){
          const h=this.mesh.hinges[forward?j:this.mesh.hinges.length-1-j];
          // Effective bending rigidity D; crease stiffness is an independent torsional spring.
          const k=rigidity[forward?j:this.mesh.hinges.length-1-j];
          solveHinge(this.positions,this.invMass,h,1/Math.max(k,1e-8),dt);
        }
        if(this.parameters.layerModel==='bonded')for(const tie of this.mesh.layerTies??[])solveLayerTie(this.positions,this.invMass,tie);
        for(let j=0;j<this.mesh.edges.length;j++)limitEdgeStrain(this.positions,this.invMass,this.mesh.edges[forward?j:this.mesh.edges.length-1-j]);
        for(const contact of contacts)solveLayerContact(this.positions,this.invMass,contact,this.material.friction);
        if(this.parameters.layerModel==='contact'){
          // Include structural projection in the sweep, not only free motion.
          // Contact is last so the strain limiter cannot silently undo it.
          if(iteration===0||iteration===this.parameters.iterations-1)this.selfContact.detect(old,this.positions);
          this.selfContact.solve(this.positions,this.invMass,forward);
        }
      }
      if(this.parameters.layerModel==='contact')for(let pass=0;pass<4;pass++)this.selfContact.solve(this.positions,this.invMass,pass%2===0);
      for(const h of this.mesh.hinges)if(h.crease)updateCreasePlasticity(h,hingeAngle(...h.ids.map(i=>this.positions[i]) as [V3,V3,V3,V3]),this.material);
      for(let i=0;i<this.positions.length;i++)for(let d=0;d<3;d++)this.velocities[i][d]=(this.positions[i][d]-old[i][d])/dt;
      dampEdges(this.positions,this.velocities,this.invMass,this.mesh.edges,1-Math.exp(-18*dt));
      this.time+=dt;
      if(this.positions.some(p=>p.some(v=>!Number.isFinite(v)||Math.abs(v)>1000))){this.positions=old;this.velocities=old.map(()=>[0,0,0]);this.status='error';return;}
      const center=this.center();this.pathLength+=length(sub(center,this.lastCenter));this.lastCenter=center;
      if(this.positions.some(p=>p[1]<=0)){this.status='landed';break;}
      if(this.time>=12){this.status='finished';break;}
      if(substep+1<substeps)yield;
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
  /** Return false from step when it yielded before finishing an outer step. */
  advance(elapsed:number,step:()=>unknown,shouldYield:()=>boolean=()=>false){
    const accepted=Math.min(Math.max(elapsed,0),0.1);this.droppedTime+=Math.max(0,elapsed-accepted);
    // Bound catch-up debt when work spans frames. Dropped wall time slows the
    // flight; it never skips integration or changes the physical timestep.
    const accumulated=this.accumulator+accepted;
    this.accumulator=Math.min(accumulated,0.1);this.droppedTime+=Math.max(0,accumulated-this.accumulator);
    let steps=0;
    while(this.accumulator>=FIXED_DT-1e-12 && steps<12){
      if(step()===false)break;
      this.accumulator-=FIXED_DT;steps++;
      if(shouldYield())break;
    }
    return steps;
  }
  reset(){this.accumulator=0;this.droppedTime=0;}
}
