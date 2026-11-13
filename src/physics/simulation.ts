import { buildMembranes, membraneBatch, solveMembranes, type Membrane, type MembraneBatch } from './membrane';
import { buildLiftingSurface, type LiftingSurface } from './lifting-surface';
import { buildContactCandidates, LayerContactSolver, type ContactCandidate } from './layer-contact';
import { SelfContactSolver } from './self-contact';
import { GroundContactSolver, GROUND_HEIGHT, type GroundSettings } from './ground-contact';
import { sheetMaterial, prepareMaterialGeometry, hingeStiffness, updateCreasePlasticity, type SheetMaterial } from './sheet-material';
import { createDart, hingeAngle, type PaperMesh } from '../geometry/dart';
import { aerodynamicLoads, type PanelLoad, type AeroSettings } from './aerodynamics';
import { airVelocity } from './weather';
import { materialSettings, type MaterialId } from './materials';
import { RELEASE_DURATION_MS, springVelocityDelta } from './launcher';
import { dampEdges, edgeBatch, edgeStrainsWithin, hingeBatch, limitEdgeStrains, setHingeOffset, solveHinges, solveLayerTie, type EdgeBatch, type HingeBatch } from './constraints';
import { cross, dot, length, radians, sub, unit, wrapAngle, type V3 } from './math';
// Work-saving schedule. Each item trades exactness for time where the
// difference was measured to be within run-to-run variation.
/** Scan every layer candidate on alternate frames; substeps between revisit the near list. */
const LAYER_SCAN_FRAMES=2;
/** A contact hovering this close (m) on its own side stays active into the next
 * substep. Tighter once on the floor, where lingering contacts delay settling. */
const CARRY_GAP=1e-4,RESTING_CARRY_GAP=1e-5;
/** Below this many broad-phase pairs a second, early swept detection is cheap enough to keep. */
const EARLY_DETECTION_PAIRS=12000;
/** Fewest constraint sweeps per substep when extra substeps share a frame's sweeps. */
const MIN_SWEEPS=18;
export const FIXED_DT=1/120;
export const SUBSTEPS=3;
/** Leave room for rendering/input; an indivisible substep can exceed this budget. */
export const FRAME_PHYSICS_BUDGET_MS=12;
export interface Parameters extends GroundSettings { aeroModel:'coupled'|'panels'; layerModel:'contact'|'bonded'; aeroStations:number; aeroChordPanels:number; substeps:number; iterations:number; speed:number; launchDuration:number; pitch:number; heading:number; elevation:number; span:number; size:number; paperWeight:number; stiffness:number; crease:number; dihedral:number; elevator:number; height:number; gravity:number; density:number; wind:V3; windHeading:number; gusts:number; turbulence:number; material:MaterialId; aero:boolean }
export const DEFAULTS: Parameters={groundStaticFriction:.55,groundKineticFriction:.38,aeroModel:'coupled',layerModel:'contact',aeroStations:12,aeroChordPanels:4,substeps:0,iterations:24,speed:6,launchDuration:RELEASE_DURATION_MS/1000,pitch:8,heading:0,elevation:0,span:1,size:1,paperWeight:80,stiffness:1,crease:1,dihedral:8,elevator:0,height:1.8,gravity:9.81,density:1.225,wind:[0,0,0],windHeading:0,gusts:0,turbulence:0,material:'paper',aero:true};
export type Status='ready'|'flying'|'settling'|'landed'|'finished'|'error';
export const isActiveStatus=(status:Status)=>status==='flying'||status==='settling';
export interface Touchdown { time:number; center:V3; distance:number }
export interface Metrics { time:number; airTime:number; flightDistance:number; groundDistance:number; speed:number; altitude:number; distance:number; pathLength:number; deformation:number; bend:number; maxStrain:number; force:V3; stallFraction:number; center:V3 }
export class Simulation {
  tributaryArea:number[]; membranes:Membrane[];
  surface:LiftingSurface; material:SheetMaterial; layerContact:LayerContactSolver;
  selfContact:SelfContactSolver;
  groundContact:GroundContactSolver;
  touchdown?:Touchdown;groundDistance=0;
  private quietTime=0;
  mesh:PaperMesh; positions:V3[]=[]; velocities:V3[]=[]; forces:V3[]=[]; invMass:number[];
  panels:PanelLoad[]=[]; parameters:Parameters; status:Status='ready'; time=0; pathLength=0;
  start:V3=[0,0,0]; trail:V3[]=[]; peakDeformation=0; lastCenter:V3=[0,0,0];
  private pendingStep?:Generator<void,void,void>;
  // Solver working state: flat xyz copies of the per-vertex arrays above, and
  // structure-of-arrays constraints. The per-vertex arrays stay authoritative
  // between substeps, so readers and external edits see a consistent state.
  private x:Float64Array; private xOld:Float64Array; private v:Float64Array; private w:Float64Array; private air:Float64Array;
  private exposure={values:new Float64Array(0),valid:false}; private frame=0;
  private hingeBatch:HingeBatch; private edgeBatch:EdgeBatch; private membraneBatch:MembraneBatch;
  constructor(parameters:Partial<Parameters>={},mesh?:PaperMesh) {
    this.parameters={...DEFAULTS,...materialSettings(parameters.material??'paper',parameters.paperWeight??80),...parameters,wind:[...(parameters.wind??DEFAULTS.wind)]};
    this.mesh=mesh?structuredClone(mesh):createDart(this.parameters.dihedral,8,5,this.parameters.span,this.parameters.size,0.005*this.parameters.size**2*this.parameters.paperWeight/80,this.parameters.elevator);this.invMass=this.mesh.masses.map(m=>1/m);
    this.tributaryArea=this.mesh.rest.map(()=>0);
    for(const [a,b,c] of this.mesh.faces){const area=length(cross(sub(this.mesh.rest[b],this.mesh.rest[a]),sub(this.mesh.rest[c],this.mesh.rest[a])))/6;for(const i of [a,b,c])this.tributaryArea[i]+=area;}
    this.membranes=buildMembranes(this.mesh);
    this.material=sheetMaterial(this.parameters.material,this.parameters.paperWeight);prepareMaterialGeometry(this.mesh);
    this.surface=buildLiftingSurface(this.mesh,this.parameters.aeroStations,Math.max(.0002,2*this.material.thickness),this.parameters.aeroChordPanels);
    this.layerContact=new LayerContactSolver(buildContactCandidates(this.mesh));this.selfContact=new SelfContactSolver(this.mesh);this.groundContact=new GroundContactSolver(this.mesh.rest.length);
    const size=this.mesh.rest.length*3;
    this.x=new Float64Array(size);this.xOld=new Float64Array(size);this.v=new Float64Array(size);this.air=new Float64Array(size);this.w=new Float64Array(this.mesh.rest.length);
    this.hingeBatch=hingeBatch(this.mesh.hinges);this.edgeBatch=edgeBatch(this.mesh.edges);this.membraneBatch=membraneBatch(this.membranes);
    this.reset();
  }
  get active(){return isActiveStatus(this.status);}
  /** Rest-coplanar layer candidates, with their current side state. */
  get contactCandidates():ContactCandidate[]{this.layerContact.sync();return this.layerContact.candidates;}
  get groundHeight(){return GROUND_HEIGHT+this.material.thickness/2;}
  center(array=this.positions):V3 {
    const c:V3=[0,0,0],mass=this.mesh.masses.reduce((a,b)=>a+b,0);
    for(let i=0;i<array.length;i++)for(let d=0;d<3;d++)c[d]+=array[i][d]*this.mesh.masses[i]/mass;
    return c;
  }
  reset() {
    this.pendingStep=undefined;this.frame=0;this.exposure.valid=false;
    this.selfContact.reset();
    this.groundContact.reset();this.touchdown=undefined;this.groundDistance=0;this.quietTime=0;
    for(const c of this.layerContact.candidates){c.sideChosen=false;if(c.sideState)c.sideState.side=undefined;}
    this.layerContact.reset();
    for(const h of this.mesh.hinges){h.plasticOffset=0;h.plasticAccumulated=0;h.lambda=0;}
    const angle=radians(this.parameters.pitch),c=Math.cos(angle),s=Math.sin(angle);
    const yaw=radians(this.parameters.heading),cy=Math.cos(yaw),sy=Math.sin(yaw);
    this.positions=this.mesh.rest.map(([x,y,z])=>{const px=c*x-s*y;return [cy*px-sy*z,s*x+c*y,sy*px+cy*z];});
    const center=this.center();
    for(const p of this.positions){p[0]-=center[0];p[1]+=this.parameters.height-center[1];p[2]-=center[2];}
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
    if(!this.active&&!this.pendingStep)return true;
    this.pendingStep??=this.integrateStep();
    do {
      if(this.pendingStep.next().done){this.pendingStep=undefined;return true;}
    } while(!shouldYield());
    return false;
  }
  private *integrateStep():Generator<void,void,void> {
    if(!this.active)return;
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
    // Resolve near-floor motion in roughly 2 mm increments, including lateral
    // motion that can tip a contacting nose/wing. Explicit substeps still win.
    let contactSubsteps=0;
    for(let i=0;i<this.positions.length;i++){
      const speed=length(this.velocities[i])+launchDelta;
      if(this.positions[i][1]-this.groundHeight<.03+speed*FIXED_DT)
        contactSubsteps=Math.max(contactSubsteps,speed>.5?12:SUBSTEPS,Math.ceil(FIXED_DT*(speed+this.parameters.gravity*FIXED_DT)/.002));
    }
    const baseSubsteps=this.parameters.paperWeight<30?SUBSTEPS*3:SUBSTEPS;
    const minimum=Math.max(baseSubsteps,launchSubsteps,contactSubsteps);
    const aeroSubsteps=Math.max(1,Math.ceil(FIXED_DT*rate/.8));
    const substeps=this.parameters.substeps||Math.min(96,Math.max(minimum,aeroSubsteps)),dt=FIXED_DT/substeps;
    const aeroStride=Math.max(1,Math.floor(substeps/aeroSubsteps));
    const preset=materialSettings(this.parameters.material,this.parameters.paperWeight);
    const flatScale=this.parameters.stiffness/preset.stiffness,creaseScale=this.parameters.crease/preset.crease;
    const count=this.positions.length,x=this.x,old=this.xOld,v=this.v,w=this.w,air=this.air,frame=this.frame++;
    const hinges=this.hingeBatch,edges=this.edgeBatch,membranes=this.membraneBatch,layers=this.layerContact,self=this.selfContact,ground=this.groundContact;
    const contact=this.parameters.layerModel==='contact';
    // Constraint sweeps converge faster per sweep at a smaller timestep. When
    // floor contact, the launch stroke or air loads raise the substep count
    // above the sheet's usual one, share that frame's sweeps between them
    // (down to MIN_SWEEPS each) instead of multiplying the work.
    const requested=this.parameters.iterations;
    const iterations=!this.parameters.substeps&&substeps>baseSubsteps?Math.min(requested,Math.max(MIN_SWEEPS,Math.ceil(requested*baseSubsteps/substeps))):requested;
    for(let i=0;i<count;i++)w[i]=this.invMass[i];
    // Effective bending rigidity D; crease stiffness is an independent torsional spring.
    this.mesh.hinges.forEach((h,j)=>{
      const k=hingeStiffness(this.material,h,flatScale,creaseScale)*(this.parameters.layerModel==='bonded'?(h.layerStiffness??1):1);
      hinges.alpha[j]=1/Math.max(k,1e-8)/(dt*dt);
    });
    this.membranes.forEach((t,j)=>{
      const scale=this.material.thickness*t.area*dt*dt;
      membranes.alpha[3*j]=1/(this.material.youngMD*scale);membranes.alpha[3*j+1]=1/(this.material.youngCD*scale);membranes.alpha[3*j+2]=1/(this.material.shear*scale);
    });
    const direction=this.launchDirection();
    for(let substep=0;substep<substeps;substep++){
      for(let i=0;i<count;i++){
        const p=this.positions[i],u=this.velocities[i],k=3*i;
        x[k]=p[0];x[k+1]=p[1];x[k+2]=p[2];v[k]=u[0];v[k+1]=u[1];v[k+2]=u[2];
      }
      old.set(x);
      // Air loads are sampled as often as their explicit stability needs, and
      // held between samples; structure and contact still advance every substep.
      if(substep%aeroStride===0){
        for(const force of this.forces){force[0]=0;force[1]=0;force[2]=0;}
        const settings=this.aeroSettings();
        // Layer shielding is refreshed once per frame.
        settings.exposure=this.exposure;if(substep===0)this.exposure.valid=false;
        this.panels=aerodynamicLoads(this.positions,this.velocities,this.mesh.faces,settings,this.forces);
        for(let i=0;i<count;i++){const force=this.forces[i],k=3*i;air[k]=force[0];air[k+1]=force[1];air[k+2]=force[2];}
      }
      for(let i=0;i<count;i++){const force=this.forces[i],k=3*i;force[0]=air[k];force[1]=air[k+1]-this.parameters.gravity*this.mesh.masses[i];force[2]=air[k+2];}
      // Distribute the launcher load by mass: a supported launch with no added
      // torque or artificial vertex-to-vertex velocity discontinuity. Local
      // hook/contact stresses are outside this ideal acceleration model.
      const acceleration=springVelocityDelta(this.time,dt,this.parameters.speed,this.parameters.launchDuration)/dt;
      if(acceleration)for(let i=0;i<this.forces.length;i++)for(let d=0;d<3;d++)this.forces[i][d]+=this.mesh.masses[i]*acceleration*direction[d];
      for(let i=0;i<count;i++){
        const force=this.forces[i],k=3*i,weight=w[i];
        v[k]+=dt*force[0]*weight;v[k+1]+=dt*force[1]*weight;v[k+2]+=dt*force[2]*weight;
        x[k]+=dt*v[k];x[k+1]+=dt*v[k+1];x[k+2]+=dt*v[k+2];
      }
      membranes.lambda.fill(0);hinges.lambda.fill(0);
      for(let j=0;j<this.mesh.hinges.length;j++)setHingeOffset(hinges,j,this.mesh.hinges[j].plasticOffset??0);
      if(contact)layers.detect(x,old,substep===0&&frame%LAYER_SCAN_FRAMES===0);else layers.count=0;
      self.beginStep();
      // Contacts that still touch, or hover just apart, stay active from the
      // previous substep: every sweep enforces them without re-detection, and
      // resting stacks do not chatter through repeated re-impact.
      if(contact)self.carry(old,this.touchdown?RESTING_CARRY_GAP:CARRY_GAP);
      ground.reset();
      // Small meshes can afford the early swept detection as well.
      const early=contact&&self.pairCount<=EARLY_DETECTION_PAIRS;
      for(let iteration=0;iteration<iterations;iteration++){
        // Alternating sweep direction reduces directional Gauss-Seidel bias.
        const forward=iteration%2===0;
        solveMembranes(x,w,membranes,forward);
        solveHinges(x,w,hinges,forward);
        if(this.parameters.layerModel==='bonded')for(const tie of this.mesh.layerTies??[])solveLayerTie(x,w,tie);
        limitEdgeStrains(x,w,edges,forward);
        layers.solve(x,w,this.material.friction,iteration);
        if(contact){
          // Include structural projection in the sweep, not only free motion.
          // Contact is last so the strain limiter cannot silently undo it.
          // Swept detection on the last sweep adds every fresh crossing.
          if((early&&iteration===0)||iteration===iterations-1)self.detectFlat(old,x);
          self.solveFlat(x,w,forward,iteration);
        }
        ground.solveFlat(x,old,w,this.groundHeight,this.parameters);
      }
      const groundCleanup=ground.touched;
      for(let pass=0;pass<(groundCleanup?32:contact?4:0);pass++){
        // A pinned nose can concentrate compression into a few very short
        // edges. Reconcile the no-tearing envelope with both contact systems.
        if(groundCleanup)limitEdgeStrains(x,w,edges,pass%2===0);
        if(contact)self.solveFlat(x,w,pass%2===0);
        ground.solveFlat(x,old,w,this.groundHeight,this.parameters);
        // Most contacts converge in eight sweeps. Small triangles in dense
        // folds need extra reconciliation at impact, not a looser strain bound.
        if(groundCleanup&&pass>=7&&(pass+1)%4===0&&edgeStrainsWithin(x,edges,.05))break;
      }
      for(let k=0;k<3*count;k++)v[k]=(x[k]-old[k])/dt;
      dampEdges(x,v,w,edges,1-Math.exp(-18*dt));
      const supported=ground.solveVelocityFlat(x,v,this.groundHeight,this.parameters);
      let finite=true;
      for(let k=0;k<3*count;k++)if(!(Math.abs(x[k])<=1000)){finite=false;break;}
      for(let i=0;i<count;i++){
        const p=this.positions[i],u=this.velocities[i],k=3*i;
        if(finite){p[0]=x[k];p[1]=x[k+1];p[2]=x[k+2];u[0]=v[k];u[1]=v[k+1];u[2]=v[k+2];}
        else{p[0]=old[k];p[1]=old[k+1];p[2]=old[k+2];u[0]=0;u[1]=0;u[2]=0;}
      }
      if(finite)for(const h of this.mesh.hinges)if(h.crease)updateCreasePlasticity(h,hingeAngle(...h.ids.map(i=>this.positions[i]) as [V3,V3,V3,V3]),this.material);
      this.time+=dt;
      if(!finite){this.status='error';return;}
      const center=this.center();this.pathLength+=length(sub(center,this.lastCenter));
      if(this.touchdown)this.groundDistance+=Math.hypot(center[0]-this.lastCenter[0],center[2]-this.lastCenter[2]);
      this.lastCenter=center;
      if(!this.touchdown&&this.groundContact.touched){
        this.touchdown={time:this.time,center:[...center],distance:Math.hypot(center[0]-this.start[0],center[2]-this.start[2])};
        this.status='settling';
      }
      // Mass-weighted world-space kinetic energy includes rocking and flutter.
      // A tiny layer vertex may jitter while the sheet is otherwise at rest;
      // an additional peak-speed limit prevents sleeping a visibly moving tip.
      let speedSquared=0,mass=0,peakSpeedSquared=0;
      for(let i=0;i<this.velocities.length;i++){
        const v=this.velocities[i],sq=dot(v,v),m=this.mesh.masses[i];
        speedSquared+=m*sq;mass+=m;peakSpeedSquared=Math.max(peakSpeedSquared,sq);
      }
      const nearGround=this.positions.some(p=>p[1]<=this.groundHeight+.0002);
      this.quietTime=nearGround&&this.time>=this.parameters.launchDuration&&speedSquared/mass<.01**2&&peakSpeedSquared<.1**2?this.quietTime+dt:0;
      // Allow submillimetre contact chatter during the dwell, but finish only
      // on a substep with actual compressive support from the floor.
      if(this.touchdown&&supported&&this.quietTime>=.5){
        for(const v of this.velocities)v.fill(0);
        this.status='landed';break;
      }
      // A gust can keep a light sheet moving. Report a timeout honestly, and
      // allow a late touchdown its own full settling window.
      if(this.touchdown?this.time-this.touchdown.time>=8:this.time>=12){this.status='finished';break;}
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
    const distance=Math.hypot(center[0]-this.start[0],center[2]-this.start[2]);
    return {time:this.time,airTime:this.touchdown?.time??this.time,flightDistance:this.touchdown?.distance??distance,groundDistance:this.groundDistance,speed:length(sub(velocity,airVelocity(this.parameters,this.time,center))),altitude:center[1],distance,pathLength:this.pathLength,deformation:this.deformation().rms,bend:Math.sqrt(bend/Math.max(count,1))*180/Math.PI,maxStrain,force,stallFraction:area?stall/area:0,center};
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
