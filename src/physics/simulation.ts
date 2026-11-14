import { ElasticSolver } from './elastic-solve';
import { buildMembranes, membraneBatch, type Membrane, type MembraneBatch } from './membrane';
import { buildLiftingSurface, type LiftingSurface } from './lifting-surface';
import { buildContactCandidates, LayerContactSolver, type ContactCandidate } from './layer-contact';
import { SelfContactSolver, type SelfContact } from './self-contact';
import { GROUND_HEIGHT, type GroundSettings } from './ground-contact';
import { sheetMaterial, prepareMaterialGeometry, type SheetMaterial } from './sheet-material';
import { createDart, hingeAngle, type PaperMesh } from '../geometry/dart';
import { unpackPanels, type PanelLoad, type AeroSettings } from './aerodynamics';
import { airVelocity, type WindMode } from './weather';
import { materialSettings, type MaterialId } from './materials';
import { RELEASE_DURATION_MS } from './launcher';
import { edgeBatch, hingeBatch, type EdgeBatch, type HingeBatch } from './constraints';
import { cross, dot, length, radians, sub, unit, wrapAngle, type V3 } from './math';
import { SolverFlight } from './solver';
export const FIXED_DT=1/120;
/** Substeps per frame in steady flight; launch, floor contact and light sheets take more. */
export const SUBSTEPS=3;
/** Leave room for rendering/input; an indivisible substep can exceed this budget. */
export const FRAME_PHYSICS_BUDGET_MS=12;
export interface Parameters extends GroundSettings { aeroModel:'coupled'|'panels'; layerModel:'contact'|'bonded'; /** 'direct' solves each substep's elastic response exactly before the constraint sweeps; 'sweeps' leaves it to them. */ elasticSolve:'direct'|'sweeps'; newtonIterations:number; /** Fraction of `iterations` run after a direct elastic solve, where sweeps only reconcile contact. */ contactSweepShare:number; aeroStations:number; aeroChordPanels:number; stackCoupling:number; substeps:number; iterations:number; speed:number; launchDuration:number; pitch:number; heading:number; elevation:number; span:number; size:number; paperWeight:number; stiffness:number; crease:number; dihedral:number; elevator:number; height:number; gravity:number; density:number; wind:V3; windHeading:number; gusts:number; turbulence:number; windMode?:WindMode; material:MaterialId; aero:boolean }
export const DEFAULTS: Parameters={groundStaticFriction:.55,groundKineticFriction:.38,aeroModel:'coupled',layerModel:'contact',elasticSolve:'direct',newtonIterations:16,contactSweepShare:.5,aeroStations:12,aeroChordPanels:4,stackCoupling:1,substeps:0,iterations:24,speed:6,launchDuration:RELEASE_DURATION_MS/1000,pitch:8,heading:0,elevation:0,span:1,size:1,paperWeight:80,stiffness:1,crease:1,dihedral:8,elevator:0,height:1.8,gravity:9.81,density:1.225,wind:[0,0,0],windHeading:0,gusts:0,turbulence:0,material:'paper',aero:true};
export type Status='ready'|'flying'|'settling'|'landed'|'finished'|'error';
export const isActiveStatus=(status:Status)=>status==='flying'||status==='settling';
export interface Touchdown { time:number; center:V3; distance:number }
export interface Metrics { time:number; airTime:number; flightDistance:number; groundDistance:number; speed:number; altitude:number; distance:number; pathLength:number; deformation:number; bend:number; maxStrain:number; force:V3; stallFraction:number; center:V3 }
/** One flight: the mesh, its derived set-up and the state the game reads. The flight
 * solver (wasm/solver, through `SolverFlight`) advances it; nothing here integrates. */
export class Simulation {
  tributaryArea:number[]; membranes:Membrane[];
  surface:LiftingSurface; material:SheetMaterial;
  /** Set-up the solver takes at launch: rest-coplanar layer candidates, self-contact
   * topology, constraint batches and the elastic solve's symbolic factorisation. */
  layerContact:LayerContactSolver; selfContact:SelfContactSolver;
  hingeBatch:HingeBatch; edgeBatch:EdgeBatch; membraneBatch:MembraneBatch; elastic:ElasticSolver;
  touchdown?:Touchdown;groundDistance=0;
  mesh:PaperMesh; positions:V3[]=[]; velocities:V3[]=[];
  /** Air, weight and launcher force on each vertex in the last substep. */
  forces:V3[]=[]; invMass:number[];
  parameters:Parameters; status:Status='ready'; time=0; pathLength=0;
  start:V3=[0,0,0]; trail:V3[]=[]; peakDeformation=0; lastCenter:V3=[0,0,0];
  /** This flight inside the solver, from the first step after a launch until the next reset. */
  private flight?:SolverFlight; private stepping=false;
  private panelLoads:PanelLoad[]=[]; private panelsStale=false;
  /** Air loads of the last sample. */
  get panels():PanelLoad[]{
    if(this.panelsStale&&this.flight)this.panelLoads=unpackPanels(this.flight.panels);
    this.panelsStale=false;return this.panelLoads;
  }
  set panels(panels:PanelLoad[]){this.panelLoads=panels;this.panelsStale=false;}
  /** Solver time per stage (ms, in `STAGES` order) while `setProfiling` is on. */
  get stageTimes(){return this.flight?.profile();}
  /** Self-contacts active in the last substep. */
  get selfContacts():SelfContact[]{return this.flight?.selfContacts()??[];}
  constructor(parameters:Partial<Parameters>={},mesh?:PaperMesh) {
    this.parameters={...DEFAULTS,...materialSettings(parameters.material??'paper',parameters.paperWeight??80),...parameters,wind:[...(parameters.wind??DEFAULTS.wind)]};
    this.mesh=mesh?structuredClone(mesh):createDart(this.parameters.dihedral,8,5,this.parameters.span,this.parameters.size,0.005*this.parameters.size**2*this.parameters.paperWeight/80,this.parameters.elevator);this.invMass=this.mesh.masses.map(m=>1/m);
    this.tributaryArea=this.mesh.rest.map(()=>0);
    for(const [a,b,c] of this.mesh.faces){const area=length(cross(sub(this.mesh.rest[b],this.mesh.rest[a]),sub(this.mesh.rest[c],this.mesh.rest[a])))/6;for(const i of [a,b,c])this.tributaryArea[i]+=area;}
    this.membranes=buildMembranes(this.mesh);
    this.material=sheetMaterial(this.parameters.material,this.parameters.paperWeight);prepareMaterialGeometry(this.mesh);
    this.surface=buildLiftingSurface(this.mesh,this.parameters.aeroStations,Math.max(.0002,2*this.material.thickness),this.parameters.aeroChordPanels);
    this.layerContact=new LayerContactSolver(buildContactCandidates(this.mesh));this.selfContact=new SelfContactSolver(this.mesh);
    this.hingeBatch=hingeBatch(this.mesh.hinges);this.edgeBatch=edgeBatch(this.mesh.edges);this.membraneBatch=membraneBatch(this.membranes);
    this.elastic=new ElasticSolver(this.mesh.rest.length,this.mesh.faces,this.mesh.hinges);
    this.reset();
  }
  get active(){return isActiveStatus(this.status);}
  /** Rest-coplanar layer candidates, with their current side state. */
  get contactCandidates():ContactCandidate[]{this.readLayerSides();this.layerContact.sync();return this.layerContact.candidates;}
  /** Side state lives in the solver during a flight; the seeded signs outlast it. */
  private readLayerSides(){
    if(!this.flight)return;
    const layers=this.layerContact,state=this.flight.layerSides(layers.sign.length,layers.side.length);
    layers.sign.set(state.sign);layers.chosen.set(state.chosen);layers.side.set(state.side);
  }
  private endFlight(){if(!this.flight)return;this.readLayerSides();this.flight.free();this.flight=undefined;this.stepping=false;}
  get groundHeight(){return GROUND_HEIGHT+this.material.thickness/2;}
  center(array=this.positions):V3 {
    const c:V3=[0,0,0],mass=this.mesh.masses.reduce((a,b)=>a+b,0);
    for(let i=0;i<array.length;i++)for(let d=0;d<3;d++)c[d]+=array[i][d]*this.mesh.masses[i]/mass;
    return c;
  }
  reset() {
    this.endFlight();this.panelsStale=false;this.selfContact.stats={narrowTests:0,vertexFace:0,edgeEdge:0};
    this.touchdown=undefined;this.groundDistance=0;
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
      air:{weather:this.parameters,time}};
  }
  /** Synchronous API for tests/offline runs; finishes any partially advanced step. */
  step() {this.stepUntil(()=>false);}
  /** Advance substep by substep until `shouldYield` says to stop or a 1/120 s step is
   * complete. True means an outer step finished and may be recorded; false leaves it
   * part done, to be continued by the next call. Vertex positions and velocities may be
   * edited between calls. A reset discards a part-done step. */
  stepUntil(shouldYield:()=>boolean):boolean {
    if(!this.active&&!this.stepping)return true;
    const flight=this.flight??=new SolverFlight(this);
    do {
      const complete=flight.advance(this.positions,this.velocities,this.forces),state=flight.state;
      this.stepping=!complete;this.panelsStale=true;
      this.time=state.time;this.status=state.status;this.touchdown=state.touchdown;this.groundDistance=state.groundDistance;this.pathLength=state.pathLength;this.lastCenter=state.lastCenter;
      this.selfContact.stats=state.contacts;
      const offset=flight.plasticOffset,accumulated=flight.plasticAccumulated,hinges=this.mesh.hinges;
      for(let j=0;j<hinges.length;j++){hinges[j].plasticOffset=offset[j];hinges[j].plasticAccumulated=accumulated[j];}
      if(complete){
        if(this.status!=='error'){this.trail.push(this.center());this.peakDeformation=Math.max(this.peakDeformation,this.deformation().rms);}
        return true;
      }
    } while(!shouldYield());
    return false;
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
