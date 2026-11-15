/** The flight solver: a Rust crate (wasm/solver) compiled to WebAssembly by `npm run wasm`,
 * which leaves it beside this file as solver.wasm. `Simulation` compiles the mesh and all
 * derived set-up; a `SolverFlight` copies that in once and then advances the flight one
 * substep per call. Importing this module loads and instantiates the solver (top-level
 * await), in the page, in the flight worker and under Node alike. */
import type { Simulation, Status } from './simulation';
import { hingeStiffness } from './sheet-material';
import { materialSettings, materialFlex } from './materials';
import type { V3 } from './math';

interface Exports {
  memory:WebAssembly.Memory;
  stage_f64(field:number,len:number):number; stage_i32(field:number,len:number):number;
  sim_create():number; sim_free(sim:number):void; sim_advance(sim:number):number; sim_step(sim:number):number;
  sim_positions(sim:number):number; sim_velocities(sim:number):number; sim_forces(sim:number):number; sim_panels(sim:number):number;
  sim_plastic(sim:number,which:number):number; sim_layer_sides(sim:number,which:number):number; sim_layer_chosen(sim:number):number;
  sim_self_contacts(sim:number):number; sim_state(sim:number):number; sim_profile(sim:number):number;
  set_profiling(on:number):void;
  staged_f64_at(slot:number):number; staged_i32_at(slot:number):number; staged_f64_len(slot:number):number; staged_i32_len(slot:number):number;
  aero_create():number; aero_free(aero:number):void; aero_loads(aero:number,time:number,enabled:number,density:number):number; panel_loads(time:number,enabled:number,density:number):number;
  vortex_lattice(m:number):void; polar(alpha:number,which:number):number; air_velocity(time:number,x:number,y:number,z:number):void;
  spring_speed(time:number,speed:number,duration:number):number; spring_velocity_delta(time:number,dt:number,speed:number,duration:number):number;
  selfc_create(vertices:number):number; selfc_free(contact:number):void; selfc_reset(contact:number):void; selfc_begin(contact:number):void; selfc_carry(contact:number,gap:number):void;
  selfc_detect(contact:number):void; selfc_solve(contact:number,forward:number,sweep:number):void; selfc_report(contact:number):number;
  contact_project(vertexFace:number,sign:number):void; cubic_roots():number;
  layers_create():number; layers_free(layers:number):void; layers_detect(layers:number,full:number):number; layers_report(layers:number):void; layers_adopt(layers:number):void;
  layers_solve(layers:number,friction:number,sweep:number):void;
  ground_create(count:number):number; ground_free(ground:number):void; ground_reset(ground:number):void;
  ground_solve(ground:number,height:number,staticFriction:number,kineticFriction:number):number; ground_solve_velocity(ground:number,height:number,staticFriction:number,kineticFriction:number):number;
  elastic_create(vertices:number,blocks:number):number; elastic_free(solver:number):void; elastic_reset(solver:number):void; elastic_solve(solver:number,dt:number,iterations:number,tolerance:number):number;
  membranes_solve(forward:number):void; hinges_solve(forward:number):void;
  crease_plasticity(angle:number,rest:number,offset:number,accumulated:number,yieldAngle:number,hardening:number):number;
}
async function load():Promise<BufferSource> {
  const url=new URL('./solver.wasm',import.meta.url);
  if(url.protocol!=='file:')return (await fetch(url)).arrayBuffer();
  // Node (tests, benchmarks): read it from disk. The specifier is kept from the bundler.
  const fs='node:fs';
  return (await import(/* @vite-ignore */ fs)).readFileSync(url);
}
// `now` times solver stages while profiling is on.
const solver=(await WebAssembly.instantiate(await load(),{env:{now:()=>performance.now()}})).instance.exports as unknown as Exports;

/** Staging slots: the solver's inputs, then scratch slots for single kernels. */
export const F64={x:21,old:22,w:23,a:24,b:25,c:26,d:27,e:28,scalars:0,positions:1,velocities:2,masses:3,tributary:4,md:5,cd:6,membraneArea:7,hingeRest:8,hingeK:9,edgeRest:10,boundaries:12,patchWeights:13,patchFraction:14,faceSigns:15,
  layerOrientation:16,layerSign:17,layerSide:18,areaWeights:19,tieWeights:20};
export const I32={ids:33,flags:34,faces:0,hingeIds:1,hingeCrease:2,edgeIds:3,patchFace:4,patchStation:5,patchPanel:6,faceOwners:7,perm:8,ap:9,ai:10,source:11,diagonal:14,faceSlots:15,hingeSlots:16,
  layerVertex:17,layerFaceIndex:18,layerFaces:19,layerAmbiguous:20,layerSideIndex:21,layerChosen:22,neighbors:23,selfEdges:24,faceTreeLeaves:25,faceTreeRight:26,faceTreeEscape:27,
  edgeTreeLeaves:28,edgeTreeRight:29,edgeTreeEscape:30,landmarks:31,tieIds:32};
/** The fun air features of the weather, by the number the solver knows each as. */
export const WIND_MODES=['none','thermal','tornado','fan'] as const;
const STATUS:Status[]=['ready','flying','settling','landed','finished','error'];
/** Stage names for `profile()`, in the solver's order. */
export const STAGES=['schedule','air loads','elastic solve','membrane/hinge/strain sweeps','layer detect','layer solve','self-contact detect','self-contact solve','ground contact','floor-cleanup strain passes','velocity, damping, plasticity, status'] as const;
/** Time each solver stage (through the host clock) while on. */
export const setProfiling=(on:boolean)=>solver.set_profiling(on?1:0);

type Numbers=ArrayLike<number>;
/** The solver's kernels one at a time (wasm/solver/src/kernels.rs). Arguments go into
 * staging slots, a call runs the kernel, and results are read back from slots. */
export const kernel={
  get call(){return solver;},
  f64(slot:number,values:Numbers){const ptr=solver.stage_f64(slot,values.length);new Float64Array(solver.memory.buffer,ptr,values.length).set(values);},
  i32(slot:number,values:Numbers){const ptr=solver.stage_i32(slot,values.length);new Int32Array(solver.memory.buffer,ptr,values.length).set(values);},
  /** Per-vertex vectors as flat xyz. */
  vectors(slot:number,vectors:V3[]){
    const ptr=solver.stage_f64(slot,3*vectors.length),flat=new Float64Array(solver.memory.buffer,ptr,3*vectors.length);
    for(let i=0;i<vectors.length;i++){flat[3*i]=vectors[i][0];flat[3*i+1]=vectors[i][1];flat[3*i+2]=vectors[i][2];}
  },
  /** A copy of a slot's contents. */
  read64(slot:number){const length=solver.staged_f64_len(slot),ptr=solver.staged_f64_at(slot);return new Float64Array(solver.memory.buffer,ptr,length).slice();},
  read32(slot:number){const length=solver.staged_i32_len(slot),ptr=solver.staged_i32_at(slot);return new Int32Array(solver.memory.buffer,ptr,length).slice();},
  /** Read flat xyz back into per-vertex vectors. */
  readVectors(slot:number,into:V3[]){const flat=kernel.read64(slot);for(let i=0;i<into.length;i++){into[i][0]=flat[3*i];into[i][1]=flat[3*i+1];into[i][2]=flat[3*i+2];}},
  /** Free a solver-side object when its owner is collected. */
  own(owner:object,free:(exports:Exports)=>void){const exports=solver;released.register(owner,()=>free(exports));},
};
const released=new FinalizationRegistry<()=>void>(free=>free());
// Flights whose owner was dropped without a reset are freed when it is collected.
const dropped=new FinalizationRegistry<{exports:Exports;sim:number}>(({exports,sim})=>exports.sim_free(sim));

/** One flight inside the solver, created from a launched Simulation. */
export class SolverFlight {
  private exports=solver; private sim:number; private count:number; private hinges:number;
  constructor(source:Simulation){
    const p=source.parameters,mesh=source.mesh,material=source.material,surface=source.surface;
    const e=this.exports,f64=(field:number,values:Numbers)=>{const ptr=e.stage_f64(field,values.length);new Float64Array(e.memory.buffer,ptr,values.length).set(values);};
    const i32=(field:number,values:Numbers)=>{const ptr=e.stage_i32(field,values.length);new Int32Array(e.memory.buffer,ptr,values.length).set(values);};
    const flat=(vectors:V3[])=>Float64Array.from(vectors.flat());
    const preset=materialSettings(p.material,p.paperWeight),flatScale=p.stiffness/preset.stiffness*(materialFlex[p.material]??1),creaseScale=p.crease/preset.crease;
    const elastic=source.elastic,layers=source.layerContact,self=source.selfContact,bonded=p.layerModel==='bonded',ties=bonded?mesh.layerTies??[]:[];
    f64(F64.scalars,[p.speed,p.launchDuration,p.gravity,p.density,p.aero?1:0,p.substeps,p.iterations,p.newtonIterations,p.contactSweepShare,p.paperWeight,p.heading,p.elevation,
      p.wind[0],p.wind[1],p.wind[2],p.windHeading,p.gusts,p.turbulence,p.groundStaticFriction,p.groundKineticFriction,
      material.thickness,material.youngMD,material.youngCD,material.shear,material.friction,material.yieldAngle,material.hardening,
      surface.shieldingDistance,surface.span,surface.chordPanels,elastic.blocks,surface.owners?1:0,p.elasticSolve==='direct'?1:0,bonded?0:1,p.aeroModel==='panels'?1:0,
      source.time,...source.start,...source.lastCenter,Math.max(0,WIND_MODES.indexOf(p.windMode??'none'))]);
    f64(F64.positions,flat(source.positions));f64(F64.velocities,flat(source.velocities));
    f64(F64.masses,mesh.masses);f64(F64.tributary,source.tributaryArea);
    f64(F64.md,source.membraneBatch.md);f64(F64.cd,source.membraneBatch.cd);f64(F64.membraneArea,source.membranes.map(m=>m.area));
    f64(F64.hingeRest,source.hingeBatch.rest);
    // Effective bending rigidity D; crease stiffness is an independent torsional spring.
    // Contact mode: friction-held stacks bend as a laminate until they slip, and folds lock
    // their ends. Each of n coincident layers carries n^(2c) of its own rigidity, so the
    // stack totals n^(1+2c) D: c=0 bends layers independently, c=1 as one n-times-thick sheet.
    f64(F64.hingeK,mesh.hinges.map(h=>hingeStiffness(material,h,flatScale,creaseScale)*(bonded?(h.layerStiffness??1):(h.layers??1)**(2*p.stackCoupling))));
    f64(F64.edgeRest,source.edgeBatch.rest);
    f64(F64.boundaries,surface.boundaries);
    f64(F64.patchWeights,surface.patches.flatMap(q=>q.weights));f64(F64.patchFraction,surface.patches.map(q=>q.fraction));f64(F64.faceSigns,surface.signs);
    f64(F64.layerOrientation,layers.orientation);f64(F64.layerSign,layers.sign);f64(F64.layerSide,layers.side);
    f64(F64.areaWeights,mesh.aeroWeights??[]);f64(F64.tieWeights,ties.flatMap(t=>t.weights));
    i32(I32.faces,mesh.faces.flat());i32(I32.hingeIds,source.hingeBatch.ids);i32(I32.hingeCrease,mesh.hinges.map(h=>h.crease?1:0));i32(I32.edgeIds,source.edgeBatch.ids);
    i32(I32.patchFace,surface.patches.map(q=>q.face));i32(I32.patchStation,surface.patches.map(q=>q.station));i32(I32.patchPanel,surface.patches.map(q=>q.panel??q.station));
    i32(I32.faceOwners,surface.owners??[]);
    i32(I32.perm,elastic.perm);i32(I32.ap,elastic.ap);i32(I32.ai,elastic.ai);i32(I32.source,elastic.source);
    i32(I32.diagonal,elastic.diagonal);i32(I32.faceSlots,elastic.faceSlots);i32(I32.hingeSlots,elastic.hingeSlots);
    i32(I32.layerVertex,layers.vertex);i32(I32.layerFaceIndex,layers.faceIndex);i32(I32.layerFaces,layers.faces);i32(I32.layerAmbiguous,layers.ambiguous);
    i32(I32.layerSideIndex,layers.sideIndex);i32(I32.layerChosen,layers.chosen);
    i32(I32.neighbors,new Int32Array(self.neighbors.buffer,self.neighbors.byteOffset,self.neighbors.length));i32(I32.selfEdges,self.edges);
    i32(I32.faceTreeLeaves,self.faceTree?.leaves??[]);i32(I32.faceTreeRight,self.faceTree?.right??[]);i32(I32.faceTreeEscape,self.faceTree?.escape??[]);
    i32(I32.edgeTreeLeaves,self.edgeTree?.leaves??[]);i32(I32.edgeTreeRight,self.edgeTree?.right??[]);i32(I32.edgeTreeEscape,self.edgeTree?.escape??[]);
    i32(I32.landmarks,[surface.nose,surface.tail,surface.left,surface.right]);
    i32(I32.tieIds,ties.flatMap(t=>[t.vertex,...t.face]));
    this.sim=e.sim_create();this.count=mesh.rest.length;this.hinges=mesh.hinges.length;
    dropped.register(this,{exports:e,sim:this.sim},this);
  }
  private f64(ptr:number,length:number){return new Float64Array(this.exports.memory.buffer,ptr,length);}
  /** Advance one substep from the given vertex state, which is updated in place.
   * True when that completes a 1/120 s frame or the flight. */
  advance(positions:V3[],velocities:V3[],forces:V3[]){
    const e=this.exports,n=this.count;
    let x=this.f64(e.sim_positions(this.sim),3*n),v=this.f64(e.sim_velocities(this.sim),3*n);
    for(let i=0,k=0;i<n;i++,k+=3){const p=positions[i],u=velocities[i];x[k]=p[0];x[k+1]=p[1];x[k+2]=p[2];v[k]=u[0];v[k+1]=u[1];v[k+2]=u[2];}
    const complete=e.sim_advance(this.sim)===1;
    // The solver's memory may have grown: take fresh views.
    x=this.f64(e.sim_positions(this.sim),3*n);v=this.f64(e.sim_velocities(this.sim),3*n);const f=this.f64(e.sim_forces(this.sim),3*n);
    for(let i=0,k=0;i<n;i++,k+=3){
      const p=positions[i],u=velocities[i],force=forces[i];
      p[0]=x[k];p[1]=x[k+1];p[2]=x[k+2];u[0]=v[k];u[1]=v[k+1];u[2]=v[k+2];force[0]=f[k];force[1]=f[k+1];force[2]=f[k+2];
    }
    return complete;
  }
  /** See `sim_state` in wasm/solver/src/lib.rs. */
  get state(){
    const s=this.f64(this.exports.sim_state(this.sim),27);
    return {time:s[0],status:STATUS[s[1]],touchdown:s[2]?{time:s[3],center:[s[4],s[5],s[6]] as V3,distance:s[7]}:undefined,groundDistance:s[8],pathLength:s[9],
      lastCenter:[s[10],s[11],s[12]] as V3,contacts:{narrowTests:s[14],vertexFace:s[15],edgeEdge:s[16]},selfContacts:s[17],layerContacts:s[18],pairs:s[19],panels:s[20],
      groundTouched:s[21]===1,substep:s[22],substeps:s[23],detectionMs:[s[24],s[25],s[26]]};
  }
  /** Air loads of the last sample, twelve values per panel (see flight-messages). */
  get panels(){return this.f64(this.exports.sim_panels(this.sim),12*this.state.panels);}
  get plasticOffset(){return this.f64(this.exports.sim_plastic(this.sim,0),this.hinges);}
  get plasticAccumulated(){return this.f64(this.exports.sim_plastic(this.sim,1),this.hinges);}
  /** Layer side state: seeded sign and chosen flag per candidate, side per shared side. */
  layerSides(candidates:number,sides:number){
    const e=this.exports,chosen=e.sim_layer_chosen(this.sim);
    return {sign:this.f64(e.sim_layer_sides(this.sim,0),candidates),side:this.f64(e.sim_layer_sides(this.sim,1),sides),chosen:new Uint8Array(e.memory.buffer,chosen,candidates)};
  }
  /** Active self-contacts of the last substep. */
  selfContacts(){
    // The call allocates, which may move the solver's memory: take the buffer after it.
    const count=this.state.selfContacts,ptr=this.exports.sim_self_contacts(this.sim),raw=new Int32Array(this.exports.memory.buffer,ptr,7*count);
    return Array.from({length:count},(_,i)=>({ids:Array.from(raw.subarray(7*i,7*i+4)) as [number,number,number,number],kind:raw[7*i+4]?'vertex-face' as const:'edge-edge' as const,sign:raw[7*i+5],key:raw[7*i+6]}));
  }
  /** Stage times (ms) accumulated while profiling is on. */
  profile(){return Array.from(this.f64(this.exports.sim_profile(this.sim),STAGES.length));}
  free(){dropped.unregister(this);this.exports.sim_free(this.sim);this.sim=0;}
}
