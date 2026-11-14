/** Compare the game's flight solver with the reference solver on the same launches.
 *
 *   npm run reference                         every design, default launch
 *   npm run reference -- basic dart           chosen designs
 *   npm run reference -- --speeds 5,6,7       several launch speeds per design
 *   npm run reference -- --set pitch=12 --material tissue
 *   npm run reference -- --game substeps=24 --game iterations=60    game-only settings
 *   npm run reference -- --json out.json      also write the table as JSON
 *
 * Reference flights are stored in reference/results and reused until the problem or the
 * reference solver changes (--refresh recomputes). Solver options: --steps-per-frame,
 * --aero-stations, --aero-chord, --patch-area, --contact-stiffness, --after-touchdown,
 * --first-opening-sides. --stored-only never runs the solver. */
import { availableParallelism } from 'node:os';
import { writeFileSync } from 'node:fs';
import { caseFor, gameSimulation, type Case } from './export';
import { solveAll, stored, type Job, type ReferenceResult, type SolverOptions } from './run';
import { designs } from '../src/folding/library';
import { dot, length, sub, unit, type V3 } from '../src/physics/math';
import type { Parameters } from '../src/physics/simulation';

export interface GameFlight { frames:{time:number;center:V3;forward:V3;deformation:number}[]; airTime:number; flightDistance:number; status:string; peakFlightDeformation:number }
/** Fly a case with the game's solver, recording what the reference records. */
export function flyGame(test:Case,gameOnly:Partial<Parameters>={}):GameFlight {
  const sim=gameSimulation({...test,parameters:{...test.parameters,...gameOnly}});sim.launch();
  const sample=()=>({time:sim.time,center:sim.center(),forward:unit(sub(sim.positions[sim.mesh.nose],sim.positions[sim.mesh.tail])),deformation:sim.deformation().rms});
  const frames=[sample()];let peak=0;
  while(sim.active&&!sim.touchdown){sim.step();frames.push(sample());if(!sim.touchdown)peak=Math.max(peak,frames.at(-1)!.deformation);}
  const metrics=sim.metrics();
  return {frames,airTime:metrics.airTime,flightDistance:metrics.flightDistance,status:sim.status,peakFlightDeformation:peak};
}
export interface Comparison {
  label:string; design:string;
  reference:{airTime:number;flightDistance:number;peakDeformation:number;status:string};
  game:{airTime:number;flightDistance:number;peakDeformation:number;status:string};
  /** Distance between the two centres of mass at fixed times after release (m). */
  separation:Record<string,number|null>;
  /** Angle between the two nose directions half a second after release (degrees). */
  attitude:number|null;
}
const CHECKPOINTS=[.25,.5,1];
export function compare(reference:ReferenceResult,game:GameFlight):Comparison {
  const column=(name:string)=>reference.columns.indexOf(name),x=column('x'),fx=column('forwardX'),deformation=column('deformation');
  const at=(time:number)=>{
    const i=Math.round(time/reference.frameSeconds),a=reference.frames[i],b=game.frames[i];
    // Only while both are still in the air.
    return a&&b&&time<=Math.min(reference.airTime,game.airTime)?{a,b}:undefined;
  };
  const separation:Record<string,number|null>={};
  for(const time of CHECKPOINTS){const pair=at(time);separation[time]=pair?length(sub(pair.a.slice(x,x+3) as V3,pair.b.center)):null;}
  const half=at(.5),airborne=reference.frames.filter(frame=>frame[0]<=reference.airTime);
  return {label:reference.case.label,design:reference.case.design,
    reference:{airTime:reference.airTime,flightDistance:reference.flightDistance,peakDeformation:Math.max(...airborne.map(frame=>frame[deformation])),status:reference.status},
    game:{airTime:game.airTime,flightDistance:game.flightDistance,peakDeformation:game.peakFlightDeformation,status:game.status},
    separation,attitude:half?Math.acos(Math.min(1,dot(half.a.slice(fx,fx+3) as V3,half.b.forward)))*180/Math.PI:null};
}

function parse(args:string[]){
  const only:string[]=[],overrides:Record<string,unknown>={},gameOnly:Record<string,number>={},options:SolverOptions={};
  let speeds:number[]|undefined,refresh=false,storedOnly=false,json:string|undefined,jobs=Math.max(1,availableParallelism()-2);
  const assign=(target:Record<string,unknown>,text:string)=>{const [key,value]=text.split('=');target[key]=value.includes(',')?value.split(',').map(Number):Number.isNaN(Number(value))?value:Number(value);};
  for(let i=0;i<args.length;i++){
    const arg=args[i],next=()=>{const value=args[++i];if(value===undefined)throw Error(`${arg} needs a value`);return value;};
    if(arg==='--speeds')speeds=next().split(',').map(Number);
    else if(arg==='--speed')overrides.speed=Number(next());
    else if(arg==='--material')overrides.material=next();
    else if(arg==='--set')assign(overrides,next());
    else if(arg==='--game')assign(gameOnly,next());
    else if(arg==='--refresh')refresh=true;
    else if(arg==='--stored-only')storedOnly=true;
    else if(arg==='--json')json=next();
    else if(arg==='--jobs')jobs=Number(next());
    else if(arg==='--steps-per-frame')options.stepsPerFrame=Number(next());
    else if(arg==='--aero-stations')options.aeroStations=Number(next());
    else if(arg==='--aero-chord')options.aeroChord=Number(next());
    else if(arg==='--patch-area')options.patchArea=Number(next());
    else if(arg==='--contact-stiffness')options.contactStiffness=Number(next());
    else if(arg==='--after-touchdown')options.afterTouchdown=Number(next());
    else if(arg==='--first-opening-sides')options.firstOpeningSides=true;
    else if(arg.startsWith('--'))throw Error(`Unknown option ${arg}`);
    else if(!designs.some(design=>design.id===arg))throw Error(`Unknown design ${arg}; choose from ${designs.map(design=>design.id).join(', ')}`);
    else only.push(arg);
  }
  return {only,overrides,gameOnly,options,speeds,refresh,storedOnly,json,jobs};
}
const mm=(metres:number)=>(metres*1000).toFixed(1),cm=(metres:number|null)=>metres===null?'–':(metres*100).toFixed(1);
async function main(){
  const {only,overrides,gameOnly,options,speeds,refresh,storedOnly,json,jobs}=parse(process.argv.slice(2));
  const list:Job[]=[];
  for(const design of designs){
    if(only.length&&!only.includes(design.id))continue;
    for(const speed of speeds??[undefined]){
      const settings=speed===undefined?overrides:{...overrides,speed};
      list.push({test:caseFor(design.id,settings as Partial<Parameters>),overrides:settings,options});
    }
  }
  const missing=list.filter(job=>!stored(job));
  if(storedOnly&&missing.length)throw Error(`No current stored reference flight for: ${missing.map(job=>job.test.design).join(', ')}`);
  if(refresh||missing.length)console.error(`Solving ${refresh?list.length:missing.length} reference flight(s), ${jobs} at a time…`);
  const references=await solveAll(list,jobs,refresh,line=>{if(/after|error/i.test(line))console.error(line);});
  const rows=references.map((reference,i)=>compare(reference,flyGame(list[i].test,gameOnly as Partial<Parameters>)));
  const table=rows.map(row=>({
    flight:row.label,
    'ref m':row.reference.flightDistance.toFixed(2),'game m':row.game.flightDistance.toFixed(2),
    'Δ range %':(100*(row.game.flightDistance/row.reference.flightDistance-1)).toFixed(0),
    'ref air s':row.reference.airTime.toFixed(2),'game air s':row.game.airTime.toFixed(2),
    'apart cm @0.25s':cm(row.separation[.25]),'@0.5s':cm(row.separation[.5]),'@1s':cm(row.separation[1]),
    'nose ° @0.5s':row.attitude===null?'–':row.attitude.toFixed(1),
    'flex mm ref':mm(row.reference.peakDeformation),'flex mm game':mm(row.game.peakDeformation),
  }));
  console.table(table);
  const mean=(values:number[])=>values.length?values.reduce((a,b)=>a+b,0)/values.length:NaN;
  const range=rows.map(row=>Math.abs(row.game.flightDistance/row.reference.flightDistance-1)),apart=rows.map(row=>row.separation[.5]).filter((v):v is number=>v!==null);
  const summary={flights:rows.length,meanRangeErrorPercent:Number((100*mean(range)).toFixed(1)),worstRangeErrorPercent:Number((100*Math.max(...range)).toFixed(1)),meanSeparationAtHalfSecondCm:Number((100*mean(apart)).toFixed(1))};
  console.log(`Range error vs reference: mean ${summary.meanRangeErrorPercent}%, worst ${summary.worstRangeErrorPercent}%. Mean separation at 0.5 s: ${summary.meanSeparationAtHalfSecondCm} cm.`);
  const unconverged=references.filter(reference=>reference.solver.worstNewton<0);
  if(unconverged.length)console.log(`Note: the reference missed its Newton tolerance on some steps of ${unconverged.map(reference=>reference.case.label).join(', ')}.`);
  if(json)writeFileSync(json,JSON.stringify({summary,gameOnly,rows},null,1));
}
if(process.argv[1]&&import.meta.url.endsWith(process.argv[1].split('/').pop()!))main().catch(error=>{console.error(error instanceof Error?error.message:error);process.exit(1);});
