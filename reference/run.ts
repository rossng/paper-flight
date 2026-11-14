/** Builds and runs the reference solver, and keeps its results on disk.
 * A stored flight is reused while the problem, the solver options and the solver's
 * source are all unchanged, so game-side work never needs to recompute it. */
import { spawn, spawnSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, mkdtempSync, readFileSync, readdirSync, rmSync, writeFileSync } from 'node:fs';
import { homedir, tmpdir } from 'node:os';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { problemText, type Case } from './export';

const here=dirname(fileURLToPath(import.meta.url));
export const solverDirectory=join(here,'solver'),resultsDirectory=join(here,'results');
const binary=join(solverDirectory,'target','release',process.platform==='win32'?'paperflight-reference.exe':'paperflight-reference');
const environment={...process.env,PATH:`${process.env.PATH}${process.platform==='win32'?';':':'}${join(homedir(),'.cargo','bin')}`};

export interface SolverOptions { stepsPerFrame?:number; aeroStations?:number; aeroChord?:number; patchArea?:number; contactStiffness?:number; afterTouchdown?:number; maxTime?:number; firstOpeningSides?:boolean }
export interface ReferenceResult {
  case:{design:string;label:string;overrides:Record<string,unknown>};
  problemHash:string;solverHash:string;
  status:string;time:number;touchdown:{time:number;center:[number,number,number];distance:number}|null;
  airTime:number;flightDistance:number;distance:number;groundDistance:number;pathLength:number;
  peakDeformation:number;peakStrain:number;peakPenetration:number;
  options:Record<string,number|boolean>;mesh:{vertices:number;faces:number;aeroPatches:number};
  solver:{steps:number;newtonPerStep:number;worstNewton:number;lineSearchHalvings:number;regularisedSolves:number;patternRebuilds:number;wallSeconds:number};
  energy:{start:number;airWork:number;launchWork:number;dampingLoss:number;residual:number};
  frameSeconds:number;columns:string[];frames:number[][];
}
const flags=(options:SolverOptions)=>[
  ...(options.stepsPerFrame?['--steps-per-frame',String(options.stepsPerFrame)]:[]),
  ...(options.aeroStations?['--aero-stations',String(options.aeroStations)]:[]),
  ...(options.aeroChord?['--aero-chord',String(options.aeroChord)]:[]),
  ...(options.patchArea!==undefined?['--patch-area',String(options.patchArea)]:[]),
  ...(options.contactStiffness?['--contact-stiffness',String(options.contactStiffness)]:[]),
  ...(options.afterTouchdown!==undefined?['--after-touchdown',String(options.afterTouchdown)]:[]),
  ...(options.maxTime?['--max-time',String(options.maxTime)]:[]),
  ...(options.firstOpeningSides?['--first-opening-sides']:[]),
];
let digest:string|undefined;
/** Digest of the solver's source: a stored flight from other source is stale. */
export function solverHash(){
  if(!digest){
    const hash=createHash('sha256');
    for(const file of ['Cargo.toml',...readdirSync(join(solverDirectory,'src')).sort().map(name=>join('src',name))])hash.update(file).update(readFileSync(join(solverDirectory,file)));
    digest=hash.digest('hex').slice(0,16);
  }
  return digest;
}
let built=false;
export function buildSolver(){
  if(built)return binary;
  const result=spawnSync('cargo',['build','--release','--quiet'],{cwd:solverDirectory,env:environment,stdio:['ignore','inherit','inherit']});
  if(result.error||result.status!==0)throw Error(result.error?'The reference solver needs a Rust toolchain (https://rustup.rs): cargo was not found':'The reference solver failed to build');
  built=true;return binary;
}
/** File-safe name of a case: the design, its non-default settings, and any solver options. */
export function caseLabel(design:string,overrides:Record<string,unknown>,options:SolverOptions={}){
  const part=(entries:[string,unknown][])=>entries.filter(([,value])=>value!==undefined).sort(([a],[b])=>a.localeCompare(b)).map(([key,value])=>`${key}${Array.isArray(value)?value.join('_'):value}`);
  return [design,...part(Object.entries(overrides)),...part(Object.entries(options))].join('-').replace(/[^A-Za-z0-9._-]/g,'');
}
export function staticLoads(test:Case,positions:number[][],velocities:number[][],options:SolverOptions={},time=0){
  const directory=mkdtempSync(join(tmpdir(),'paperflight-reference-'));
  try{
    writeFileSync(join(directory,'problem'),problemText(test).replace('\nlandmarks',`\nscalar time ${time}\nlandmarks`));
    writeFileSync(join(directory,'state'),positions.map((p,i)=>[...p,...velocities[i]].join(' ')).join('\n'));
    const result=spawnSync(buildSolver(),['loads',join(directory,'problem'),join(directory,'state'),...flags(options)],{encoding:'utf8',maxBuffer:1<<28});
    if(result.status!==0)throw Error(`Reference solver failed: ${result.stderr}`);
    return JSON.parse(result.stdout) as {patches:number;air:number[];airForce:[number,number,number];membraneEnergy:number;bendingEnergy:number;elastic:number[]};
  }finally{rmSync(directory,{recursive:true,force:true});}
}
export interface Job { test:Case; overrides:Record<string,unknown>; options?:SolverOptions }
/** Stored reference flight for a job, or undefined when absent or stale. */
export function stored(job:Job,text=problemText(job.test)):ReferenceResult|undefined {
  const path=join(resultsDirectory,`${caseLabel(job.test.design,job.overrides,job.options)}.json`);
  if(!existsSync(path))return;
  const result=JSON.parse(readFileSync(path,'utf8')) as ReferenceResult;
  return result.problemHash===hashOf(text)&&result.solverHash===solverHash()?result:undefined;
}
const hashOf=(text:string)=>createHash('sha256').update(text).digest('hex').slice(0,16);
/** Run the solver for one job and, unless `keep` is false, store the flight. */
export function solve(job:Job,log:(line:string)=>void=()=>{},keep=true):Promise<ReferenceResult> {
  const text=problemText(job.test),label=caseLabel(job.test.design,job.overrides,job.options),executable=buildSolver();
  const directory=mkdtempSync(join(tmpdir(),'paperflight-reference-')),problem=join(directory,'problem'),output=join(directory,'result.json');
  writeFileSync(problem,text);
  return new Promise((resolve,reject)=>{
    const child=spawn(executable,['run',problem,output,...flags(job.options??{})],{stdio:['ignore','ignore','pipe']});
    let tail='',partial='';
    child.stderr.setEncoding('utf8').on('data',(chunk:string)=>{
      tail=(tail+chunk).slice(-2000);
      const lines=(partial+chunk).split('\n');partial=lines.pop()!;
      for(const line of lines)if(line.trim())log(`${label}: ${line.trim()}`);
    });
    child.on('error',reject);
    child.on('close',code=>{
      try{
        if(!existsSync(output))throw Error(`Reference solver failed for ${label}: ${tail}`);
        const result={case:{design:job.test.design,label,overrides:job.overrides},problemHash:hashOf(text),solverHash:solverHash(),...JSON.parse(readFileSync(output,'utf8'))} as ReferenceResult;
        if(code!==0||result.status==='error')throw Error(`Reference solver reported an error for ${label}: ${tail}`);
        const {frames,...summary}=result;
        if(keep){mkdirSync(resultsDirectory,{recursive:true});writeFileSync(join(resultsDirectory,`${label}.json`),`${JSON.stringify(summary,null,1).slice(0,-2)},\n "frames":[\n${frames.map(row=>JSON.stringify(row)).join(',\n')}\n]}\n`);}
        resolve(result);
      }catch(error){reject(error);}finally{rmSync(directory,{recursive:true,force:true});}
    });
  });
}
/** Stored flights where current, solving the rest `jobs` at a time. */
export async function solveAll(list:Job[],jobs:number,refresh=false,log:(line:string)=>void=()=>{}):Promise<ReferenceResult[]> {
  const results:(ReferenceResult|undefined)[]=list.map(job=>refresh?undefined:stored(job));
  const pending=list.map((_,i)=>i).filter(i=>!results[i]);
  if(pending.length)buildSolver();
  await Promise.all(Array.from({length:Math.max(1,Math.min(jobs,pending.length))},async()=>{
    for(let i=pending.shift();i!==undefined;i=pending.shift())results[i]=await solve(list[i],log);
  }));
  return results as ReferenceResult[];
}
