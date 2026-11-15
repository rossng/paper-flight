// Builds the flight solver (wasm/solver, Rust) to WebAssembly and puts it at
// src/physics/solver.wasm, which src/physics/solver.ts loads in the game and in tests. The crate has no
// dependencies; it needs cargo and the wasm32-unknown-unknown target:
//   rustup target add wasm32-unknown-unknown
import { execFileSync } from 'node:child_process';
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { homedir } from 'node:os';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const here=dirname(fileURLToPath(import.meta.url)),crate=join(here,'solver');
const env={...process.env,PATH:`${process.env.PATH}:${join(homedir(),'.cargo','bin')}`};
const cargo=(...args)=>{
  try{execFileSync('cargo',['build','--release','--target','wasm32-unknown-unknown',...args],{env,cwd:crate,stdio:['ignore','inherit','inherit']});}
  catch(error){
    console.error(error.code==='ENOENT'
      ?'\nThe flight solver is written in Rust. Install a toolchain from https://rustup.rs, then run:\n  rustup target add wasm32-unknown-unknown\n'
      :'\nBuilding the flight solver failed. If the wasm32 target is missing: rustup target add wasm32-unknown-unknown\n');
    process.exit(1);
  }
};
// Rewrite only on change, so watchers and caches see a new solver when there is one.
const publish=(wasm,target)=>{
  const bytes=readFileSync(wasm);
  if(!existsSync(target)||!readFileSync(target).equals(bytes))writeFileSync(target,bytes);
};
cargo();
publish(join(crate,'target/wasm32-unknown-unknown/release/paperflight_wasm_solver.wasm'),join(here,'../src/physics/solver.wasm'));
