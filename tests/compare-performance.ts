import assert from 'node:assert/strict';
import { readFileSync, writeFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';

// Paired benchmark evidence includes trajectories, all vertex forces and
// velocities, deformation/strain, crease plasticity and contact diagnostics.
const input=process.argv[2]??'docs/performance/paired-final.json';
const states=JSON.parse(gunzipSync(readFileSync(input+'.states.json.gz')).toString());
const results=[];
for(const before of states.filter((s:any)=>s.variant==='baseline')){
 const after=states.find((s:any)=>s.variant==='candidate'&&s.name===before.name);
 assert.ok(after,`Missing candidate ${before.name}`);
 const fields:Record<string,{values:number;maxAbsoluteDifference:number}>={};
 const compare=(a:any,b:any,path:string,field:string)=>{
  if(typeof a==='number'){
   assert.ok(Number.isFinite(a)&&Number.isFinite(b),`Nonfinite ${path}`);
   const difference=Math.abs(a-b),summary=fields[field]??={values:0,maxAbsoluteDifference:0};
   summary.values++;summary.maxAbsoluteDifference=Math.max(summary.maxAbsoluteDifference,difference);
   assert.ok(difference<=1e-12,`${before.name} ${path}: ${a} vs ${b}`);
  }else if(a!==null&&typeof a==='object'){
   assert.deepEqual(Object.keys(a),Object.keys(b),path);
   for(const key of Object.keys(a))compare(a[key],b[key],`${path}.${key}`,field);
  }else assert.equal(a,b,path);
 };
 assert.equal(before.snapshots.length,after.snapshots.length);
 before.snapshots.forEach((snapshot:any,i:number)=>{
  for(const field of Object.keys(snapshot))compare(snapshot[field],after.snapshots[i][field],`${i}.${field}`,field);
 });
 results.push({name:before.name,checkpoints:before.snapshots.length,fields});
}
assert.ok(results.length>0,'Expected baseline and candidate variants');
const report={input,tolerance:1e-12,results};
writeFileSync(input+'.comparison.json',JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));
