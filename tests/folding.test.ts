import test from 'node:test';
import assert from 'node:assert/strict';
import {build,inspect,exportFold,fold} from '../src/folding/model';
import {recipes} from '../src/folding/recipes';
import {FoldSequence} from '../src/folding/animation';
for(const recipe of recipes)test(`${recipe.id}: animated folds preserve material and reverse through identical states`,()=>{
 const sequence=new FoldSequence(recipe);
 assert.deepEqual(sequence.sample(0),build(recipe,0));
 assert.deepEqual(sequence.sample(recipe.operations.length),build(recipe));
 for(let i=0;i<recipe.operations.length;i++){
  for(const fraction of [.25,.5,.75]){
   const state=sequence.sample(i+fraction),report=inspect(state);
   assert.ok(report.maxStrain<1e-7,`rigid panels at ${i+fraction}`);
   assert.ok(report.seamGap<1e-7,`connected paper at ${i+fraction}: ${report.seamGap}`);
   assert.ok(Math.abs(report.area-recipe.width*recipe.height)<1e-10);
   assert.ok(report.manifold);assert.equal(report.euler,1);
   assert.deepEqual(state,sequence.sample(i+1-(1-fraction)));
  }
 }
 // Sampling intermediate states cannot mutate the committed recipe or final flight shape.
 assert.deepEqual(sequence.sample(recipe.operations.length),build(recipe));
});
for(const recipe of recipes)test(`${recipe.id}: every fold preserves sheet area, rigidity, seams and disk topology`,()=>{
 for(let step=0;step<=recipe.operations.length;step++){
  const state=build(recipe,step),m=inspect(state);
  assert.ok(Math.abs(m.area-recipe.width*recipe.height)<1e-10,`area at ${step}`);
  assert.ok(m.maxStrain<1e-7,`strain at ${step}: ${m.maxStrain}`);
  assert.ok(m.seamGap<1e-7,`torn seam at ${step}: ${m.seamGap}`);
  assert.ok(m.manifold,`nonmanifold at ${step}`);assert.equal(m.euler,1,`sheet disk topology at ${step}`);
  assert.ok(state.panels.flatMap(p=>p.points).every(p=>p.position.every(Number.isFinite)));
 }
 const final=build(recipe),data=exportFold(final);assert.equal(data.file_frames[0].vertices_coords.length,data.vertices_coords.length);
 assert.ok(final.panels.some(p=>p.points.some(v=>Math.abs(v.position[2])>0.01)),'wings open in 3D');
});
test('Stable locking fold explicitly leaves underlying paper in place',()=>{
 const recipe=recipes[2],index=recipe.operations.findIndex(op=>op.id==='nose-lock'),before=build(recipe,index),after=fold(before,recipe.operations[index]);
 const untouched=before.panels.filter(p=>!p.tags.includes('nose-down'));assert.ok(untouched.length);
 for(const panel of untouched)assert.deepEqual(after.panels.find(p=>p.id===panel.id),panel);
 assert.ok(after.panels.some(p=>p.tags.includes('nose-lock')));
});

test('reference-sized studies have distinct, symmetric deployed silhouettes',()=>{
 // Approximate published Letter-paper dimensions, not a flight validation.
 const targets=[[.28,.11],[.295,.111],[.17,.125]];
 recipes.forEach((r,i)=>{const p=build(r).panels.flatMap(f=>f.points.map(v=>v.position));const span=Math.max(...p.map(v=>v[2]))-Math.min(...p.map(v=>v[2]));const chord=Math.max(...p.map(v=>v[1]))-Math.min(...p.map(v=>v[1]));assert.ok(Math.abs(chord/targets[i][0]-1)<.05);assert.ok(Math.abs(span/targets[i][1]-1)<.05);assert.ok(Math.abs(Math.max(...p.map(v=>v[2]))+Math.min(...p.map(v=>v[2])))<1e-10);});
});
test('FOLD export does not weld separate paper layers at coincident world positions',()=>{
 const data=exportFold(build(recipes[0],3)),world=data.file_frames[0].vertices_coords,uv=data.vertices_coords;
 assert.ok(world.some((p,i)=>world.some((q,j)=>i!==j&&Math.hypot(...p.map((x,k)=>x-q[k]))<1e-9&&Math.hypot(uv[i][0]-uv[j][0],uv[i][1]-uv[j][1])>1e-3)));
});
test('unsafe layer selection is rejected instead of silently tearing connected paper',async()=>{
 const {checkedFold}=await import('../src/folding/model');
 const state=build(recipes[0],3),before=JSON.stringify(state);
 assert.throws(()=>checkedFold(state,{id:'tear',label:'Invalid isolated flap rotation',axis:[[-.2,.05,0],[.2,.05,0]],movingNormal:[0,1,0],angleDegrees:90,layers:{includeTag:'left-corner'}}),/disconnect/);
 assert.equal(JSON.stringify(state),before);
});
