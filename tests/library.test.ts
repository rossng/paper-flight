import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {designs,sheetDesigns} from '../src/folding/library';
import {designFold,collectionFold,foldFilename} from '../src/folding/downloads';
import {designMesh} from '../src/folding/design';
import {build,exportFold,materialArea} from '../src/folding/model';
import {DEFAULTS} from '../src/physics/simulation';
import {readDesignSettings} from '../src/folding/settings';

test('library contains nine distinct sheet recipes and one explicitly effective design',()=>{
 assert.equal(sheetDesigns.length,9);assert.equal(designs.length,10);
 assert.equal(new Set(designs.map(d=>d.id)).size,10);
 assert.equal(new Set(sheetDesigns.map(d=>JSON.stringify(d.recipe!.operations))).size,9);
 assert.throws(()=>designFold('legacy',DEFAULTS),/effective/);
 assert.throws(()=>designFold('unknown',DEFAULTS),/effective/);
 for(const d of sheetDesigns){assert.ok(d.name&&d.detail&&d.note);assert.ok(d.recipe?.source.startsWith('https://'));}
});
for(const d of sheetDesigns)test(`${d.id}: exported frames round-trip the recipe and preserve actual flight geometry at workbench extremes`,()=>{
 for(const [size,span,dihedral,paperWeight] of [[1,1,8,80],[.85,.7,0,60],[1.15,1.35,16,110]]){
  const p={...DEFAULTS,size,span,dihedral,paperWeight},data=JSON.parse(JSON.stringify(designFold(d.id,p))),recipe=data['paperflight:recipe'];
  const flat=data.vertices_coords as number[][],folded=data.file_frames[0].vertices_coords as number[][];
  assert.equal(data.frame_unit,'m');assert.equal(data.file_frames[0].frame_unit,'m');
  assert.equal(flat.length,folded.length);assert.ok(flat.flat().every(Number.isFinite));assert.ok(folded.flat().every(Number.isFinite));
  const area=data.faces_vertices.reduce((sum:number,face:number[])=>sum+materialArea(face.map(i=>flat[i] as [number,number])),0);
  assert.ok(Math.abs(area-d.recipe!.width*d.recipe!.height*size**2)<1e-10);
  const uses=new Map<string,number>();
  for(const face of data.faces_vertices as number[][]){assert.ok(face.length>=3);assert.equal(new Set(face).size,face.length);for(let i=0;i<face.length;i++){const a=face[i],b=face[(i+1)%face.length];assert.ok(a>=0&&a<flat.length);const key=[a,b].sort((a,b)=>a-b).join(':');uses.set(key,(uses.get(key)??0)+1);}}
  assert.equal(flat.length-data.edges_vertices.length+data.faces_vertices.length,1);
  for(const [i,[a,b]] of (data.edges_vertices as number[][]).entries()){
   const n=uses.get([a,b].sort((a,b)=>a-b).join(':'));assert.ok(n===1||n===2);assert.equal(data.edges_assignment[i],n===1?'B':'U');
   assert.ok(Math.abs(Math.hypot(...flat[a].map((x,k)=>x-flat[b][k]))-Math.hypot(...folded[a].map((x,k)=>x-folded[b][k])))<1e-9);
  }
  assert.deepEqual(exportFold(build(recipe)).file_frames[0].vertices_coords,folded);
  const mesh=designMesh(d.id,p);
  // Compiler starts with the conformed recipe vertices, in flight axes (-y,x,z).
  for(let i=0;i<folded.length;i++)assert.ok(Math.hypot(mesh.rest[i][0]+folded[i][1],mesh.rest[i][1]-folded[i][0],mesh.rest[i][2]-folded[i][2])<1e-9);
  assert.ok(Math.abs(mesh.masses.reduce((a,b)=>a+b,0)-area*paperWeight/1000)<1e-12);
  assert.equal(data['paperflight:license'],readFileSync(new URL('../LICENSE',import.meta.url),'utf8'));
  assert.equal(data['paperflight:designId'],d.id);
  assert.ok(foldFilename(d.id,p).includes(`${paperWeight}gsm`));
 }
});
test('collection is 18 independent valid frame dictionaries; partial exports match their step',()=>{
 const collection=collectionFold(DEFAULTS),frames=[collection,...collection.file_frames];
 assert.deepEqual(collection.file_classes,['multiModel']);assert.equal(frames.length,18);
 for(const frame of collection.file_frames)assert.ok(Object.keys(frame).every(k=>!k.startsWith('file_')));
 for(const design of sheetDesigns){
  const pair=frames.filter(f=>f['paperflight:designId']===design.id);assert.equal(pair.length,2);
  assert.deepEqual(pair[0].frame_classes,['creasePattern']);assert.deepEqual(pair[1].frame_classes,['foldedForm']);
  const partial=designFold(design.id,DEFAULTS,2);assert.equal(partial['paperflight:appliedSteps'],2);
  assert.deepEqual(exportFold(build(partial['paperflight:recipe'],2)).file_frames[0].vertices_coords,partial.file_frames[0].vertices_coords);
 }
});
test('game URLs restore workbench settings, reject invalid numbers, and retain special materials',()=>{
 const p={...DEFAULTS,size:.85,span:1.35,dihedral:16,paperWeight:110,stiffness:(110/80)**3};
 assert.deepEqual(readDesignSettings(new URLSearchParams('design=raven&size=0.85&span=1.35&dihedral=16&paperWeight=110&material=paper')),p);
 const foil=readDesignSettings(new URLSearchParams('material=foil&paperWeight=80&size=1.15&dihedral=0'));
 assert.equal(foil.material,'foil');assert.equal(foil.paperWeight,135);assert.equal(foil.size,1.15);assert.equal(foil.dihedral,0);
 const invalid=readDesignSettings(new URLSearchParams('size=NaN&span=100&dihedral=-1&material=bogus&paperWeight=Infinity'));
 assert.equal(invalid.size,1);assert.equal(invalid.span,1);assert.equal(invalid.dihedral,DEFAULTS.dihedral);assert.equal(invalid.material,'paper');assert.equal(invalid.paperWeight,80);
});
