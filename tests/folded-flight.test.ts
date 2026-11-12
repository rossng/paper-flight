import test from 'node:test';
import assert from 'node:assert/strict';
import {build} from '../src/folding/model';
import {recipes} from '../src/folding/recipes';
import {compileFlightMesh} from '../src/folding/flight-mesh';
import {designMesh} from '../src/folding/design';
import {DEFAULTS,Simulation} from '../src/physics/simulation';
import {aerodynamicLoads} from '../src/physics/aerodynamics';
import {length,sub,cross,type V3} from '../src/physics/math';
for(const recipe of recipes)test(`${recipe.id}: flight mesh preserves folded mass distribution and local layers`,()=>{
 const state=build(recipe),mesh=compileFlightMesh(state),mass=mesh.masses.reduce((a,b)=>a+b,0);
 assert.ok(Math.abs(mass-recipe.width*recipe.height*recipe.gsm/1000)<1e-12);
 const expected:V3=[0,0,0];for(const panel of state.panels){for(let j=1;j<panel.points.length-1;j++){const tri=[panel.points[0],panel.points[j],panel.points[j+1]].map(p=>[-p.position[1],p.position[0],p.position[2]] as V3),m=length(cross(sub(tri[1],tri[0]),sub(tri[2],tri[0])))/2*recipe.gsm/1000;for(let d=0;d<3;d++)expected[d]+=m*(tri[0][d]+tri[1][d]+tri[2][d])/3/mass;}}
 const actual:V3=[0,0,0];mesh.rest.forEach((p,i)=>{for(let d=0;d<3;d++)actual[d]+=p[d]*mesh.masses[i]/mass;});assert.ok(length(sub(actual,expected))<1e-12,'folded centre of mass');
 assert.ok(Math.max(...mesh.layerCounts!)>=4);assert.ok(mesh.layerTies!.length>0);assert.ok(mesh.hinges.some(h=>(h.layerStiffness??1)>1));assert.ok(mesh.aeroWeights!.every(w=>w>0&&w<=1));
 const area=mesh.faces.reduce((sum,[a,b,c],i)=>sum+length(cross(sub(mesh.rest[b],mesh.rest[a]),sub(mesh.rest[c],mesh.rest[a])))/2*mesh.aeroWeights![i],0);assert.ok(area<recipe.width*recipe.height*.8,'hidden layers are not extra lifting surfaces');
});
test('lamination changes local stiffness without moving mass or changing the folded shape',()=>{
 const state=build(recipes[2]),loose=compileFlightMesh(state,{layerCoupling:0}),bonded=compileFlightMesh(state,{layerCoupling:1});
 assert.deepEqual(loose.rest,bonded.rest);assert.deepEqual(loose.masses,bonded.masses);assert.ok(bonded.hinges.some((h,i)=>(h.layerStiffness??1)>loose.hinges[i].layerStiffness!*2));
});
test('shared aerodynamic area does not double forces for two coincident layers',()=>{
 const positions:V3[]=[[0,0,0],[.2,0,0],[0,0,.2]],velocities:V3[]=positions.map(()=>[6,-1,0]),settings={density:1.225,wind:[0,0,0] as V3,enabled:true};const forces=()=>positions.map(()=>[0,0,0] as V3);
 const single=aerodynamicLoads(positions,velocities,[[0,1,2]],settings,forces())[0].force;
 const double=aerodynamicLoads(positions,velocities,[[0,1,2],[0,1,2]],{...settings,areaWeights:[.5,.5]},forces());const sum=single.map((_,d)=>double[0].force[d]+double[1].force[d]) as V3;assert.ok(length(sub(single,sum))<1e-12);
});
test('flex vectors exclude rigid translation and rotation, and measure a local bend',()=>{
 const s=new Simulation({pitch:23,heading:43});assert.ok(s.deformation().rms<1e-12);const vertex=s.mesh.rest.findIndex((_,i)=>![s.mesh.nose,s.mesh.tail,s.mesh.leftRoot,s.mesh.rightRoot].includes(i));s.positions[vertex][1]+=.002;const field=s.deformation();assert.ok(Math.abs(field.vectors[vertex][1]-.002)<1e-12);assert.ok(field.perVertex.filter((_,i)=>i!==vertex).every(d=>d<1e-12));
});
for(const id of ['basic','dart','stable'])test(`${id}: compiled sheet completes flight and conserves free translation`,()=>{
 const p={...DEFAULTS,speed:7,pitch:15,elevation:8},mesh=designMesh(id,p),free=new Simulation({...p,gravity:0,aero:false,height:100,launchDuration:0},mesh);free.launch();for(let i=0;i<30;i++)free.step();assert.ok(Math.abs(free.metrics().distance-7*.25*Math.cos(8*Math.PI/180))<1e-7);assert.ok(free.deformation().rms<1e-7);
 const s=new Simulation(p,mesh);s.launch();while(s.status==='flying')s.step();assert.equal(s.status,'landed');assert.ok(s.metrics().distance>1);assert.ok(s.metrics().maxStrain<.1);assert.ok(s.peakDeformation>1e-6);
});
test('folded material options retain area scaling and complete an extreme launch',()=>{
 const p={...DEFAULTS,size:.85,span:1.35,paperWeight:60,stiffness:(60/80)**3,dihedral:16,pitch:35,elevation:16,heading:-70,speed:9},mesh=designMesh('stable',p),s=new Simulation(p,mesh);
 assert.ok(Math.abs(mesh.masses.reduce((a,b)=>a+b,0)-recipes[2].width*recipes[2].height*.85**2*.060)<1e-12);s.launch();while(s.status==='flying')s.step();assert.equal(s.status,'landed');assert.ok(s.positions.flat().every(Number.isFinite));
});

test('greater layer bending coupling reduces flex under the same launch',()=>{
 const state=build(recipes[1]),settings={speed:7,pitch:15,elevation:8,layerModel:'bonded' as const};
 const loose=new Simulation(settings,compileFlightMesh(state,{layerCoupling:0})),bonded=new Simulation(settings,compileFlightMesh(state,{layerCoupling:1}));
 loose.launch();bonded.launch();for(let i=0;i<48;i++){loose.step();bonded.step();}
 assert.deepEqual(loose.mesh.masses,bonded.mesh.masses);assert.ok(bonded.deformation().rms<loose.deformation().rms*.9);assert.ok(bonded.metrics().bend<loose.metrics().bend);
});
