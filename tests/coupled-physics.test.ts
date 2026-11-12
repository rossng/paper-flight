import test from 'node:test';
import assert from 'node:assert/strict';
import { buildLiftingSurface, solveLiftingLine } from '../src/physics/lifting-surface';
import { aerodynamicLoads } from '../src/physics/aerodynamics';
import { buildContactCandidates, detectLayerContacts, solveLayerContact } from '../src/physics/layer-contact';
import { sheetMaterial, bendingRigidity, directionalYoung, edgeCompliance, prepareMaterialGeometry, updateCreasePlasticity } from '../src/physics/sheet-material';
import { createDart, type PaperMesh, type Hinge } from '../src/geometry/dart';
import { add, sub, scale, cross, dot, length, type V3 } from '../src/physics/math';
import { Simulation } from '../src/physics/simulation';
import { materialSettings } from '../src/physics/materials';
import { designMesh } from '../src/folding/design';
import { FlightRecording } from '../src/rendering/flight-recording';

import { rectangle } from './fixtures/lifting-wing';
const zeros=(p:V3[])=>p.map(()=>[0,0,0] as V3);
const total=(v:V3[])=>v.reduce(add,[0,0,0]);
const close=(a:V3,b:V3,tol=1e-10)=>assert.ok(length(sub(a,b))<tol,`${a} != ${b}`);
const settings={enabled:true,density:1.225,wind:[0,0,0] as V3};
function loads(mesh:PaperMesh,positions=mesh.rest,velocity:V3=[8,-.6,0]){
  const forces=zeros(positions),panels=aerodynamicLoads(positions,positions.map(()=>velocity),mesh.faces,{...settings,surface:buildLiftingSurface(mesh)},forces);
  return {forces,panels};
}
test('exposed quadrature preserves area, total force and all three moments',()=>{
 const mesh=createDart(),{forces,panels}=loads(mesh);
 close(total(forces),total(panels.map(p=>p.force)));
 close(total(forces.map((f,i)=>cross(mesh.rest[i],f))),total(panels.map(p=>cross(p.center,p.force))));
 const physical=mesh.faces.reduce((sum,[a,b,c])=>sum+length(cross(sub(mesh.rest[b],mesh.rest[a]),sub(mesh.rest[c],mesh.rest[a])))/2,0);
 assert.ok(Math.abs(panels.reduce((s,p)=>s+p.area,0)-physical)<1e-12);
});
test('coupled loads are winding, rigid-transform and Galilean invariant',()=>{
 const mesh=rectangle(),surface=buildLiftingSurface(mesh),p=mesh.rest,v:V3=[8,-.6,0];
 const baseline=loads(mesh).forces;
 const reversed={...mesh,faces:mesh.faces.map(([a,b,c])=>[a,c,b] as [number,number,number])};
 loads(reversed).forces.forEach((f,i)=>close(f,baseline[i]));
 const rotate=([x,y,z]:V3):V3=>[z,x,y],boost:V3=[2,-3,4],translated=p.map(a=>add(rotate(a),[3,8,-2]));
 const f=zeros(p);aerodynamicLoads(translated,p.map(()=>add(rotate(v),boost)),mesh.faces,{...settings,wind:boost,surface},f);
 f.forEach((a,i)=>close(a,rotate(baseline[i])));
});
test('span interaction changes neighbouring loads and reduces circulation at free tips',()=>{
 const bounds=Array.from({length:17},(_,i)=>i/16-0.5),chords=Array(16).fill(.2),speed=Array(16).fill(8),alpha=Array(16).fill(.08);
 const base=solveLiftingLine(bounds,chords,speed,alpha);alpha[8]+=.05;
 const perturbed=solveLiftingLine(bounds,chords,speed,alpha);
 assert.ok(base.circulation[0]<base.circulation[7]*.8);
 assert.ok(Math.abs(perturbed.circulation[5]-base.circulation[5])>1e-4);
 assert.ok(base.downwash.every(w=>w>0));
});
test('higher aspect ratio increases lift slope and decreases induced drag per lift squared',()=>{
 const result=(span:number)=>{const bounds=Array.from({length:25},(_,i)=>span*(i/24-.5)),c=Array(24).fill(.2),v=Array(24).fill(8),a=Array(24).fill(.08),r=solveLiftingLine(bounds,c,v,a);return {cl:2*r.circulation.reduce((s,g)=>s+g,0)/24/(8*.2),cd:2*r.circulation.reduce((s,g,i)=>s+g*r.downwash[i],0)/24/(64*.2)};};
 const low=result(.4),high=result(1.6);assert.ok(high.cl>low.cl*1.5);assert.ok(high.cd/high.cl**2<low.cd/low.cl**2*.5);
});
test('elliptic reference approaches lifting-line analytic lift and induced drag',()=>{
 const run=(n:number)=>{const b=2,c0=.4,alpha=.05,V=8,bounds=Array.from({length:n+1},(_,i)=>-b/2+b*(1-Math.cos(Math.PI*i/n))/2),c=Array.from({length:n},(_,i)=>c0*Math.sqrt(1-((bounds[i]+bounds[i+1])/b)**2)),r=solveLiftingLine(bounds,c,c.map(()=>V),c.map(()=>alpha));let L=0,D=0,S=0;for(let i=0;i<n;i++){const dz=bounds[i+1]-bounds[i];L+=V*r.circulation[i]*dz;D+=r.circulation[i]*r.downwash[i]*dz;S+=c[i]*dz;}return {cl:2*L/(V*V*S),cd:2*D/(V*V*S),ar:b*b/S};};
 const coarse=run(8),fine=run(64),expected=2*Math.PI*.05/(1+2/fine.ar);
 assert.ok(Math.abs(fine.cl-expected)<.015*expected);
 assert.ok(Math.abs(fine.cl-expected)<Math.abs(coarse.cl-expected));
 assert.ok(Math.abs(fine.cd-fine.cl**2/(Math.PI*fine.ar))<.025*fine.cd);
});
test('reverse, edge-on, high incidence, rest and tumbling loads are finite and dissipative',()=>{
 const mesh=rectangle(),surface=buildLiftingSurface(mesh);
 for(const velocity of [[-8,-1,0],[0,-8,0],[.01,0,8],[0,0,0],[8,-5,0]] as V3[]){
  const force=zeros(mesh.rest),panels=aerodynamicLoads(mesh.rest,mesh.rest.map(()=>velocity),mesh.faces,{...settings,surface},force);
  assert.ok(force.flat().every(Number.isFinite));assert.ok(dot(total(force),velocity)<=1e-10);
  if(length(velocity)===0)assert.equal(length(total(force)),0);
  assert.ok(panels.every(p=>Math.abs(dot(p.lift,velocity))<1e-10));
 }
 const velocities=mesh.rest.map(p=>cross([30,20,10],p)),force=zeros(mesh.rest);
 aerodynamicLoads(mesh.rest,velocities,mesh.faces,{...settings,surface},force);
 assert.ok(force.flat().every(Number.isFinite));
 assert.ok(force.reduce((power,f,i)=>power+dot(f,velocities[i]),0)<=1e-10);
});
test('dynamic exposed area shares coincident layers and recovers area after peeling',()=>{
 const mesh=rectangle(),count=mesh.rest.length,area=mesh.faces.length;
 const doubled={...mesh,rest:[...mesh.rest,...mesh.rest.map(p=>[...p] as V3)],faces:[...mesh.faces,...mesh.faces.map(f=>f.map(i=>i+count) as [number,number,number])],faceOwners:[...Array(area).fill(0),...Array(area).fill(1)]};
 const original=loads(mesh),shared=loads(doubled);
 close(total(original.forces),total(shared.forces),1e-9);
 const open=doubled.rest.map((p,i)=>add(p,[0,i>=count?.01:0,0]));
 const peeled=loads(doubled,open);
 assert.ok(Math.abs(peeled.panels.reduce((s,p)=>s+p.area,0)/shared.panels.reduce((s,p)=>s+p.area,0)-2)<1e-10);
});
test('contact blocks penetration, permits separation and sliding, and preserves mass centre',()=>{
 const old:V3[]=[[0,0,0],[0,0,1],[1,0,0],[.2,0,.2]],p=old.map(v=>[...v] as V3);p[3]=[.3,-.02,.2];
 const candidate={vertex:3,face:[0,1,2] as [number,number,number],sign:1},w=[1,2,3,4];
 const contact=detectLayerContacts(p,old,[candidate])[0];assert.ok(contact);
 const center=(a:V3[])=>total(a.map((v,i)=>scale(v,1/w[i]))),before=center(p);
 solveLayerContact(p,w,contact,0);close(center(p),before);
 const relative=sub(p[3],total(contact.face.map((id,j)=>scale(p[id],contact.weights[j]))));
 assert.ok(Math.abs(dot(relative,contact.normal))<1e-12);assert.equal(p[3][0],.3);
 const opened=old.map(v=>[...v] as V3);opened[3][1]=.01;assert.equal(detectLayerContacts(opened,old,[candidate]).length,0);
});
test('Coulomb friction opposes slip, is pressure limited, and does not glue uncompressed layers',()=>{
 const old:V3[]=[[0,0,0],[0,0,1],[1,0,0],[.2,0,.2]],candidate={vertex:3,face:[0,1,2] as [number,number,number],sign:1};
 for(const penetration of [0,.02]){
  const p=old.map(v=>[...v] as V3);p[3]=[.3,-penetration,.2];const contact=detectLayerContacts(p,old,[candidate])[0];
  solveLayerContact(p,[0,0,0,1],contact,.4);
  assert.ok(length(contact.tangent)<=.4*contact.lambda+1e-12);
  if(penetration)assert.ok(p[3][0]<.3&&p[3][0]>.2);else assert.equal(p[3][0],.3);
 }
});
test('contact candidates persist across adjacent triangles during sliding',()=>{
 const mesh:PaperMesh={...rectangle(),rest:[[0,0,0],[0,0,1],[1,0,0],[1,0,1],[.2,0,.2],[.3,0,.2],[.2,0,.3]],faces:[[0,1,2],[2,1,3],[4,5,6]],faceOwners:[0,0,1]};
 const candidates=buildContactCandidates(mesh),old=mesh.rest.map(v=>[...v] as V3);old[4][1]=.0001;detectLayerContacts(old,old,candidates);const p=old.map(v=>[...v] as V3);p[4]=[.8,-.01,.8];
 // Owner 1 is on the positive side of the canonical +Y normal.
 const contacts=detectLayerContacts(p,old,candidates).filter(c=>c.vertex===4);
 assert.ok(contacts.some(c=>c.face.includes(3)));for(const c of contacts)solveLayerContact(p,p.map(()=>1),c,0);
 assert.ok(p[4][1]>-.01);
});
test('density, thickness, directional elastic moduli and mesh geometry set stiffness',()=>{
 const m=sheetMaterial('paper',80),heavy=sheetMaterial('paper',160),mesh=createDart();prepareMaterialGeometry(mesh);
 assert.equal(m.density*m.thickness,.08);
 assert.ok(Math.abs(bendingRigidity(heavy)/bendingRigidity(m)-8)<1e-12);
 assert.ok(Math.abs(directionalYoung(m,0)/m.youngMD-1)<1e-12);assert.ok(Math.abs(directionalYoung(m,Math.PI/2)/m.youngCD-1)<1e-12);
 assert.ok(Math.abs(edgeCompliance(heavy,mesh.edges[0])/edgeCompliance(m,mesh.edges[0])-.5)<1e-12);
 const metal=sheetMaterial('steel',7850);assert.equal(metal.thickness,.001);assert.ok(bendingRigidity(metal)>bendingRigidity(m)*10000);
 const scaled={...mesh,rest:mesh.rest.map(p=>scale(p,2)),edges:mesh.edges.map(e=>({...e,rest:e.rest*2}))};prepareMaterialGeometry(scaled);
 assert.ok(Math.abs(edgeCompliance(m,mesh.edges[0])/edgeCompliance(m,scaled.edges[0])-1)<1e-12);
});
test('crease plasticity has an elastic range, permanent set, dissipative yielding and reset',()=>{
 const m=sheetMaterial('paper',80),h:Hinge={ids:[0,1,2,3],rest:0,crease:true,weight:1,lambda:0};
 updateCreasePlasticity(h,.1,m);assert.equal(h.plasticOffset,undefined);
 updateCreasePlasticity(h,1,m);assert.ok(h.plasticOffset!>.5);assert.ok((1-h.plasticOffset!)**2<1);
 const offset=h.plasticOffset;updateCreasePlasticity(h,offset!,m);assert.equal(h.plasticOffset,offset);
 const s=new Simulation();s.mesh.hinges[0].plasticOffset=.4;s.reset();assert.equal(s.mesh.hinges[0].plasticOffset,0);
});
test('replay recomputes coupled exposure, circulation and weather from its detached frame',()=>{
 const s=new Simulation({gusts:1,turbulence:.5}),r=new FlightRecording();s.launch();r.capture(s);s.step();r.capture(s);
 const frame=r.sample(s.time)!,p=Array.from({length:s.positions.length},(_,i)=>Array.from(frame.positions.slice(i*3,i*3+3)) as V3),v=Array.from({length:s.positions.length},(_,i)=>Array.from(frame.velocities.slice(i*3,i*3+3)) as V3);
 const replay=aerodynamicLoads(p,v,s.mesh.faces,s.aeroSettings(frame.time),zeros(p));
 const live=aerodynamicLoads(s.positions,s.velocities,s.mesh.faces,s.aeroSettings(s.time),zeros(p));
 close(total(replay.map(p=>p.force)),total(live.map(p=>p.force)),1e-6);
 const before=structuredClone(replay);s.step();assert.deepEqual(replay,before);
});
test('contact normal corrections are covariant and carry no net force or torque',()=>{
 const old:V3[]=[[0,0,0],[0,0,1],[1,0,0],[.2,0,.2]],p=old.map(v=>[...v] as V3);p[3][1]=-.01;
 const candidate={vertex:3,face:[0,1,2] as [number,number,number],sign:1},w=[1,2,3,4],before=structuredClone(p);
 solveLayerContact(p,w,detectLayerContacts(p,old,[candidate])[0],0);
 const impulses=p.map((v,i)=>scale(sub(v,before[i]),1/w[i]));close(total(impulses),[0,0,0]);close(total(impulses.map((v,i)=>cross(before[i],v))),[0,0,0]);
 const transform=([x,y,z]:V3):V3=>[3+z,2+x,-1+y],q=before.map(transform);
 solveLayerContact(q,w,detectLayerContacts(q,old.map(transform),[candidate])[0],0);
 q.forEach((v,i)=>close(v,transform(p[i])));
});
test('crease reversal accumulates hardening instead of erasing permanent history',()=>{
 const m=sheetMaterial('foil',135),h:Hinge={ids:[0,1,2,3],rest:0,crease:true,weight:1,lambda:0};
 updateCreasePlasticity(h,.7,m);const accumulated=h.plasticAccumulated!;
 updateCreasePlasticity(h,-.7,m);assert.ok(h.plasticAccumulated!>accumulated);assert.ok(h.plasticOffset!<0);
});
test('material coordinates preserve paper grain across compiled refinements',()=>{
 const mesh=designMesh('stable',new Simulation().parameters);prepareMaterialGeometry(mesh);
 for(const e of mesh.edges){const [a,b]=[mesh.materialUV![e.a],mesh.materialUV![e.b]];assert.ok(Math.abs(Math.hypot(a[0]-b[0],a[1]-b[1])-e.rest)<1e-8);}
 assert.ok(mesh.edges.some(e=>Math.abs(Math.sin(e.materialDirection!))>.8));assert.ok(mesh.edges.some(e=>Math.abs(Math.cos(e.materialDirection!))>.8));
});
test('membrane shear response is rigid-motion invariant and preserves centre of mass',async()=>{
 const {buildMembranes,solveMembrane}=await import('../src/physics/membrane');
 const mesh=rectangle(1,.2,2),material=sheetMaterial('paper',80),tri=buildMembranes(mesh)[0],p=structuredClone(mesh.rest);
 const a=tri.face[0];p[a][2]+=.002;
 const before=structuredClone(p),center=total(p),w=p.map(()=>1);
 solveMembrane(p,w,tri,material,.001);close(total(p),center);
 assert.ok(length(sub(p[a],before[a]))>1e-5);
 const transform=([x,y,z]:V3):V3=>[4+z,5+x,6+y],q=before.map(transform),other=buildMembranes(mesh)[0];
 solveMembrane(q,w,other,material,.001);q.forEach((v,i)=>close(v,transform(p[i])));
 const rest=structuredClone(mesh.rest),original=structuredClone(rest);
 solveMembrane(rest,w,buildMembranes(mesh)[0],material,.001);rest.forEach((v,i)=>close(v,original[i]));
});
test('deforming one wing changes loads on the untouched wing through circulation',()=>{
 const mesh=rectangle(1,.2,12),base=loads(mesh),twisted=mesh.rest.map(([x,y,z])=>[x,y+(z>0?.08*x*z:0),z] as V3),changed=loads(mesh,twisted);
 const left=(panels:typeof base.panels)=>total(panels.filter(p=>p.center[2]<-.1).map(p=>p.lift));
 assert.ok(length(sub(left(base.panels),left(changed.panels)))>1e-4);
 assert.deepEqual(twisted.filter(p=>p[2]<0),mesh.rest.filter(p=>p[2]<0));
});
test('contact ignores the legacy laminate multiplier while bonded mode remains explicit',()=>{
 const mesh=designMesh('basic',new Simulation().parameters),bondedMetadata=structuredClone(mesh);
 bondedMetadata.hinges.forEach(h=>h.layerStiffness=100);
 const a=new Simulation({},mesh),b=new Simulation({},bondedMetadata);a.launch();b.launch();for(let i=0;i<3;i++){a.step();b.step();}
 assert.deepEqual(a.positions,b.positions);assert.ok(a.contactCandidates.length>0);
 const before=structuredClone(a.positions);a.launch();for(let i=0;i<3;i++)a.step();assert.deepEqual(a.positions,before,'reset must clear contact side history');
});
