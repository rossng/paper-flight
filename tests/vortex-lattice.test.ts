import test from 'node:test';
import assert from 'node:assert/strict';
import { buildLiftingSurface, solveVortexLattice } from '../src/physics/lifting-surface';
import { aerodynamicLoads } from '../src/physics/aerodynamics';
import { cross, sub, type V3 } from '../src/physics/math';
import { rectangle } from './fixtures/lifting-wing';

const cosine=(n:number,b:number)=>Array.from({length:n+1},(_,i)=>-b/2+b*(1-Math.cos(Math.PI*i/n))/2);
/** Lift coefficient and centre of pressure (body x, forward positive) of a flat planform. */
function lattice(span:number,section:(y:number)=>[lead:number,chord:number],n=24,m=4,alpha=.05,deflection=(_bin:number)=>0){
 const bounds=cosine(n,span),count=n*m,lead=new Float64Array(count),chord=new Float64Array(count),speed=new Float64Array(count).fill(8),incidence=new Float64Array(count),attached=new Float64Array(count).fill(1);
 for(let i=0;i<n;i++){const [le,c]=section((bounds[i]+bounds[i+1])/2);for(let j=0;j<m;j++){lead[i*m+j]=le-c*j/m;chord[i*m+j]=c/m;incidence[i*m+j]=alpha+deflection(j);}}
 const circulation=solveVortexLattice(bounds,m,lead,chord,speed,incidence,attached);
 let lift=0,area=0,moment=0;
 for(let p=0;p<count;p++){const i=Math.floor(p/m),dy=bounds[i+1]-bounds[i],l=8*circulation[p]*dy;lift+=l;area+=chord[p]*dy;moment+=l*(lead[p]-chord[p]/4);}
 return {cl:2*lift/(64*area),xcp:moment/lift};
}

test('vortex lattice recovers the two-dimensional lift slope and quarter-chord centre',()=>{
 const r=lattice(400,()=>[0,1]);
 assert.ok(Math.abs(r.cl/(2*Math.PI*.05)-1)<.03,`CL/(2 pi alpha) ${r.cl/(2*Math.PI*.05)}`);
 assert.ok(Math.abs(r.xcp+.25)<.005,`xcp ${r.xcp}`);
});
test('vortex lattice matches the Helmbold low-aspect-ratio lift slope for rectangles',()=>{
 for(const ar of [1,2,6]){
  const r=lattice(ar*.2,()=>[0,.2]),helmbold=2*Math.PI*ar/(2+Math.sqrt(ar*ar+4))*.05;
  assert.ok(Math.abs(r.cl/helmbold-1)<.05,`AR ${ar}: ${r.cl} vs ${helmbold}`);
  // Low aspect ratio moves the rectangular centre of pressure ahead of the quarter chord.
  assert.ok(-r.xcp/.2>.15&&-r.xcp/.2<.26,`AR ${ar} xcp/c ${-r.xcp/.2}`);
 }
});
test('delta centre of pressure lies between strip quarter-chord and slender-body limits',()=>{
 const root=.3,span=.225;
 const r=lattice(span,y=>{const c=root*(1-2*Math.abs(y)/span);return [-(root-c),c];},24,6);
 assert.ok(-r.xcp/root>.55&&-r.xcp/root<.667,`xcp/c0 ${-r.xcp/root}`);
});
test('a raised trailing edge gives a nose-up moment and less lift',()=>{
 const base=lattice(1,()=>[0,.2],24,4,.05),flap=lattice(1,()=>[0,.2],24,4,.05,bin=>bin===3?-10*Math.PI/180:0);
 assert.ok(flap.cl<base.cl);
 // Moment about the leading edge, positive nose up.
 assert.ok(flap.cl*flap.xcp>base.cl*base.xcp);
});
test('detached and empty panels carry no circulation',()=>{
 const bounds=cosine(4,1),lead=new Float64Array(8),chord=new Float64Array(8).fill(.1),speed=new Float64Array(8).fill(8),alpha=new Float64Array(8).fill(.1),attached=new Float64Array(8).fill(1);
 attached[3]=0;chord[5]=0;
 const g=solveVortexLattice(bounds,2,lead.map((_,p)=>-.1*(p%2)),chord,speed,alpha,attached);
 assert.equal(g[3],0);assert.equal(g[5],0);assert.ok(g[0]>0&&g.every(Number.isFinite));
});
test('chordwise bins conserve exposed area and keep loads conservative',()=>{
 const mesh=rectangle(1,.2,12);
 for(const chordPanels of [1,4]){
  const surface=buildLiftingSurface(mesh,12,.0005,chordPanels),forces=mesh.rest.map(()=>[0,0,0] as V3);
  assert.equal(new Set(surface.patches.map(p=>p.panel)).size,12*chordPanels);
  const panels=aerodynamicLoads(mesh.rest,mesh.rest.map(()=>[8,-.6,0] as V3),mesh.faces,{enabled:true,density:1.225,wind:[0,0,0],surface},forces);
  const physical=mesh.faces.reduce((s,[a,b,c])=>s+Math.hypot(...cross(sub(mesh.rest[b],mesh.rest[a]),sub(mesh.rest[c],mesh.rest[a])))/2,0);
  assert.ok(Math.abs(panels.reduce((s,p)=>s+p.area,0)-physical)<1e-12);
  assert.ok(panels.reduce((s,p)=>s+p.lift[1],0)>0);
 }
});
