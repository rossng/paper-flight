import type { Face, PaperMesh } from '../geometry/dart';
import { add, sub, scale, dot, cross, unit, clamp, type V3 } from './math';
import type { AeroSettings, PanelLoad } from './aerodynamics';

export interface SurfacePatch { face:number; weights:V3; fraction:number; station:number }
export interface LiftingSurface {
  patches:SurfacePatch[]; boundaries:number[]; span:number;
  nose:number; tail:number; left:number; right:number; owners?:number[];
  signs:number[]; shieldingDistance:number;
}
/** Clip in material/rest span coordinates, retaining a conservative barycentric map.
 * Aerodynamic resolution is independent of the structural triangulation. */
export function buildLiftingSurface(mesh:PaperMesh, stations=12, shieldingDistance=.0005):LiftingSurface {
  if(!Number.isInteger(stations)||stations<2||stations>64)throw Error('Use 2–64 aerodynamic stations');
  const z=mesh.rest.map(p=>p[2]),lo=Math.min(...z),hi=Math.max(...z),span=hi-lo;
  const boundaries=Array.from({length:stations+1},(_,i)=>lo+span*(1-Math.cos(Math.PI*i/stations))/2);
  const patches:SurfacePatch[]=[],signs:number[]=[];
  mesh.faces.forEach((face,fi)=>{
    const p=face.map(i=>mesh.rest[i]),normal=cross(sub(p[1],p[0]),sub(p[2],p[0]));signs.push(normal[1]<0?-1:1);
    const coord=(w:V3)=>w.reduce((s,v,i)=>s+v*p[i][2],0);
    const clip=(poly:V3[],limit:number,sign:number)=>{
      const out:V3[]=[];
      for(let i=0;i<poly.length;i++){
        const a=poly[i],b=poly[(i+1)%poly.length],da=sign*(coord(a)-limit),db=sign*(coord(b)-limit);
        if(da>=0)out.push(a);
        if((da<0&&db>0)||(da>0&&db<0))out.push(add(a,scale(sub(b,a),da/(da-db))));
      }
      return out;
    };
    for(let station=0;station<stations;station++){
      const polygon=clip(clip([[1,0,0],[0,1,0],[0,0,1]],boundaries[station],1),boundaries[station+1],-1);
      for(let k=1;k+1<polygon.length;k++){
        const [a,b,c]=[polygon[0],polygon[k],polygon[k+1]],ab=sub(b,a),ac=sub(c,a);
        const fraction=Math.abs(ab[1]*ac[2]-ab[2]*ac[1]);
        if(fraction>1e-10)patches.push({face:fi,station,fraction,weights:scale(add(add(a,b),c),1/3)});
      }
    }
  });
  return {patches,boundaries,span,nose:mesh.nose,tail:mesh.tail,left:mesh.leftRoot,right:mesh.rightRoot,owners:mesh.faceOwners,signs,shieldingDistance};
}
const smooth=(lo:number,hi:number,x:number)=>{const t=clamp((x-lo)/(hi-lo),0,1);return t*t*(3-2*t);};

/** Piecewise-constant circulation with straight, semi-infinite trailing vortex pairs.
 * Downwash is positive down. The dense solve couples every span station, including
 * the tip jumps to zero circulation. This is planar lifting-line, NOT AVL/VLM. */
export function solveLiftingLine(boundaries:number[],chords:number[],speeds:number[],incidence:number[],attachment=chords.map(()=>1)) {
  const n=chords.length,centers=chords.map((_,i)=>(boundaries[i]+boundaries[i+1])/2);
  const influence=centers.map(y=>chords.map((_,j)=>(1/(y-boundaries[j])-1/(y-boundaries[j+1]))/(4*Math.PI)));
  const matrix=chords.map((c,i)=>{
    const k=Math.PI*c*attachment[i];
    return [...influence[i].map((d,j)=>(i===j?1:0)+k*d),k*speeds[i]*incidence[i]];
  });
  for(let k=0;k<n;k++){
    let pivot=k;for(let i=k+1;i<n;i++)if(Math.abs(matrix[i][k])>Math.abs(matrix[pivot][k]))pivot=i;
    [matrix[k],matrix[pivot]]=[matrix[pivot],matrix[k]];
    if(Math.abs(matrix[k][k])<1e-12)return {circulation:chords.map(()=>0),downwash:chords.map(()=>0)};
    for(let i=k+1;i<n;i++){const f=matrix[i][k]/matrix[k][k];for(let j=k;j<=n;j++)matrix[i][j]-=f*matrix[k][j];}
  }
  const circulation=chords.map(()=>0);
  for(let i=n-1;i>=0;i--){let rhs=matrix[i][n];for(let j=i+1;j<n;j++)rhs-=matrix[i][j]*circulation[j];circulation[i]=rhs/matrix[i][i];}
  return {circulation,downwash:influence.map(row=>row.reduce((sum,d,j)=>sum+d*circulation[j],0))};
}

// Reused spatial-hash storage: bucket heads, and chained (face, next) entries.
const HASH_SIZE=4096;
const hashHead=new Int32Array(HASH_SIZE);
let hashFace=new Int32Array(4096),hashNext=new Int32Array(4096),faceGeometry=new Float64Array(0);
const cellHash=(x:number,y:number,z:number)=>(Math.imul(x,73856093)^Math.imul(y,19349663)^Math.imul(z,83492791))&(HASH_SIZE-1);
/** Dynamic near-coplanar shielding; count material panels, not triangulation faces.
 * A small spatial hash avoids an all-pairs query. No rest-state exposure cache.
 * Hash collisions only add candidates to the exact geometric test below. */
function exposureQuery(p:V3[],faces:Face[],surface:LiftingSurface) {
  const cell=Math.max(.008,surface.span/16),gap=surface.shieldingDistance,faceOwners=surface.owners;
  // Deformed face geometry is invariant across this evaluation's patch queries,
  // not across substeps: peeling/sliding and replay always rebuild this cache.
  // Per face: a, ab, ac, unit normal, |ab|^2, |ac|^2, ab.ac, Gram determinant.
  if(faceGeometry.length<faces.length*16)faceGeometry=new Float64Array(faces.length*16);
  const g=faceGeometry;
  hashHead.fill(-1);let entries=0;
  const overflow:number[]=[];
  for(let fi=0;fi<faces.length;fi++){
    const face=faces[fi],a=p[face[0]],b=p[face[1]],c=p[face[2]],o=16*fi;
    const abx=b[0]-a[0],aby=b[1]-a[1],abz=b[2]-a[2],acx=c[0]-a[0],acy=c[1]-a[1],acz=c[2]-a[2];
    const nx=aby*acz-abz*acy,ny=abz*acx-abx*acz,nz=abx*acy-aby*acx,inverse=1/Math.max(Math.hypot(nx,ny,nz),1e-12);
    const aa=abx*abx+aby*aby+abz*abz,bb=acx*acx+acy*acy+acz*acz,cc=abx*acx+aby*acy+abz*acz;
    g[o]=a[0];g[o+1]=a[1];g[o+2]=a[2];g[o+3]=abx;g[o+4]=aby;g[o+5]=abz;g[o+6]=acx;g[o+7]=acy;g[o+8]=acz;
    g[o+9]=nx*inverse;g[o+10]=ny*inverse;g[o+11]=nz*inverse;g[o+12]=aa;g[o+13]=bb;g[o+14]=cc;g[o+15]=aa*bb-cc*cc;
    const lx=Math.floor((Math.min(a[0],b[0],c[0])-gap)/cell),hx=Math.floor((Math.max(a[0],b[0],c[0])+gap)/cell);
    const ly=Math.floor((Math.min(a[1],b[1],c[1])-gap)/cell),hy=Math.floor((Math.max(a[1],b[1],c[1])+gap)/cell);
    const lz=Math.floor((Math.min(a[2],b[2],c[2])-gap)/cell),hz=Math.floor((Math.max(a[2],b[2],c[2])+gap)/cell);
    const cells=(hx-lx+1)*(hy-ly+1)*(hz-lz+1);
    if(!(cells<=512)){overflow.push(fi);continue;}
    if(entries+cells>hashFace.length){
      const size=Math.max(hashFace.length*2,entries+cells),nextFace=new Int32Array(size),nextLink=new Int32Array(size);
      nextFace.set(hashFace);nextLink.set(hashNext);hashFace=nextFace;hashNext=nextLink;
    }
    for(let x=lx;x<=hx;x++)for(let y=ly;y<=hy;y++)for(let z=lz;z<=hz;z++){
      const h=cellHash(x,y,z);hashFace[entries]=fi;hashNext[entries]=hashHead[h];hashHead[h]=entries++;
    }
  }
  const owners:number[]=[];
  let px0=0,py0=0,pz0=0,ox=0,oy=0,oz=0;
  const shield=(j:number)=>{
    const other=faceOwners?.[j]??j,o=16*j;
    if(owners.includes(other)||Math.abs(ox*g[o+9]+oy*g[o+10]+oz*g[o+11])<.98)return;
    const px=px0-g[o],py=py0-g[o+1],pz=pz0-g[o+2],den=g[o+15];
    if(Math.abs(px*g[o+9]+py*g[o+10]+pz*g[o+11])>gap||den<1e-20)return;
    const dab=px*g[o+3]+py*g[o+4]+pz*g[o+5],dac=px*g[o+6]+py*g[o+7]+pz*g[o+8];
    const u=(dab*g[o+13]-dac*g[o+14])/den,v=(dac*g[o+12]-dab*g[o+14])/den;
    if(u>=-1e-8&&v>=-1e-8&&u+v<=1+1e-8)owners.push(other);
  };
  return (point:V3,fi:number)=>{
    owners.length=0;owners.push(faceOwners?.[fi]??fi);
    const own=16*fi;ox=g[own+9];oy=g[own+10];oz=g[own+11];px0=point[0];py0=point[1];pz0=point[2];
    for(let e=hashHead[cellHash(Math.floor(px0/cell),Math.floor(py0/cell),Math.floor(pz0/cell))];e>=0;e=hashNext[e])shield(hashFace[e]);
    for(let k=0;k<overflow.length;k++)shield(overflow[k]);
    return 1/owners.length;
  };
}

let patchScratch=new Float64Array(0),stripScratch=new Float64Array(0),normalScratch=new Float64Array(0);
export function liftingSurfaceLoads(positions:V3[],velocities:V3[],faces:Face[],settings:AeroSettings,out:V3[]):PanelLoad[] {
  const surface=settings.surface!,n=surface.boundaries.length-1,patches=surface.patches;
  const forward=unit(sub(positions[surface.nose],positions[surface.tail]));
  const spanVector=sub(positions[surface.right],positions[surface.left]);
  const spanAxis=unit(sub(spanVector,scale(forward,dot(spanVector,forward))));
  const tail=positions[surface.tail];
  let low=Infinity,high=-Infinity;
  for(const p of positions){const c=(p[0]-tail[0])*spanAxis[0]+(p[1]-tail[1])*spanAxis[1]+(p[2]-tail[2])*spanAxis[2];if(c<low)low=c;if(c>high)high=c;}
  const spanScale=Math.max(.1,(high-low)/surface.span);
  const up=unit(cross(spanVector,forward)),boundaries=surface.boundaries.map(x=>x*spanScale);
  // Layer shielding changes slowly; a caller that samples loads several
  // times per frame may hold one frame's exposure in `settings.exposure`.
  const held=settings.exposure,reuse=!!held&&held.valid&&held.values.length===patches.length;
  if(held&&!reuse){if(held.values.length!==patches.length)held.values=new Float64Array(patches.length);held.valid=true;}
  const exposure=surface.owners&&!reuse?exposureQuery(positions,faces,surface):undefined;
  // Per face: area and oriented unit normal.
  if(normalScratch.length<faces.length*4)normalScratch=new Float64Array(faces.length*4);
  const geometry=normalScratch;
  for(let fi=0;fi<faces.length;fi++){
    const face=faces[fi],a=positions[face[0]],b=positions[face[1]],c=positions[face[2]];
    const ux=b[0]-a[0],uy=b[1]-a[1],uz=b[2]-a[2],vx=c[0]-a[0],vy=c[1]-a[1],vz=c[2]-a[2];
    const nx=uy*vz-uz*vy,ny=uz*vx-ux*vz,nz=ux*vy-uy*vx,len=Math.hypot(nx,ny,nz),inverse=surface.signs[fi]/Math.max(len,1e-12);
    geometry[4*fi]=len/2;geometry[4*fi+1]=nx*inverse;geometry[4*fi+2]=ny*inverse;geometry[4*fi+3]=nz*inverse;
  }
  // Per strip: area, area-weighted speed, attached incidence, attached area.
  if(stripScratch.length<n*4)stripScratch=new Float64Array(n*4);
  const strips=stripScratch;strips.fill(0,0,n*4);
  // Per patch: direction, area, speed, incidence, attachment.
  if(patchScratch.length<patches.length*7)patchScratch=new Float64Array(patches.length*7);
  const samples=patchScratch,panels:PanelLoad[]=new Array(patches.length);
  for(let k=0;k<patches.length;k++){
    const patch=patches[k],face=faces[patch.face],weights=patch.weights,g=4*patch.face,w0=weights[0],w1=weights[1],w2=weights[2];
    const pa=positions[face[0]],pb=positions[face[1]],pc=positions[face[2]],va=velocities[face[0]],vb=velocities[face[1]],vc=velocities[face[2]];
    const center:V3=[pa[0]*w0+pb[0]*w1+pc[0]*w2,pa[1]*w0+pb[1]*w1+pc[1]*w2,pa[2]*w0+pb[2]*w1+pc[2]*w2];
    const wind=settings.windAt?.(center)??settings.wind;
    const rx=va[0]*w0+vb[0]*w1+vc[0]*w2-wind[0],ry=va[1]*w0+vb[1]*w1+vc[1]*w2-wind[1],rz=va[2]*w0+vb[2]*w1+vc[2]*w2-wind[2];
    const speed=Math.hypot(rx,ry,rz),inverse=1/Math.max(speed,1e-12),dx=rx*inverse,dy=ry*inverse,dz=rz*inverse;
    const nx=geometry[g+1],ny=geometry[g+2],nz=geometry[g+3];
    const alpha=Math.asin(clamp(-(nx*dx+ny*dy+nz*dz),-1,1));
    const exposed=reuse?held!.values[k]:exposure?exposure(center,patch.face):1;
    if(held&&!reuse)held.values[k]=exposed;
    const area=geometry[g]*patch.fraction*exposed;
    // Thin-wing assumptions fade out for reversed flow, side-slip, steep panels and stall.
    const attached=(1-smooth(.21,.49,Math.abs(alpha)))*smooth(.25,.75,dx*forward[0]+dy*forward[1]+dz*forward[2])*smooth(.2,.7,Math.abs(nx*up[0]+ny*up[1]+nz*up[2]));
    const strip=4*patch.station;strips[strip]+=area;strips[strip+1]+=area*speed;strips[strip+2]+=area*attached*alpha;strips[strip+3]+=area*attached;
    const o=7*k;samples[o]=dx;samples[o+1]=dy;samples[o+2]=dz;samples[o+3]=area;samples[o+4]=speed;samples[o+5]=alpha;samples[o+6]=attached;
    panels[k]={center,area,alpha,stalled:attached<.5,force:[0,0,0],lift:[0,0,0]};
  }
  const chords:number[]=[],speeds:number[]=[],angles:number[]=[],attachment:number[]=[];
  for(let i=0;i<n;i++){
    const area=strips[4*i],attached=strips[4*i+3];
    chords.push(area/Math.max(boundaries[i+1]-boundaries[i],1e-8));speeds.push(strips[4*i+1]/Math.max(area,1e-12));
    angles.push(strips[4*i+2]/Math.max(attached,1e-12));attachment.push(attached/Math.max(area,1e-12));
  }
  const solution=solveLiftingLine(boundaries,chords,speeds,angles,attachment);
  for(let k=0;k<patches.length;k++){
    const patch=patches[k],i=patch.station,o=7*k,g=4*patch.face,panel=panels[k];
    const dx=samples[o],dy=samples[o+1],dz=samples[o+2],area=samples[o+3],speed=samples[o+4],alpha=samples[o+5],blend=samples[o+6];
    const nx=geometry[g+1],ny=geometry[g+2],nz=geometry[g+3];
    const clCoupled=2*solution.circulation[i]/Math.max(speeds[i]*chords[i],1e-10);
    // Span solve fixes the mean; local incidence retains chordwise reflex/trim moments.
    const clAttached=blend/Math.max(attachment[i],1e-12)*clCoupled+blend*2*Math.PI*(alpha-angles[i]);
    const cl=clamp(clAttached,-1.5,1.5)+(1-blend)*.85*Math.sin(2*alpha);
    const cd=blend*.035+(1-blend)*(.08+1.75*Math.sin(alpha)**2)+Math.max(0,clAttached*solution.downwash[i]/Math.max(speed,1e-8));
    const along=nx*dx+ny*dy+nz*dz,lx=nx-dx*along,ly=ny-dy*along,lz=nz-dz*along,liftScale=1/Math.max(Math.hypot(lx,ly,lz),1e-12);
    const qA=settings.enabled?.5*settings.density*speed*speed*area:0,lifting=qA*cl*liftScale,drag=qA*cd;
    const lift=panel.lift,force=panel.force;
    lift[0]=lx*lifting;lift[1]=ly*lifting;lift[2]=lz*lifting;
    force[0]=lift[0]-dx*drag;force[1]=lift[1]-dy*drag;force[2]=lift[2]-dz*drag;
    // Partition of unity and linear precision preserve both force and moment exactly.
    const face=faces[patch.face],weights=patch.weights;
    for(let j=0;j<3;j++){const target=out[face[j]],weight=weights[j];target[0]+=weight*force[0];target[1]+=weight*force[1];target[2]+=weight*force[2];}
  }
  return panels;
}
