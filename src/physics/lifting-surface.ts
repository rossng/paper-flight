import type { Face, PaperMesh } from '../geometry/dart';
import { add, sub, scale, dot, cross, unit, clamp, type V3 } from './math';
import type { AeroSettings, PanelLoad } from './aerodynamics';

/** `panel` indexes the vortex lattice: station*chordPanels + chordwise bin (0 = leading). */
export interface SurfacePatch { face:number; weights:V3; fraction:number; station:number; panel:number }
export interface LiftingSurface {
  patches:SurfacePatch[]; boundaries:number[]; span:number; chordPanels:number;
  nose:number; tail:number; left:number; right:number; owners?:number[];
  signs:number[]; shieldingDistance:number;
}
/** Clip in material/rest span coordinates, retaining a conservative barycentric map.
 * Each span station is further clipped into `chordPanels` equal bins along the rest
 * body axis (x), from the station's own leading to trailing extent. Aerodynamic
 * resolution is independent of the structural triangulation. */
export function buildLiftingSurface(mesh:PaperMesh, stations=12, shieldingDistance=.0005, chordPanels=4):LiftingSurface {
  if(!Number.isInteger(stations)||stations<2||stations>64)throw Error('Use 2–64 aerodynamic stations');
  if(!Number.isInteger(chordPanels)||chordPanels<1||chordPanels>8)throw Error('Use 1–8 chordwise aerodynamic panels');
  const z=mesh.rest.map(p=>p[2]),lo=Math.min(...z),hi=Math.max(...z),span=hi-lo;
  const boundaries=Array.from({length:stations+1},(_,i)=>lo+span*(1-Math.cos(Math.PI*i/stations))/2);
  const patches:SurfacePatch[]=[],signs:number[]=[];
  const clip=(p:V3[],poly:V3[],axis:number,limit:number,sign:number)=>{
    const coord=(w:V3)=>w[0]*p[0][axis]+w[1]*p[1][axis]+w[2]*p[2][axis],out:V3[]=[];
    for(let i=0;i<poly.length;i++){
      const a=poly[i],b=poly[(i+1)%poly.length],da=sign*(coord(a)-limit),db=sign*(coord(b)-limit);
      if(da>=0)out.push(a);
      if((da<0&&db>0)||(da>0&&db<0))out.push(add(a,scale(sub(b,a),da/(da-db))));
    }
    return out;
  };
  // First pass: span strips, and each strip's rest chordwise extent.
  const strips:{face:number;station:number;polygon:V3[]}[]=[];
  const front=new Array(stations).fill(-Infinity),back=new Array(stations).fill(Infinity);
  mesh.faces.forEach((face,fi)=>{
    const p=face.map(i=>mesh.rest[i]),normal=cross(sub(p[1],p[0]),sub(p[2],p[0]));signs.push(normal[1]<0?-1:1);
    for(let station=0;station<stations;station++){
      const polygon=clip(p,clip(p,[[1,0,0],[0,1,0],[0,0,1]],2,boundaries[station],1),2,boundaries[station+1],-1);
      if(polygon.length<3)continue;
      strips.push({face:fi,station,polygon});
      for(const w of polygon){const x=w[0]*p[0][0]+w[1]*p[1][0]+w[2]*p[2][0];front[station]=Math.max(front[station],x);back[station]=Math.min(back[station],x);}
    }
  });
  // Second pass: chordwise bins, leading bin first.
  for(const {face:fi,station,polygon} of strips){
    const p=mesh.faces[fi].map(i=>mesh.rest[i]),chord=front[station]-back[station];
    for(let bin=0;bin<chordPanels;bin++){
      const piece=chordPanels===1?polygon:clip(p,clip(p,polygon,0,front[station]-chord*(bin+1)/chordPanels,1),0,front[station]-chord*bin/chordPanels,-1);
      for(let k=1;k+1<piece.length;k++){
        const [a,b,c]=[piece[0],piece[k],piece[k+1]],ab=sub(b,a),ac=sub(c,a);
        const fraction=Math.abs(ab[1]*ac[2]-ab[2]*ac[1]);
        if(fraction>1e-10)patches.push({face:fi,station,panel:station*chordPanels+bin,fraction,weights:scale(add(add(a,b),c),1/3)});
      }
    }
  }
  return {patches,boundaries,span,chordPanels,nose:mesh.nose,tail:mesh.tail,left:mesh.leftRoot,right:mesh.rightRoot,owners:mesh.faceOwners,signs,shieldingDistance};
}
/** Thin-sheet profile drag, attached or edge-on separated. */
export const PROFILE_DRAG=.035;
const smooth=(lo:number,hi:number,x:number)=>{const t=clamp((x-lo)/(hi-lo),0,1);return t*t*(3-2*t);};

const FOUR_PI=4*Math.PI;
/** Upward-normal (planar z) velocity at (px,py) from a unit vortex segment a->b. */
function segmentWash(px:number,py:number,ax:number,ay:number,bx:number,by:number){
  const r1x=px-ax,r1y=py-ay,r2x=px-bx,r2y=py-by,cz=r1x*r2y-r1y*r2x;
  const l1=Math.hypot(r1x,r1y),l2=Math.hypot(r2x,r2y),r0x=bx-ax,r0y=by-ay;
  // Collinear points (including a segment's own extension) receive no normal velocity.
  if(Math.abs(cz)<1e-12*(l1*l2+1e-30))return 0;
  return ((r0x*r1x+r0y*r1y)/l1-(r0x*r2x+r0y*r2y)/l2)/(FOUR_PI*cz);
}
/** Unit semi-infinite vortex from (ax,ay) downstream along -x. */
function trailingWash(px:number,py:number,ax:number,ay:number){
  const r1x=px-ax,r1y=py-ay,cz=-r1y,l1=Math.hypot(r1x,r1y);
  if(Math.abs(cz)<1e-12*(l1+1e-30))return 0;
  return (1-r1x/l1)/(FOUR_PI*cz);
}
/** Planar vortex lattice. Panel p = station*m + bin spans [boundaries[i], boundaries[i+1]]
 * and chordwise [lead-chord, lead] in body x (forward positive). A horseshoe has its bound
 * segment at the quarter chord and straight trailing legs to downstream infinity; the
 * flow-tangency point is at the three-quarter chord. Positive circulation lifts along
 * +up and induces downwash. Attachment a blends each row from tangency (a=1) to zero
 * circulation (a=0), as the lifting-line form scales its section slope. */
export function solveVortexLattice(boundaries:number[],m:number,lead:ArrayLike<number>,chord:ArrayLike<number>,speeds:ArrayLike<number>,incidence:ArrayLike<number>,attachment:ArrayLike<number>) {
  const stations=boundaries.length-1,n=stations*m,width=n+1,matrix=new Float64Array(n*width),circulation=new Float64Array(n);
  const active=(p:number)=>chord[p]>1e-9&&attachment[p]>1e-9;
  for(let p=0;p<n;p++){
    const row=p*width;
    if(!active(p)){matrix[row+p]=1;continue;}
    const i=Math.floor(p/m),px=lead[p]-.75*chord[p],py=(boundaries[i]+boundaries[i+1])/2;
    for(let q=0;q<n;q++){
      if(!active(q))continue;
      const j=Math.floor(q/m),bx=lead[q]-.25*chord[q],ya=boundaries[j],yb=boundaries[j+1];
      // Leg in from downstream to a, bound a->b, leg out from b downstream.
      matrix[row+q]=attachment[p]*(segmentWash(px,py,bx,ya,bx,yb)+trailingWash(px,py,bx,yb)-trailingWash(px,py,bx,ya));
    }
    matrix[row+p]+=(1-attachment[p])*matrix[row+p]/attachment[p];
    matrix[row+n]=attachment[p]*speeds[p]*incidence[p];
  }
  for(let k=0;k<n;k++){
    let pivot=k;for(let i=k+1;i<n;i++)if(Math.abs(matrix[i*width+k])>Math.abs(matrix[pivot*width+k]))pivot=i;
    if(pivot!==k)for(let j=k;j<=n;j++){const t=matrix[k*width+j];matrix[k*width+j]=matrix[pivot*width+j];matrix[pivot*width+j]=t;}
    const diagonal=matrix[k*width+k];if(Math.abs(diagonal)<1e-12)return new Float64Array(n);
    for(let i=k+1;i<n;i++){const f=matrix[i*width+k]/diagonal;if(f)for(let j=k;j<=n;j++)matrix[i*width+j]-=f*matrix[k*width+j];}
  }
  for(let i=n-1;i>=0;i--){let rhs=matrix[i*width+n];for(let j=i+1;j<n;j++)rhs-=matrix[i*width+j]*circulation[j];circulation[i]=rhs/matrix[i*width+i];}
  for(let i=0;i<n;i++)if(!Number.isFinite(circulation[i]))return new Float64Array(n);
  return circulation;
}
/** Downwash (positive down) at strip centres from straight trailing legs of strip-total
 * circulation: the lifting-line induced angle used for induced drag. */
export function trailingDownwash(boundaries:number[],circulation:ArrayLike<number>){
  const n=boundaries.length-1,out:number[]=[];
  for(let i=0;i<n;i++){const y=(boundaries[i]+boundaries[i+1])/2;let w=0;for(let j=0;j<n;j++)w+=circulation[j]*(1/(y-boundaries[j])-1/(y-boundaries[j+1]))/FOUR_PI;out.push(w);}
  return out;
}

/** Piecewise-constant circulation with straight, semi-infinite trailing vortex pairs.
 * Downwash is positive down. The dense solve couples every span station, including
 * the tip jumps to zero circulation. Reference planar lifting-line; flight loads use
 * the chordwise vortex lattice above. */
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
  // Per lattice panel: area, area-weighted speed, attached incidence, attached area, area-weighted body x.
  const m=surface.chordPanels??1,lattice=n*m;
  if(stripScratch.length<lattice*5)stripScratch=new Float64Array(lattice*5);
  const strips=stripScratch;strips.fill(0,0,lattice*5);
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
    const x=(center[0]-tail[0])*forward[0]+(center[1]-tail[1])*forward[1]+(center[2]-tail[2])*forward[2];
    const strip=5*(patch.panel??patch.station);strips[strip]+=area;strips[strip+1]+=area*speed;strips[strip+2]+=area*attached*alpha;strips[strip+3]+=area*attached;strips[strip+4]+=area*x;
    const o=7*k;samples[o]=dx;samples[o+1]=dy;samples[o+2]=dz;samples[o+3]=area;samples[o+4]=speed;samples[o+5]=alpha;samples[o+6]=attached;
    panels[k]={center,area,alpha,stalled:attached<.5,force:[0,0,0],lift:[0,0,0]};
  }
  // Equivalent panel chord is exposed area over strip width (as in lifting-line strip theory).
  // Bins are laid end to end, leading bin first, centred on the strip's exposed-area centroid
  // along the body axis. Contiguous panels keep every tangency point a quarter chord clear of
  // the neighbouring bound vortex, however the sheet deforms.
  const chords=new Float64Array(lattice),lead=new Float64Array(lattice),speeds=new Float64Array(lattice),angles=new Float64Array(lattice),attachment=new Float64Array(lattice);
  for(let i=0;i<n;i++){
    const width=Math.max(boundaries[i+1]-boundaries[i],1e-8);let area=0,moment=0;
    for(let p=i*m;p<(i+1)*m;p++){area+=strips[5*p];moment+=strips[5*p+4];}
    let edge=moment/Math.max(area,1e-12)+area/width/2;
    for(let p=i*m;p<(i+1)*m;p++){
      const a=strips[5*p],attached=strips[5*p+3];
      chords[p]=a/width;lead[p]=edge;edge-=chords[p];
      speeds[p]=strips[5*p+1]/Math.max(a,1e-12);angles[p]=strips[5*p+2]/Math.max(attached,1e-12);attachment[p]=attached/Math.max(a,1e-12);
    }
  }
  const circulation=solveVortexLattice(boundaries,m,lead,chords,speeds,angles,attachment);
  const stripCirculation=new Float64Array(n);for(let p=0;p<lattice;p++)stripCirculation[Math.floor(p/m)]+=circulation[p];
  const downwash=trailingDownwash(boundaries,stripCirculation);
  for(let k=0;k<patches.length;k++){
    const patch=patches[k],i=patch.station,p=patch.panel??i,o=7*k,g=4*patch.face,panel=panels[k];
    const dx=samples[o],dy=samples[o+1],dz=samples[o+2],area=samples[o+3],speed=samples[o+4],alpha=samples[o+5],blend=samples[o+6];
    const nx=geometry[g+1],ny=geometry[g+2],nz=geometry[g+3];
    // Kutta-Joukowski panel lift rho V Gamma dy, shared by exposed area: CL = 2 Gamma/(V c).
    const clCoupled=2*circulation[p]/Math.max(speeds[p]*chords[p],1e-10);
    // Lattice fixes each panel's mean; local incidence retains sub-panel variations.
    const clAttached=blend/Math.max(attachment[p],1e-12)*clCoupled+blend*2*Math.PI*(alpha-angles[p]);
    const bounded=clamp(clAttached,-1.5,1.5),cl=bounded+(1-blend)*.85*Math.sin(2*alpha);
    // Separated sheets keep the attached profile-drag floor; edge-on panels only add skin friction.
    const cd=PROFILE_DRAG+(1-blend)*1.75*Math.sin(alpha)**2+Math.max(0,bounded*downwash[i]/Math.max(speed,1e-8));
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
