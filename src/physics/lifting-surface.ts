import type { Face, PaperMesh } from '../geometry/dart';
import { add, sub, scale, dot, cross, unit, clamp, type V3 } from './math';
import { kernel, F64 } from './solver';

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
      // One patch per clipped polygon, at its area centroid: velocity is linear over a face
      // and the normal constant, so a fan of sub-triangles would only resample the same field.
      let fraction=0,weights:V3=[0,0,0];
      for(let k=1;k+1<piece.length;k++){
        const [a,b,c]=[piece[0],piece[k],piece[k+1]],ab=sub(b,a),ac=sub(c,a);
        const part=Math.abs(ab[1]*ac[2]-ab[2]*ac[1]);
        fraction+=part;weights=add(weights,scale(add(add(a,b),c),part/3));
      }
      if(fraction>1e-10)patches.push({face:fi,station,panel:station*chordPanels+bin,fraction,weights:scale(weights,1/fraction)});
    }
  }
  return {patches,boundaries,span,chordPanels,nose:mesh.nose,tail:mesh.tail,left:mesh.leftRoot,right:mesh.rightRoot,owners:mesh.faceOwners,signs,shieldingDistance};
}

/** Planar vortex lattice. Panel p = station*m + bin spans [boundaries[i], boundaries[i+1]]
 * and chordwise [lead-chord, lead] in body x (forward positive). A horseshoe has its bound
 * segment at the quarter chord and straight trailing legs to downstream infinity; the
 * flow-tangency point is at the three-quarter chord. Positive circulation lifts along
 * +up and induces downwash. Attachment a blends each row from tangency (a=1) to zero
 * circulation (a=0), as the lifting-line form scales its section slope. Returns the
 * circulation of each panel. */
export function solveVortexLattice(boundaries:number[],m:number,lead:ArrayLike<number>,chord:ArrayLike<number>,speeds:ArrayLike<number>,incidence:ArrayLike<number>,attachment:ArrayLike<number>) {
  kernel.f64(F64.boundaries,boundaries);kernel.f64(F64.x,lead);kernel.f64(F64.old,chord);kernel.f64(F64.w,speeds);kernel.f64(F64.a,incidence);kernel.f64(F64.b,attachment);
  kernel.call.vortex_lattice(m);
  return kernel.read64(F64.c);
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
