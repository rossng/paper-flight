import type { Face, PaperMesh } from '../geometry/dart';
import { add, sub, scale, dot, cross, length, unit, clamp, type V3 } from './math';
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

/** Dynamic near-coplanar shielding; count material panels, not triangulation faces.
 * A small spatial hash avoids an all-pairs query. No rest-state exposure cache. */
function exposureQuery(p:V3[],faces:Face[],surface:LiftingSurface) {
  const cell=Math.max(.015,surface.span/6),gap=surface.shieldingDistance,grid=new Map<string,number[]>();
  const key=(x:number,y:number,z:number)=>`${x},${y},${z}`;
  const normals=faces.map(([a,b,c])=>unit(cross(sub(p[b],p[a]),sub(p[c],p[a]))));
  const overflow:number[]=[];
  faces.forEach((face,fi)=>{
    const low=[0,1,2].map(d=>Math.floor((Math.min(...face.map(i=>p[i][d]))-gap)/cell));
    const high=[0,1,2].map(d=>Math.floor((Math.max(...face.map(i=>p[i][d]))+gap)/cell));
    if(high.reduce((n,v,d)=>n*(v-low[d]+1),1)>512){overflow.push(fi);return;}
    for(let x=low[0];x<=high[0];x++)for(let y=low[1];y<=high[1];y++)for(let z=low[2];z<=high[2];z++){
      const k=key(x,y,z),bucket=grid.get(k);if(bucket)bucket.push(fi);else grid.set(k,[fi]);
    }
  });
  return (point:V3,fi:number)=>{
    const owners=new Set<number>(),owner=surface.owners?.[fi]??fi;owners.add(owner);
    for(const j of [...(grid.get(key(...point.map(v=>Math.floor(v/cell)) as V3))??[]),...overflow]){
      const other=surface.owners?.[j]??j;if(owners.has(other)||Math.abs(dot(normals[fi],normals[j]))<.98)continue;
      const [a,b,c]=faces[j].map(i=>p[i]),ab=sub(b,a),ac=sub(c,a),ap=sub(point,a);
      if(Math.abs(dot(ap,normals[j]))>gap)continue;
      const aa=dot(ab,ab),bb=dot(ac,ac),cc=dot(ab,ac),den=aa*bb-cc*cc;if(den<1e-20)continue;
      const u=(dot(ap,ab)*bb-dot(ap,ac)*cc)/den,v=(dot(ap,ac)*aa-dot(ap,ab)*cc)/den;
      if(u>=-1e-8&&v>=-1e-8&&u+v<=1+1e-8)owners.add(other);
    }
    return 1/owners.size;
  };
}

export function liftingSurfaceLoads(positions:V3[],velocities:V3[],faces:Face[],settings:AeroSettings,out:V3[]):PanelLoad[] {
  const surface=settings.surface!,n=surface.boundaries.length-1;
  const forward=unit(sub(positions[surface.nose],positions[surface.tail]));
  const spanVector=sub(positions[surface.right],positions[surface.left]);
  const spanAxis=unit(sub(spanVector,scale(forward,dot(spanVector,forward))));
  const coordinates=positions.map(p=>dot(sub(p,positions[surface.tail]),spanAxis));
  const spanScale=Math.max(.1,(Math.max(...coordinates)-Math.min(...coordinates))/surface.span);
  const up=unit(cross(spanVector,forward)),boundaries=surface.boundaries.map(x=>x*spanScale);
  const exposure=surface.owners?exposureQuery(positions,faces,surface):()=>1;
  const geometry=faces.map(([a,b,c],fi)=>{
    const av=cross(sub(positions[b],positions[a]),sub(positions[c],positions[a]));
    return {area:length(av)/2,normal:scale(unit(av),surface.signs[fi])};
  });
  const strips=Array.from({length:n},()=>({area:0,speed:0,alpha:0,attached:0}));
  const samples=surface.patches.map(patch=>{
    const face=faces[patch.face],g=geometry[patch.face],center:V3=[0,0,0],v:V3=[0,0,0];
    face.forEach((id,j)=>{for(let d=0;d<3;d++){center[d]+=positions[id][d]*patch.weights[j];v[d]+=velocities[id][d]*patch.weights[j];}});
    const rel=sub(v,settings.windAt?.(center)??settings.wind),speed=length(rel),direction=unit(rel);
    const alpha=Math.asin(clamp(-dot(g.normal,direction),-1,1));
    const area=g.area*patch.fraction*exposure(center,patch.face);
    // Thin-wing assumptions fade out for reversed flow, side-slip, steep panels and stall.
    const attached=(1-smooth(.21,.49,Math.abs(alpha)))*smooth(.25,.75,dot(direction,forward))*smooth(.2,.7,Math.abs(dot(g.normal,up)));
    const strip=strips[patch.station];strip.area+=area;strip.speed+=area*speed;strip.alpha+=area*attached*alpha;strip.attached+=area*attached;
    return {patch,center,area,speed,direction,alpha,normal:g.normal,attached};
  });
  const chords=strips.map((s,i)=>s.area/Math.max(boundaries[i+1]-boundaries[i],1e-8));
  const speeds=strips.map(s=>s.speed/Math.max(s.area,1e-12)),angles=strips.map(s=>s.alpha/Math.max(s.attached,1e-12));
  const attachment=strips.map(s=>s.attached/Math.max(s.area,1e-12));
  const solution=solveLiftingLine(boundaries,chords,speeds,angles,attachment);
  return samples.map(s=>{
    const i=s.patch.station,blend=s.attached;
    const clCoupled=2*solution.circulation[i]/Math.max(speeds[i]*chords[i],1e-10);
    // Span solve fixes the mean; local incidence retains chordwise reflex/trim moments.
    const clAttached=blend/Math.max(attachment[i],1e-12)*clCoupled+blend*2*Math.PI*(s.alpha-angles[i]);
    const cl=clamp(clAttached,-1.5,1.5)+(1-blend)*.85*Math.sin(2*s.alpha);
    const cd=blend*.035+(1-blend)*(.08+1.75*Math.sin(s.alpha)**2)+Math.max(0,clAttached*solution.downwash[i]/Math.max(s.speed,1e-8));
    const liftDirection=unit(sub(s.normal,scale(s.direction,dot(s.normal,s.direction))));
    const qA=settings.enabled?.5*settings.density*s.speed*s.speed*s.area:0;
    const lift=scale(liftDirection,qA*cl),force=sub(lift,scale(s.direction,qA*cd));
    // Partition of unity and linear precision preserve both force and moment exactly.
    faces[s.patch.face].forEach((id,j)=>{for(let d=0;d<3;d++)out[id][d]+=s.patch.weights[j]*force[d];});
    return {center:s.center,area:s.area,alpha:s.alpha,stalled:blend<.5,force,lift};
  });
}
