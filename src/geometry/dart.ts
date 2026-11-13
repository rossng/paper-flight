import { cross, length, sub, radians, type V3 } from '../physics/math';
export type Face = [number, number, number];
export interface Edge { a: number; b: number; rest: number; lambda: number; materialDirection?:number; dualArea?:number }
export interface Hinge { ids: [number, number, number, number]; rest: number; crease: boolean; weight: number; lambda: number; layerStiffness?:number; materialDirection?:number; plasticOffset?:number; plasticAccumulated?:number; edgeLength?:number }
export interface PaperMesh {
  rest: V3[]; faces: Face[]; edges: Edge[]; hinges: Hinge[]; masses: number[];
  materialUV?:[number,number][]; faceOwners?:number[];
  aeroWeights?:number[];layerCounts?:number[];layerTies?:import('../folding/flight-mesh').LayerTie[];sourceDesign?:string;
  regions: number[]; nose: number; tail: number; leftRoot: number; rightRoot: number;
}
/** Signed dihedral. ids are opposite vertices followed by the shared edge. */
export function hingeAngle(p0: V3, p1: V3, p2: V3, p3: V3) {
  const e = sub(p3,p2), n1 = cross(sub(p2,p0),sub(p3,p0)), n2 = cross(sub(p3,p1),sub(p2,p1));
  const s = cross(n1,n2);
  return Math.atan2((e[0]*s[0]+e[1]*s[1]+e[2]*s[2])/Math.max(length(e),1e-12),n1[0]*n2[0]+n1[1]*n2[1]+n1[2]*n2[2]);
}
/** Coarse effective folded dart, not a literal multilayer A4 folding simulation. */
/** Reflex the original effective dart was authored with: 6 mm over the rear 30% (0.09 m). */
export const LEGACY_REFLEX_DEGREES=Math.atan(.006/.09)*180/Math.PI;
/** `elevator` bends the rear 30% of each wing up by that angle (degrees), like a folded sheet's flap. */
export function createDart(dihedral = 8, rows = 8, halfColumns = 5, span = 1, size = 1, mass = 0.005, elevator = LEGACY_REFLEX_DEGREES): PaperMesh {
  const rest: V3[] = [[0.19,0,0]], regions = [0];
  const ids: number[][] = [[0]];
  for (let i=1;i<=rows;i++) {
    const t=i/rows; ids[i]=[];
    for (let j=-halfColumns;j<=halfColumns;j++) {
      const side=Math.sign(j), u=Math.abs(j);
      const z=side*t*(u===0 ? 0 : 0.010+(u-1)/(halfColumns-1)*0.080);
      const x=0.19-0.30*t+0.014*t*(u/halfColumns);
      const reflex=0.09*Math.tan(radians(elevator))*Math.max(0,(t-0.7)/0.3)*(u===0 ? 0 : 1);
      const y=u===0 ? -0.022*t : Math.tan(radians(dihedral))*(Math.abs(z)-0.01*t)+reflex;
      ids[i].push(rest.length); rest.push([x,y,z]); regions.push(j);
    }
  }
  for(const p of rest){p[0]*=size;p[1]*=size;p[2]*=span*size;}
  const faces: Face[]=[];
  for(let j=0;j<2*halfColumns;j++) faces.push([0,ids[1][j],ids[1][j+1]]);
  for(let i=1;i<rows;i++) for(let j=0;j<2*halfColumns;j++) {
    // Mirror the diagonal so the two wings have the same discretization.
    if(j<halfColumns) faces.push([ids[i][j],ids[i+1][j],ids[i+1][j+1]],[ids[i][j],ids[i+1][j+1],ids[i][j+1]]);
    else faces.push([ids[i][j],ids[i+1][j],ids[i][j+1]],[ids[i][j+1],ids[i+1][j],ids[i+1][j+1]]);
  }
  const masses=rest.map(()=>0);
  const edges: Edge[]=[], hinges: Hinge[]=[];
  const adjacency=new Map<string,{a:number;b:number;op:number;face:number}>();
  const areas=faces.map(([a,b,c])=>0.5*length(cross(sub(rest[b],rest[a]),sub(rest[c],rest[a]))));
  faces.forEach((f,fi)=> {
    const t=f.reduce((s,i)=>s+(0.19*size-rest[i][0])/(0.3*size),0)/3;
    const layers=1+4*(1-t)**2+(f.some(i=>Math.abs(regions[i])<=1)?2:0);
    for(const i of f) masses[i]+=areas[fi]*layers/3;
    for(let k=0;k<3;k++) {
      const a=f[k],b=f[(k+1)%3],op=f[(k+2)%3],key=[Math.min(a,b),Math.max(a,b)].join(':');
      const old=adjacency.get(key);
      if(old) {
        const hingeIds: Hinge['ids']=[old.op,op,old.a,old.b];
        const crease=(a===0 || b===0 || regions[a]===regions[b]) && Math.abs(regions[a]||regions[b])<=1;
        hinges.push({ids:hingeIds,rest:hingeAngle(...hingeIds.map(i=>rest[i]) as [V3,V3,V3,V3]),crease,
          weight:length(sub(rest[a],rest[b]))**2/(areas[fi]+areas[old.face]),lambda:0});
      } else {
        adjacency.set(key,{a,b,op,face:fi});
        edges.push({a,b,rest:length(sub(rest[a],rest[b])),lambda:0});
      }
    }
  });
  const total=masses.reduce((s,m)=>s+m,0);
  masses.forEach((m,i)=>masses[i]=m/total*mass);
  return {rest,faces,edges,hinges,masses,regions,nose:0,tail:ids[rows][halfColumns],leftRoot:ids[rows][halfColumns-1],rightRoot:ids[rows][halfColumns+1]};
}
