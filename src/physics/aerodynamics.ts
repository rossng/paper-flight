import { clamp, cross, dot, length, scale, sub, unit, type V3 } from './math';
import type { Face } from '../geometry/dart';
export interface PanelLoad { center: V3; force: V3; lift: V3; alpha: number; area: number; stalled: boolean }
export interface AeroSettings { density: number; wind: V3; enabled: boolean; areaWeights?:number[]; windAt?:(position:V3)=>V3 }
/** Quasi-steady low-aspect-ratio polar with a smooth, unvalidated separated-flow blend. */
export function coefficients(alpha: number) {
  const a=Math.abs(alpha), start=15*Math.PI/180, end=30*Math.PI/180;
  const t=clamp((a-start)/(end-start),0,1), blend=t*t*(3-2*t);
  const attached=2.7*alpha;
  const cl=(1-blend)*attached+blend*0.85*Math.sin(2*alpha);
  const cd=(1-blend)*(0.035+0.18*cl*cl)+blend*(0.08+1.75*Math.sin(alpha)**2);
  return {cl,cd,stalled:a>start};
}
/** Reads CURRENT vertices, areas, normals and velocities every substep. No rest normals. */
export function aerodynamicLoads(positions: V3[], velocities: V3[], faces: Face[], settings: AeroSettings, out: V3[]): PanelLoad[] {
  const panels: PanelLoad[]=[];
  for(const [fi,[a,b,c]] of faces.entries()) {
    const pa=positions[a],pb=positions[b],pc=positions[c];
    const center: V3=[(pa[0]+pb[0]+pc[0])/3,(pa[1]+pb[1]+pc[1])/3,(pa[2]+pb[2]+pc[2])/3];
    const av=cross(sub(pb,pa),sub(pc,pa)), twiceArea=length(av), area=twiceArea/2*(settings.areaWeights?.[fi]??1);
    const v: V3=[0,0,0],wind=settings.windAt?.(center)??settings.wind;
    for(let d=0;d<3;d++) v[d]=(velocities[a][d]+velocities[b][d]+velocities[c][d])/3-wind[d];
    const speed=length(v), force: V3=[0,0,0], liftForce: V3=[0,0,0];
    let alpha=0,stalled=false;
    if(settings.enabled && twiceArea>1e-12 && speed>1e-8) {
      const n=scale(av,1/twiceArea), direction=scale(v,1/speed), s=clamp(dot(n,direction),-1,1);
      alpha=Math.asin(-s);
      const polar=coefficients(alpha); stalled=polar.stalled;
      // Lift perpendicular to velocity; drag always removes energy in still air.
      const lift=unit(sub(n,scale(direction,s))), qA=0.5*settings.density*speed*speed*area;
      for(let d=0;d<3;d++) {liftForce[d]=qA*polar.cl*lift[d];force[d]=qA*(polar.cl*lift[d]-polar.cd*direction[d]);}
      for(const i of [a,b,c]) for(let d=0;d<3;d++) out[i][d]+=force[d]/3;
    }
    panels.push({center,force,lift:liftForce,alpha,area,stalled});
  }
  return panels;
}

/** Signed lift centroid projected along net lift. This is an estimate for
 * nonparallel panel loads, not a unique 3D centre of pressure or aerodynamic centre. */
export function centerOfLift(panels:PanelLoad[]):V3|undefined {
  const net:V3=[0,0,0];let gross=0;
  for(const panel of panels){gross+=length(panel.lift);for(let d=0;d<3;d++)net[d]+=panel.lift[d];}
  const magnitude=length(net);
  // Avoid an unstable location when opposing loads nearly cancel.
  if(magnitude<Math.max(1e-8,gross*0.001))return;
  const direction=scale(net,1/magnitude),center:V3=[0,0,0];
  for(const panel of panels){const weight=dot(panel.lift,direction)/magnitude;for(let d=0;d<3;d++)center[d]+=panel.center[d]*weight;}
  return center;
}
