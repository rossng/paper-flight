import type { MaterialId } from './materials';
import type { Edge, Hinge, PaperMesh } from '../geometry/dart';
import { sub, length, cross, wrapAngle } from './math';

/** Nominal engineering inputs, not measurements of the user's sheet.
 * Paper MD follows material +v (the original long edge). SI throughout. */
export interface SheetMaterial {
  density:number; thickness:number; youngMD:number; youngCD:number; shear:number; poisson:number;
  creaseWidth:number; creaseRetention:number; yieldAngle:number; hardening:number; friction:number;
}
const properties:Record<MaterialId,Omit<SheetMaterial,'thickness'>>={
  paper:{density:800,youngMD:4e9,youngCD:2e9,shear:1e9,poisson:.25,creaseWidth:.001,creaseRetention:.2,yieldAngle:.35,hardening:.08,friction:.45},
  tissue:{density:300,youngMD:.25e9,youngCD:.12e9,shear:.06e9,poisson:.2,creaseWidth:.001,creaseRetention:.12,yieldAngle:.25,hardening:.03,friction:.5},
  foil:{density:2700,youngMD:69e9,youngCD:69e9,shear:69e9/(2*1.33),poisson:.33,creaseWidth:.0005,creaseRetention:.025,yieldAngle:.08,hardening:.02,friction:.3},
  steel:{density:7850,youngMD:200e9,youngCD:200e9,shear:200e9/(2*1.3),poisson:.3,creaseWidth:.003,creaseRetention:.3,yieldAngle:.04,hardening:.1,friction:.35},
};
export function sheetMaterial(id:MaterialId,gsm:number):SheetMaterial {
  const p=properties[id];return {...p,thickness:gsm/1000/p.density};
}
/** Orthotropic engineering compliance under uniaxial loading; angle from MD. */
export function directionalYoung(m:SheetMaterial,angle:number){
  const c=Math.cos(angle),s=Math.sin(angle);
  return 1/(c**4/m.youngMD+s**4/m.youngCD+(1/m.shear-2*m.poisson/m.youngMD)*s*s*c*c);
}
export function bendingRigidity(m:SheetMaterial,angle=0){
  const nuProduct=m.poisson*m.poisson*m.youngCD/m.youngMD;
  return directionalYoung(m,angle)*m.thickness**3/(12*(1-nuProduct));
}
/** Geometric dual areas and material directions survive refinement and folding. */
export function prepareMaterialGeometry(mesh:PaperMesh){
  const uv=mesh.materialUV??mesh.rest.map(p=>[p[2],p[0]] as [number,number]);
  const areas=new Map<string,number>();
  mesh.faces.forEach(([a,b,c])=>{
    const area=length(cross(sub(mesh.rest[b],mesh.rest[a]),sub(mesh.rest[c],mesh.rest[a])))/2;
    for(const [i,j] of [[a,b],[b,c],[c,a]]){const key=[i,j].sort((a,b)=>a-b).join(':');areas.set(key,(areas.get(key)??0)+area/3);}
  });
  const angle=(a:number,b:number)=>Math.atan2(uv[b][0]-uv[a][0],uv[b][1]-uv[a][1]);
  for(const e of mesh.edges){e.materialDirection=angle(e.a,e.b);e.dualArea=areas.get([e.a,e.b].sort((a,b)=>a-b).join(':'))??e.rest**2;}
  for(const h of mesh.hinges){h.materialDirection=angle(h.ids[2],h.ids[3])+Math.PI/2;h.edgeLength=length(sub(mesh.rest[h.ids[2]],mesh.rest[h.ids[3]]));h.plasticOffset=0;h.plasticAccumulated=0;}
}
export function edgeCompliance(m:SheetMaterial,e:Edge){return e.rest**2/(directionalYoung(m,e.materialDirection??0)*m.thickness*(e.dualArea??e.rest**2));}
export function hingeStiffness(m:SheetMaterial,h:Hinge,flatScale=1,creaseScale=1){
  const d=bendingRigidity(m,h.materialDirection??0);
  // Each actual layer contributes its own D. No n^3 bonded multiplier in contact mode.
  return h.crease?d*m.creaseRetention*(h.edgeLength??.03)/m.creaseWidth*creaseScale:d*h.weight*flatScale;
}
/** Rate-independent 1D return mapping with linear isotropic hardening.
 * Store only permanent offset; kinematic recipe rest angles stay immutable. */
export function updateCreasePlasticity(h:Hinge,angle:number,m:SheetMaterial){
  if(!h.crease)return;
  const offset=h.plasticOffset??0,error=wrapAngle(angle-h.rest-offset);
  const accumulated=h.plasticAccumulated??0,yieldAngle=m.yieldAngle+m.hardening*accumulated;
  if(Math.abs(error)>yieldAngle){
    const increment=(Math.abs(error)-yieldAngle)/(1+m.hardening);
    h.plasticOffset=offset+Math.sign(error)*increment;h.plasticAccumulated=accumulated+increment;
  }
}
