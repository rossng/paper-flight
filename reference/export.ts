/** Writes the problem the reference solver integrates: the game's own mesh, material
 * model and launch, as set up by `Simulation`. Only the set-up is shared; none of the
 * game's solver kernels run in the reference. */
import { Simulation, DEFAULTS, type Parameters } from '../src/physics/simulation';
import { designMesh, flightRecipe } from '../src/folding/design';
import { build } from '../src/folding/model';
import { stackingSides } from './stack-order';
import { hingeStiffness } from '../src/physics/sheet-material';
import { materialSettings } from '../src/physics/materials';
import { radians } from '../src/physics/math';
import type { PaperMesh } from '../src/geometry/dart';

export interface Case { design:string; parameters:Parameters }
/** Full parameters for a design, as the game would launch it. */
export function caseFor(design:string,overrides:Partial<Parameters>={}):Case {
  const material=overrides.material??'paper';
  return {design,parameters:{...DEFAULTS,...materialSettings(material,overrides.paperWeight??80),...overrides,wind:[...(overrides.wind??DEFAULTS.wind)]}};
}
export function meshFor({design,parameters}:Case):PaperMesh|undefined {
  return design==='legacy'?undefined:designMesh(design,parameters);
}
export function gameSimulation(test:Case){return new Simulation(test.parameters,meshFor(test));}

export function problemText(test:Case):string {
  const sim=gameSimulation(test),p=sim.parameters,mesh=sim.mesh,material=sim.material;
  if(p.layerModel!=='contact'||p.aeroModel!=='coupled')throw Error('The reference solver models sliding contact and the coupled lifting surface only');
  if(p.substeps)throw Error('Explicit substeps are a game solver setting; the reference has its own step');
  const preset=materialSettings(p.material,p.paperWeight),flatScale=p.stiffness/preset.stiffness,creaseScale=p.crease/preset.crease;
  const yaw=radians(p.heading),elevation=radians(p.elevation);
  const scalars:Record<string,number>={
    thickness:material.thickness,youngMD:material.youngMD,youngCD:material.youngCD,shear:material.shear,
    yieldAngle:material.yieldAngle,hardening:material.hardening,layerFriction:material.friction,
    gravity:p.gravity,airDensity:p.density,aero:p.aero?1:0,
    windX:p.wind[0],windY:p.wind[1],windZ:p.wind[2],windHeading:p.windHeading,gusts:p.gusts,turbulence:p.turbulence,
    launchSpeed:p.speed,launchDuration:p.launchDuration,
    launchDirectionX:Math.cos(elevation)*Math.cos(yaw),launchDirectionY:Math.sin(elevation),launchDirectionZ:Math.cos(elevation)*Math.sin(yaw),
    groundHeight:sim.groundHeight,groundStaticFriction:p.groundStaticFriction,groundKineticFriction:p.groundKineticFriction,
    shieldingDistance:Math.max(.0002,2*material.thickness),edgeDampingRate:18,
    aeroStations:p.aeroStations,aeroChordPanels:p.aeroChordPanels,
  };
  const lines=['paperflight-reference-problem 1',`# ${test.design}`];
  const n=(value:number)=>{if(!Number.isFinite(value))throw Error('Non-finite problem value');return String(value);};
  for(const [name,value] of Object.entries(scalars))lines.push(`scalar ${name} ${n(value)}`);
  lines.push(`landmarks ${mesh.nose} ${mesh.tail} ${mesh.leftRoot} ${mesh.rightRoot}`);
  lines.push(`vertices ${mesh.rest.length}`);
  mesh.rest.forEach((rest,i)=>{
    const uv=mesh.materialUV?.[i]??[rest[2],rest[0]];
    lines.push([...rest,...sim.positions[i],mesh.masses[i],...uv].map(n).join(' '));
  });
  lines.push(`faces ${mesh.faces.length}`);
  mesh.faces.forEach((face,i)=>lines.push(`${face.join(' ')} ${mesh.faceOwners?mesh.faceOwners[i]:-1}`));
  lines.push(`hinges ${mesh.hinges.length}`);
  // Bending rigidity per hinge, with the documented friction-held laminate factor n^(2c).
  for(const h of mesh.hinges)lines.push(`${h.ids.join(' ')} ${n(h.rest)} ${n(hingeStiffness(material,h,flatScale,creaseScale)*(h.layers??1)**(2*p.stackCoupling))} ${h.crease?1:0}`);
  lines.push(`edges ${mesh.edges.length}`);
  for(const e of mesh.edges)lines.push(`${e.a} ${e.b} ${n(e.rest)}`);
  // Which side of each coincident layer a vertex was folded onto.
  if(test.design!=='legacy'){
    const sides=stackingSides(build(flightRecipe(test.design,p)),mesh,p.size);
    lines.push(`sides ${sides.length}`);
    for(const side of sides)lines.push(`${side.vertex} ${side.owner} ${side.sign}`);
  }
  lines.push('end','');
  return lines.join('\n');
}
