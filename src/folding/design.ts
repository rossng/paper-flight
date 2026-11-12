import {recipes} from './recipes';
import {build,type FoldRecipe} from './model';
import {compileFlightMesh} from './flight-mesh';
import type {Parameters} from '../physics/simulation';
/** The tapered Folded Dart recipe currently supports only its authored width. */
export function supportsWingSpan(id:string){return ['legacy','basic','stable'].includes(id);}
export function flightRecipe(id:string,parameters:Parameters):FoldRecipe {
 const original=recipes.find(r=>r.id===id);if(!original)throw Error('Unknown fold design');const recipe=structuredClone(original);recipe.gsm=parameters.paperWeight;
 for(const op of recipe.operations)if(op.id==='left-wing'||op.id==='right-wing'){
  const delta=(1-(supportsWingSpan(id)?parameters.span:1))*recipe.width/4;
  if(op.axis[0][0]>1e-8)op.axis[0][0]+=delta;op.axis[1][0]+=delta;
  const slope=(op.axis[1][0]-op.axis[0][0])/(op.axis[1][1]-op.axis[0][1]);op.movingNormal=[1,-slope,0];op.angleDegrees=(op.id==='left-wing'?1:-1)*(90-parameters.dihedral);
 }
 return recipe;
}
export function designMesh(id:string,parameters:Parameters){return compileFlightMesh(build(flightRecipe(id,parameters)),{size:parameters.size});}
