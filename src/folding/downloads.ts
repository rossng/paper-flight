import {geometryLicense} from './license';
import {build,exportFold,type FoldRecipe} from './model';
import {flightRecipe,supportsWingSpan} from './design';
import {findDesign,sheetDesigns,reuseBasis} from './library';
import type {Parameters} from '../physics/simulation';

export function scaledDesignRecipe(id:string,parameters:Parameters):FoldRecipe {
 const recipe:FoldRecipe=flightRecipe(id,parameters),size=parameters.size;
 recipe.width*=size;recipe.height*=size;
 // Wing operations may share axis arrays. Replace each axis, never scale aliases twice.
 recipe.operations=recipe.operations.map(operation=>({...operation,axis:operation.axis.map(point=>point.map(value=>value*size)) as typeof operation.axis}));
 return recipe;
}
/** Export the same full sheet and deployment used by designMesh, in metres. */
export function designFold(id:string,parameters:Parameters,count?:number){
 const design=findDesign(id);
 if(!design?.recipe)throw Error('The effective Original dart has no full-sheet FOLD representation.');
 const recipe=scaledDesignRecipe(id,parameters),size=parameters.size;
 const data=exportFold(build(recipe,count));
 return {...data,file_creator:'Paper Flight',file_description:`${design.note} ${reuseBasis}`,
  'paperflight:designId':id,'paperflight:source':design.recipe.source,'paperflight:reuseBasis':reuseBasis,
  'paperflight:license':geometryLicense,
  'paperflight:parameters':{size,span:supportsWingSpan(id)?parameters.span:1,dihedral:parameters.dihedral,paperWeight:parameters.paperWeight,material:parameters.material},
  'paperflight:materialArea':recipe.width*recipe.height};
}
export function foldFilename(id:string,parameters:Parameters,count?:number){
 return `paper-flight-${id}-${Math.round(parameters.size*100)}pct-${parameters.paperWeight}gsm-${parameters.dihedral}deg${supportsWingSpan(id)?`-span${Math.round(parameters.span*100)}`:''}${count===undefined?'':`-step${count}`}.fold`;
}
/** A standard multiModel FOLD, with independent sheet/folded frame pairs. */
export function collectionFold(parameters:Parameters){
 const frames=sheetDesigns.flatMap(design=>{
  const data=designFold(design.id,parameters);
  const {file_spec,file_creator,file_title,file_classes,file_description,file_frames,...frame}=data;
  return [{...frame,frame_description:file_description,frame_title:`${design.name} — original sheet`},
   {...frame,...data.file_frames[0],frame_description:file_description,frame_title:`${design.name} — folded state`}];
 });
 return {file_spec:1.2,file_creator:'Paper Flight',file_title:'Paper Flight sheet collection',file_classes:['multiModel'],
  file_description:`${sheetDesigns.length} independent full-sheet studies. Original dart excluded. ${reuseBasis}`,
  ...frames[0],file_frames:frames.slice(1)};
}
export function saveFold(data:unknown,filename:string){
 const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)+'\n'],{type:'application/json'}));
 const anchor=document.createElement('a');anchor.href=url;anchor.download=filename;document.body.append(anchor);anchor.click();anchor.remove();
 setTimeout(()=>URL.revokeObjectURL(url),1000);
}
