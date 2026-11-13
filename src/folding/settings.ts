import {DEFAULTS,type Parameters} from '../physics/simulation';
import {materialSettings,type MaterialId} from '../physics/materials';
/** Read supported workbench and release settings from a game URL. */
export function readDesignSettings(query:URLSearchParams):Parameters {
 const number=(name:string,fallback:number,min:number,max:number)=>{const raw=query.get(name),n=raw===null?NaN:Number(raw);return Number.isFinite(n)&&n>=min&&n<=max?n:fallback;};
 const material=query.get('material');
 const id:MaterialId=material==='tissue'||material==='foil'||material==='steel'?material:'paper';
 return {...DEFAULTS,...materialSettings(id,number('paperWeight',80,60,110)),
  size:number('size',1,.85,1.15),span:number('span',1,.7,1.35),dihedral:number('dihedral',DEFAULTS.dihedral,0,16),elevator:number('elevator',DEFAULTS.elevator,-20,30),height:number('height',DEFAULTS.height,.2,100)};
}
/** Launcher aim from a game URL: throw angle (dial) and trim (nose relative to throw), degrees. */
export function readLaunchSettings(query:URLSearchParams){
 const number=(name:string,fallback:number,min:number,max:number)=>{const raw=query.get(name),n=raw===null?NaN:Number(raw);return Number.isFinite(n)&&n>=min&&n<=max?n:fallback;};
 return {throwAngle:number('throw',10,-180,180),trim:Math.round(number('trim',5,-45,45)),field:query.get('mode')==='throw'};
}
