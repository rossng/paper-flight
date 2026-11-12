import {DEFAULTS,type Parameters} from '../physics/simulation';
import {materialSettings,type MaterialId} from '../physics/materials';
/** Only workbench settings travel between the game and study. */
export function readDesignSettings(query:URLSearchParams):Parameters {
 const number=(name:string,fallback:number,min:number,max:number)=>{const raw=query.get(name),n=raw===null?NaN:Number(raw);return Number.isFinite(n)&&n>=min&&n<=max?n:fallback;};
 const material=query.get('material');
 const id:MaterialId=material==='tissue'||material==='foil'||material==='steel'?material:'paper';
 return {...DEFAULTS,...materialSettings(id,number('paperWeight',80,60,110)),
  size:number('size',1,.85,1.15),span:number('span',1,.7,1.35),dihedral:number('dihedral',DEFAULTS.dihedral,0,16)};
}
export function designQuery(id:string,p:Parameters){
 return new URLSearchParams({design:id,size:String(p.size),span:String(p.span),dihedral:String(p.dihedral),paperWeight:String(p.paperWeight),material:p.material}).toString();
}
