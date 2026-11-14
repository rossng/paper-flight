import type {Parameters} from './physics/simulation';
import {findDesign} from './folding/library';
import type {WindMode} from './physics/weather';

/** Everything needed to fold and launch the same plane, plus the score to beat. */
export interface SharedFlight {
 design:string;
 parameters:Pick<Parameters,'material'|'paperWeight'|'size'|'span'|'dihedral'|'elevator'|'height'>;
 /** Launcher dial angle and trim (nose relative to throw), degrees. */
 throwAngle:number; trim:number;
 breeze:number; windHeading:number; gusts:boolean; turbulence:boolean;
 /** Fun settings; omitted when they are the everyday defaults (Earth, plain air, hand throw). */
 world?:World; windMode?:WindMode; cannon?:boolean;
 /** Distance and air time of the flight being shared, if any. */
 distance?:number; time?:number;
}
/** Weather and the challenge distance; the plane and throw are read by `readDesignSettings`/`readLaunchSettings`. */
export interface Challenge { breeze:number; windHeading:number; gusts:boolean; turbulence:boolean; world?:World; windMode?:WindMode; cannon?:boolean; distance?:number; time?:number }
export type World='earth'|'moon'|'mars';
const WORLDS:World[]=['moon','mars'],WIND_MODES:WindMode[]=['thermal','tornado','fan'];

const BREEZES=[0,1.5,3],HEADINGS=[0,90,180,270];
const round=(value:number,places:number)=>Number(value.toFixed(places));
function finite(raw:string|null,min:number,max:number){
 if(raw===null||raw.trim()==='')return undefined;
 const value=Number(raw);return Number.isFinite(value)&&value>=min&&value<=max?round(value,1):undefined;
}

/**
 * Serialise a flight using the game's own URL settings (`design`, `paperWeight`,
 * `throw`, `trim`, `mode=throw`, …) so the link opens straight in the field.
 */
export function encodeShare(flight:SharedFlight):URLSearchParams {
 const p=flight.parameters,query=new URLSearchParams({
  design:flight.design,material:p.material,size:String(p.size),span:String(round(p.span,2)),
  dihedral:String(p.dihedral),height:String(round(p.height,1)),
  throw:String(Math.round(flight.throwAngle)),trim:String(Math.round(flight.trim)),mode:'throw',
 });
 if(p.material==='paper')query.set('paperWeight',String(p.paperWeight));
 if(p.elevator)query.set('elevator',String(p.elevator));
 if(flight.breeze){query.set('breeze',String(flight.breeze));query.set('wind',String(flight.windHeading));}
 if(flight.gusts)query.set('gusts','1');
 if(flight.turbulence)query.set('swirl','1');
 if(flight.world&&flight.world!=='earth')query.set('world',flight.world);
 if(flight.windMode&&flight.windMode!=='none')query.set('air',flight.windMode);
 if(flight.cannon)query.set('cannon','1');
 if(flight.distance!==undefined)query.set('dist',String(round(flight.distance,1)));
 if(flight.time!==undefined)query.set('time',String(round(flight.time,1)));
 return query;
}

export function shareUrl(flight:SharedFlight,base:string):string {
 const url=new URL(base);url.search=encodeShare(flight).toString();url.hash='';return url.toString();
}

/** Read shared weather and the distance to beat. Unknown values fall back to still air and no challenge. */
export function readChallenge(query:URLSearchParams):Challenge {
 const breeze=BREEZES.find(value=>value===Number(query.get('breeze')))??0;
 const challenge:Challenge={breeze,windHeading:breeze?HEADINGS.find(value=>value===Number(query.get('wind')))??0:0,gusts:query.get('gusts')==='1',turbulence:query.get('swirl')==='1'};
 const world=WORLDS.find(value=>value===query.get('world')),windMode=WIND_MODES.find(value=>value===query.get('air'));
 if(world)challenge.world=world;
 if(windMode)challenge.windMode=windMode;
 if(query.get('cannon')==='1')challenge.cannon=true;
 const distance=finite(query.get('dist'),0,10000),time=finite(query.get('time'),0,1000);
 if(distance!==undefined)challenge.distance=distance;
 if(time!==undefined)challenge.time=time;
 return challenge;
}

/** Compare at the displayed precision so a matching throw reads as a tie, not a narrow win. */
export function challengeVerdict(mine:number,theirs:number):string {
 const rounded=round(mine,1),target=theirs.toFixed(1);
 return rounded>theirs?`You beat your friend’s ${target} m!`:rounded===theirs?`Dead heat with your friend’s ${target} m!`:`Your friend’s plane managed ${target} m. Keep trying!`;
}

export function shareText(flight:SharedFlight):string {
 const name=findDesign(flight.design)?.name??'paper plane';
 return flight.distance===undefined?`Try my paper plane (${name}) in Paper Flight!`:`My paper plane (${name}) flew ${flight.distance.toFixed(1)} m in Paper Flight. Can you beat it?`;
}
