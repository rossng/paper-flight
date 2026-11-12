export const MAX_PULL=100;
export const SPRING_STIFFNESS=.8;
const RESISTANCE=70;
/** Pointer travel is an effort proxy: each extra unit of extension needs more
 * hand travel. The actual launch spring remains Hookean (energy = k x² / 2). */
export function elasticPull(x:number,y:number){
 y=Math.max(0,y);const travel=Math.hypot(x,y);
 if(!travel)return {x:0,y:0};
 const extension=Math.min(MAX_PULL,Math.sqrt(RESISTANCE**2+2*RESISTANCE*travel)-RESISTANCE);
 return {x:x*extension/travel,y:y*extension/travel};
}
/** Inverse mapping keeps a new grab or keyboard adjustment continuous. */
export function pullTravel(x:number,y:number){
 const extension=Math.hypot(x,y),factor=1+extension/(2*RESISTANCE);
 return {x:x*factor,y:y*factor};
}
export function springSpeed(pull:number,mass:number){
 const extension=Math.min(MAX_PULL,Math.max(0,pull))/MAX_PULL*.7;
 return Math.min(9,Math.sqrt(SPRING_STIFFNESS*extension**2/mass));
}
/** Release is the first quarter-cycle of an ideal spring, accelerating to launch. */
export function springReleaseDuration(mass:number){return Math.PI/2*Math.sqrt(mass/SPRING_STIFFNESS)*1000;}
export function springRemaining(progress:number){return Math.cos(Math.max(0,Math.min(1,progress))*Math.PI/2);}
