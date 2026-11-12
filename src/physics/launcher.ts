export const MAX_PULL=100;
export const MAX_LAUNCH_SPEED=11;
export const RELEASE_DURATION_MS=125;
const RESISTANCE=70;
/** Pointer travel is an effort proxy: each extra unit of extension needs more
 * hand travel. This controls the gesture feel independently of aircraft mass. */
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
/** Equal pull gives every material and design the same launch speed. */
export function launchSpeed(pull:number){
 return Math.min(MAX_PULL,Math.max(0,pull))/MAX_PULL*MAX_LAUNCH_SPEED;
}
/** Cosmetic quarter-cycle snap: fixed timing keeps every material responsive. */
export function springRemaining(progress:number){return Math.cos(Math.max(0,Math.min(1,progress))*Math.PI/2);}
