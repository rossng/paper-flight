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
/** Equal pull gives every material and design the same nominal launch speed. */
export function launchSpeed(pull:number){
 return Math.min(MAX_PULL,Math.max(0,pull))/MAX_PULL*MAX_LAUNCH_SPEED;
}
/** Ideal spring relaxation: x/x0 = cos(omega*t), omega = pi/(2*T). */
export function springRemaining(progress:number){return Math.cos(Math.max(0,Math.min(1,progress))*Math.PI/2);}
/** Velocity contributed by an ideal elastic launcher, in the absence of other
 * loads. Its acceleration starts finite and decreases to zero at detachment.
 * The effective spring stiffness scales with mass (k/m = omega²), preserving
 * the existing equal-pull speed control. This prescribes the unloaded spring
 * profile; aerodynamic drag and gravity still change the actual exit velocity. */
export function springSpeed(time:number,speed:number,duration=RELEASE_DURATION_MS/1000){
 if(time<=0)return 0;
 if(duration<=0||time>=duration)return speed;
 return speed*Math.sin(time/duration*Math.PI/2);
}
/** Exact impulse per unit mass across a substep, including detachment. A zero
 * duration is the explicit legacy initial-velocity mode, handled by launch(). */
export function springVelocityDelta(time:number,dt:number,speed:number,duration=RELEASE_DURATION_MS/1000){
 if(duration<=0||dt<=0||time>=duration)return 0;
 return springSpeed(time+dt,speed,duration)-springSpeed(time,speed,duration);
}
