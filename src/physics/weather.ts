import { length, type V3 } from './math';
export interface Weather { wind:V3; windHeading?:number; gusts:number; turbulence:number }
/** Deterministic, smooth synthetic air field: replay and flight sample the same
 * space/time function. This is prescribed wind, not a solved fluid or a wake. */
export function airVelocity(weather:Weather,time:number,position:V3):V3 {
 const speed=length(weather.wind),direction:V3=speed>1e-9?weather.wind.map(v=>v/speed) as V3:[Math.cos((weather.windHeading??0)*Math.PI/180),0,Math.sin((weather.windHeading??0)*Math.PI/180)];
 const gust=weather.gusts*(.5+.5*Math.sin(time*2.1-.8))**3;
 const [x,y,z]=position,t=weather.turbulence;
 // Each component is independent of its own coordinate: the imposed swirl
 // field has zero divergence, and varies across panels as well as over time.
 return [
  weather.wind[0]+direction[0]*gust+t*.45*(Math.sin(1.7*z+2.9*time)+.5*Math.sin(4.3*y-5.1*time)),
  weather.wind[1]+direction[1]*gust+t*.35*(Math.sin(2.3*x-3.1*time+.7)+.5*Math.sin(5.4*z+6.7*time)),
  weather.wind[2]+direction[2]*gust+t*.55*(Math.sin(2.1*y+2.5*time+1.2)+.5*Math.sin(4.1*x-4.7*time)),
 ];
}
