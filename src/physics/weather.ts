import { length, type V3 } from './math';
export type WindMode='none'|'thermal'|'tornado'|'fan';
export interface Weather { wind:V3; windHeading?:number; gusts:number; turbulence:number; windMode?:WindMode }
/** Fun prescribed air features, positioned in field metres from the launcher. */
function feature(mode:WindMode|undefined,[x,y,z]:V3):V3 {
 if(mode==='thermal')return [0,3.5*Math.exp(-((x-5)**2+z*z)/(2*1.5**2)),0];
 if(mode==='tornado'){const dx=x-6,dz=z-1,r=Math.hypot(dx,dz)+1e-6,v=16*r/(1+r*r);return [-dz/r*v,2.5*Math.exp(-r*r/4),dx/r*v];}
 if(mode==='fan'){const d=Math.hypot(x,z+2);return [0,0,6*Math.exp(-((y-1.6)**2)/.5)/(1+.3*d)];}
 return [0,0,0];
}
/** Deterministic, smooth synthetic air field: replay and flight sample the same
 * space/time function. This is prescribed wind, not a solved fluid or a wake. */
export function airVelocity(weather:Weather,time:number,position:V3):V3 {
 const speed=length(weather.wind),direction:V3=speed>1e-9?weather.wind.map(v=>v/speed) as V3:[Math.cos((weather.windHeading??0)*Math.PI/180),0,Math.sin((weather.windHeading??0)*Math.PI/180)];
 const gust=weather.gusts*(.5+.5*Math.sin(time*2.1-.8))**3;
 const [x,y,z]=position,t=weather.turbulence,f=feature(weather.windMode,position);
 // Each component is independent of its own coordinate: the imposed swirl
 // field has zero divergence, and varies across panels as well as over time.
 return [
  f[0]+weather.wind[0]+direction[0]*gust+t*.45*(Math.sin(1.7*z+2.9*time)+.5*Math.sin(4.3*y-5.1*time)),
  f[1]+weather.wind[1]+direction[1]*gust+t*.35*(Math.sin(2.3*x-3.1*time+.7)+.5*Math.sin(5.4*z+6.7*time)),
  f[2]+weather.wind[2]+direction[2]*gust+t*.55*(Math.sin(2.1*y+2.5*time+1.2)+.5*Math.sin(4.1*x-4.7*time)),
 ];
}
