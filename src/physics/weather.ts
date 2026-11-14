import type { V3 } from './math';
import { kernel, F64, WIND_MODES } from './solver';
export type WindMode=typeof WIND_MODES[number];
export interface Weather { wind:V3; windHeading?:number; gusts:number; turbulence:number; windMode?:WindMode }
/** A weather as the solver's kernels take it: wind, heading, gusts, turbulence and the
 * number of its air feature. */
export const stagedWeather=(weather:Weather)=>[weather.wind[0],weather.wind[1],weather.wind[2],weather.windHeading??0,weather.gusts,weather.turbulence,Math.max(0,WIND_MODES.indexOf(weather.windMode??'none'))];
/** Deterministic, smooth synthetic air field: replay and flight sample the same
 * space/time function. This is prescribed wind, not a solved fluid or a wake. A steady
 * wind, a gust along it that pulses in time, a divergence-free swirl that varies
 * across panels as well as over time, and one of the fun air features (a thermal, a
 * tornado or a fan) fixed in the field. */
export function airVelocity(weather:Weather,time:number,position:V3):V3 {
 kernel.f64(F64.a,stagedWeather(weather));
 kernel.call.air_velocity(time,position[0],position[1],position[2]);
 const air=kernel.read64(F64.x);
 return [air[0],air[1],air[2]];
}
