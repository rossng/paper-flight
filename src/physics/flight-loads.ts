import type { PanelLoad } from './aerodynamics';
import { length, type V3 } from './math';

/** Sum signed panel vectors before taking magnitude: opposing loads cancel.
 * Vertical lift excludes drag and is measured in world-up coordinates. */
export function liftSummary(panels:PanelLoad[],mass:number,gravity:number){
 const net:V3=[0,0,0];
 for(const panel of panels)for(let d=0;d<3;d++)net[d]+=panel.lift[d];
 const weight=mass*Math.abs(gravity);
 return {net,magnitude:length(net),upward:net[1],weight,supportRatio:weight>1e-12?net[1]/weight:null};
}
export type LiftSummary=ReturnType<typeof liftSummary>;

/** Area-weighted existing model classification, not a measured stall boundary.
 * Coupled patches classify stall, reversed flow and strong side-slip together.
 * Without aerodynamic loading, a percentage would be misleading. */
export function stallSummary(panels:PanelLoad[]){
 let area=0,separated=0,incidence=0,load=0;
 for(const p of panels){
  if(p.area<=0)continue;
  area+=p.area;if(p.stalled)separated+=p.area;
  incidence+=Math.abs(p.alpha)*p.area;load+=length(p.force);
 }
 const available=area>1e-12&&load>1e-10;
 return {fraction:available?separated/area:null,meanIncidence:available?incidence/area*180/Math.PI:null};
}
export type StallSummary=ReturnType<typeof stallSummary>;
