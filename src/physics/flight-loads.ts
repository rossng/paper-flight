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
