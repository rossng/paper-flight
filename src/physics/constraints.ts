import { type Edge, type Hinge } from '../geometry/dart';
import { kernel, F64, I32 } from './solver';
/** Structure-of-arrays constraint data for the solver, which reads flat xyz coordinates
 * (three per vertex). `alpha` is compliance/dt² and `lambda` the multiplier per hinge;
 * `offset` is each hinge's permanent (plastic) offset from its rest angle. */
export interface HingeBatch { ids:Int32Array; rest:Float64Array; offset:Float64Array; alpha:Float64Array; lambda:Float64Array }
export interface EdgeBatch { ids:Int32Array; rest:Float64Array }
export type Weights=ArrayLike<number>;
export function hingeBatch(hinges:Hinge[]):HingeBatch {
  return {ids:Int32Array.from(hinges.flatMap(h=>h.ids)),rest:Float64Array.from(hinges.map(h=>h.rest)),
    offset:new Float64Array(hinges.length),alpha:new Float64Array(hinges.length),lambda:new Float64Array(hinges.length)};
}
/** Set hinge j's permanent (plastic) offset from its rest angle. */
export function setHingeOffset(batch:HingeBatch,j:number,offset:number){batch.offset[j]=offset;}
export function edgeBatch(edges:Edge[]):EdgeBatch {
  return {ids:Int32Array.from(edges.flatMap(e=>[e.a,e.b])),rest:Float64Array.from(edges.map(e=>e.rest))};
}
/** Stage a hinge batch for a solver kernel. */
export function stageHinges(batch:HingeBatch){
  kernel.i32(I32.hingeIds,batch.ids);kernel.f64(F64.hingeRest,batch.rest);kernel.f64(F64.c,batch.alpha);kernel.f64(F64.d,batch.lambda);kernel.f64(F64.e,batch.offset);
}
/** One XPBD sweep over every hinge on flat xyz coordinates, with signed dihedral
 * gradients (adapted from MIT-licensed PositionBasedDynamics; see THIRD_PARTY.md).
 * The angle error is wrapped, so it is continuous across the ±π branch cut. */
export function solveHinges(x:Float64Array,w:Weights,batch:HingeBatch,forward:boolean) {
  stageHinges(batch);kernel.f64(F64.x,x);kernel.f64(F64.w,w);
  kernel.call.hinges_solve(forward?1:0);
  x.set(kernel.read64(F64.x));batch.lambda.set(kernel.read64(F64.d));
}
