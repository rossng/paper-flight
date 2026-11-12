import { checkedFold, fold, sheet, type FoldRecipe, type FoldState } from './model';

/** Rotate rigid panels around the recipe's crease; never interpolate vertex positions. */
export class FoldSequence {
 readonly states:FoldState[];
 constructor(readonly recipe:FoldRecipe){
  this.states=[sheet(recipe)];
  for(const operation of recipe.operations)this.states.push(checkedFold(this.states.at(-1)!,operation));
 }
 sample(progress:number):FoldState {
  const p=Math.max(0,Math.min(this.recipe.operations.length,progress)),index=Math.floor(p);
  if(p===index)return this.states[index];
  const operation=this.recipe.operations[index];
  return fold(this.states[index],{...operation,angleDegrees:operation.angleDegrees*(p-index)});
 }
}

/** One cancellable animation per interface, with easing at each individual crease. */
export class FoldAnimator {
 private frame=0;
 get running(){return this.frame!==0;}
 cancel(){cancelAnimationFrame(this.frame);this.frame=0;}
 play(from:number,to:number,draw:(progress:number)=>void,complete:()=>void){
  this.cancel();
  if(from===to||matchMedia('(prefers-reduced-motion: reduce)').matches){draw(to);complete();return;}
  const start=performance.now(),direction=Math.sign(to-from),distance=Math.abs(to-from);
  const tick=(now:number)=>{
   const elapsed=Math.min(distance,(now-start)/700),whole=Math.floor(elapsed),fraction=elapsed-whole;
   const eased=fraction*fraction*(3-2*fraction);
   draw(from+direction*(whole+eased));
   if(elapsed>=distance){this.frame=0;complete();}else this.frame=requestAnimationFrame(tick);
  };
  draw(from);this.frame=requestAnimationFrame(tick);
 }
}
