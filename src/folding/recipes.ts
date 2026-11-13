import type {FoldOperation,FoldRecipe} from './model';
import type {V3} from '../physics/math';
const width=0.2159,height=0.2794,half=width/2;
const op=(id:string,label:string,axis:[V3,V3],movingNormal:V3,angleDegrees=180,layers?:FoldOperation['layers']):FoldOperation=>({id,label,axis,movingNormal,angleDegrees,layers});
const centre=op('centre-guide','Mark the centre crease',[[0,0,0],[0,height,0]],[-1,0,0],0);
const corners=[op('left-corner','Bring in the left corner',[[0,0,0],[-half,half,0]],[-1,-1,0]),op('right-corner','Bring in the right corner',[[0,0,0],[half,half,0]],[1,-1,0])];
const body=(noseY:number,tapered=true,root=width/4):FoldOperation[]=>{
 const slope=tapered?root/(height-noseY):0,axis:[V3,V3]=[[tapered?0:root,noseY,0],[root,height,0]];
 return [op('left-stack','Close the centre fold',[[0,0,0],[0,height,0]],[-1,0,0]),op('left-wing','Open the first wing',axis,[1,-slope,0],90,{includeTag:'left-stack'}),op('right-wing','Open the opposite wing',axis,[1,-slope,0],-90,{excludeTag:'left-stack'})];
};
const t=Math.tan(Math.PI/8);
const base={width,height,gsm:80};
export const recipes:FoldRecipe[]=[
 {...base,id:'basic',name:'The Basic — geometry study',source:'https://www.foldnfly.com/0.html',operations:[centre,...corners,...body(0,false)]},
 {...base,id:'dart',name:'Basic Dart — geometry study',source:'https://www.foldnfly.com/1.html',operations:[centre,...corners,op('left-shoulder','Narrow the left shoulder',[[0,0,0],[-t*height,height,0]],[-1,-t,0]),op('right-shoulder','Narrow the right shoulder',[[0,0,0],[t*height,height,0]],[1,-t,0]),...body(0)]},
 {...base,id:'stable',name:'The Stable — locking-flap study',source:'https://www.foldnfly.com/2.html',operations:[centre,...corners,op('nose-down','Fold the layered nose down',[[-half,half,0],[half,half,0]],[0,-1,0]),...([-1,1] as const).map(side=>{
  const source:V3=[side*half,half,0],target:V3=[0,width-0.0254,0],mid:V3=[source[0]/2,(source[1]+target[1])/2,0],nx=source[0],ny=source[1]-target[1];
  return op(side<0?'left-shoulder':'right-shoulder','Bring the shoulder over the nose',[mid,[mid[0]-ny,mid[1]+nx,0]],[nx,ny,0]);
 }),op('nose-lock','Turn up only the nose flap',[[-half,width-0.0254,0],[half,width-0.0254,0]],[0,1,0],180,{includeTag:'nose-down'}),...body(half,false,(width-0.125)/2)]}
];

// Independently constructed coordinates and labels. These are geometric studies of
// the linked folding ideas, not downloaded Fold 'N Fly artwork or templates.
const across=(id:string,label:string,y:number,side=-1,angle=180,layers?:FoldOperation['layers'])=>
 op(id,label,[[-width,y,0],[width,y,0]],[0,side,0],angle,layers);
const shoulders=(y:number,slope:number,prefix='shoulder'):FoldOperation[]=>([-1,1] as const).map(side=>
 op(`${side<0?'left':'right'}-${prefix}`,'Turn the sloping edge inward',[[0,y,0],[side*slope*height,y+height,0]],[side,-slope,0]));
const deployed=(h:number,nose:number,root:number,tapered=false):FoldOperation[]=>{
 const axis:[V3,V3]=[[tapered?0:root,nose,0],[root,h,0]],slope=tapered?root/(h-nose):0;
 return [op('left-stack','Close the centre fold',[[0,0,0],[0,h,0]],[-1,0,0]),
 op('left-wing','Open the first wing',axis,[1,-slope,0],90,{includeTag:'left-stack'}),
 op('right-wing','Open the opposite wing',axis,[1,-slope,0],-90,{excludeTag:'left-stack'})];
};
const shiftedCorners=(w:number,y:number)=>shoulders(y,1,'corner').map(o=>({...o,axis:[o.axis[0],[Math.sign(o.axis[1][0])*w/2,y+w/2,0]] as [V3,V3]}));
const noseReturn=(height/2-.055+height)/2;
recipes.push(
 {...base,id:'heavy-nose',name:'Heavy Nose — pleated dart study',source:'https://www.foldnfly.com/7.html',operations:[centre,...corners,...shoulders(0,t),
  across('nose-down','Reverse the long nose',height/2),across('nose-return','Return the nose over the front edge',noseReturn,1,180,{includeTag:'nose-down'}),
  across('nose-pleat','Fold the tip into the nose pleat',(height/2-.055+noseReturn)/2,-1,180,{includeTag:'nose-return'}),...deployed(height,height/2,.022)]},
 {...base,id:'hunting',name:'Hunting — rolled glider study',source:'https://www.foldnfly.com/6.html',operations:[centre,
  ...[1,2,3].map(i=>across(`nose-roll-${i}`,'Roll the leading edge over',i*.0254)),...shiftedCorners(width,.0762),
  ...([-1,1] as const).map(side=>op(`${side<0?'left':'right'}-tip`,'Raise the outer wingtip',[[side*(half-.012),0,0],[side*(half-.012),height,0]],[side,0,0],side*90)),
  ...deployed(height,.0762,.015)]},
 {...base,id:'buzz',name:'The Buzz — folded nose study',source:'https://www.foldnfly.com/3.html',operations:[centre,...corners,
  across('nose-down','Bring the point to the shoulder line',half/2),
  // Each shoulder flap crosses the centreline, so the second overlaps the first instead of refolding its tip.
  ...shoulders(half/2,t).map(o=>o.id==='right-shoulder'?{...o,layers:{excludeTag:'left-shoulder'}}:o),
  across('nose-hem','Turn the nose edge underneath',half/2+.013,-1,-180),...deployed(height,half/2+.013,.013)]},
 {...base,height:width,id:'v-wing',name:'V-Wing — square sheet study',source:'https://www.foldnfly.com/39.html',operations:[
  {...centre,axis:[[0,0,0],[0,width,0]]},...corners,across('nose-down','Bring the point to the shoulder line',half/2),
  ...shiftedCorners(width,half/2),...deployed(width,half/2,.018,true)]},
 {...base,width:height,height:width,id:'underside',name:'Underside — broad wing study',source:'https://www.foldnfly.com/27.html',operations:[
  {...centre,axis:[[0,0,0],[0,width,0]]},across('edge-roll-1','Double the leading band',width/4),across('edge-roll-2','Roll the band to the middle',width/2),
  ...([.027,.054] as const).flatMap((reach,i)=>([-1,1] as const).map(side=>op(`${side<0?'left':'right'}-band-corner-${i}`,'Turn the band corner inward',[[side*(height/2-reach),width/2,0],[side*height/2,width/2+reach,0]],[side,-1,0]))),
  across('nose-down','Fold the layered tip inward',width/2+.02),...deployed(width,width/2+.02,.0254)]},
 {...base,id:'raven',name:'Raven — layered lock study',source:'https://www.foldnfly.com/50.html',operations:[centre,...corners,...shoulders(0,t),
  across('nose-down','Reverse the layered dart nose',height/2),...shiftedCorners(width,height/2),
  across('nose-lock','Return only the centre nose flap',height*.75,1,180,{includeTag:'nose-down'}),...deployed(height,height/2,.028)]}
);
