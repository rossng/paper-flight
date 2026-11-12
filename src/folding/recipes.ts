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
