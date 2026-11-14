import {sheet,checkedFold,type FoldOperation,type FoldRecipe,type FoldState,type UV} from './model';
import {add,sub,scale,dot,unit,type V3} from '../physics/math';

/** Experimental "fun" designs. Folds after wing deployment are authored on
 * material coordinates, then resolved to concrete 3D creases by replaying the
 * recipe, so every operation here is an ordinary checked FoldOperation. */
type Layers=FoldOperation['layers'];
const op=(id:string,label:string,axis:[V3,V3],movingNormal:V3,angleDegrees=180,layers?:Layers):FoldOperation=>({id,label,axis,movingNormal,angleDegrees,layers});

function inside(uv:UV,poly:UV[]){let sign=0;for(let i=0;i<poly.length;i++){const a=poly[i],b=poly[(i+1)%poly.length],c=(b[0]-a[0])*(uv[1]-a[1])-(b[1]-a[1])*(uv[0]-a[0]);if(Math.abs(c)<1e-12)continue;const s=Math.sign(c);if(sign&&s!==sign)return false;sign=s;}return true;}
/** Current position of a material point; panels are convex and rigid. */
function at(state:FoldState,uv:UV):V3{
 for(const panel of state.panels){
  if(!inside(uv,panel.points.map(p=>p.uv)))continue;
  const [p0,p1]=panel.points;let k=2;
  while(k<panel.points.length&&Math.abs((p1.uv[0]-p0.uv[0])*(panel.points[k].uv[1]-p0.uv[1])-(p1.uv[1]-p0.uv[1])*(panel.points[k].uv[0]-p0.uv[0]))<1e-14)k++;
  const q=panel.points[k],e1=[p1.uv[0]-p0.uv[0],p1.uv[1]-p0.uv[1]],e2=[q.uv[0]-p0.uv[0],q.uv[1]-p0.uv[1]],d=[uv[0]-p0.uv[0],uv[1]-p0.uv[1]],den=e1[0]*e2[1]-e1[1]*e2[0];
  const s=(d[0]*e2[1]-d[1]*e2[0])/den,t=(e1[0]*d[1]-e1[1]*d[0])/den;
  return add(p0.position,add(scale(sub(p1.position,p0.position),s),scale(sub(q.position,p0.position),t)));
 }
 throw Error(`Material point ${uv} is not on the sheet`);
}

class Builder {
 state:FoldState;operations:FoldOperation[]=[];
 constructor(readonly recipe:FoldRecipe){this.state=sheet(recipe);}
 fold(operation:FoldOperation){this.state=checkedFold(this.state,operation);this.operations.push(operation);return this;}
 /** Crease a→b in material space; m lies on the moving side. `want` picks the
  * rotation sign that moves m up (flight up is +x here) or down, or along ±z. */
 material(id:string,label:string,a:UV,b:UV,m:UV,angle:number,layers?:Layers,want?:'up'|'down'|'+z'){
  const A=at(this.state,a),B=at(this.state,b),M=at(this.state,m),d=unit(sub(B,A)),v=sub(M,A),n=sub(v,scale(d,dot(v,d)));
  const make=(g:number)=>op(id,label,[A,B],n,g,layers);
  if(!want)return this.fold(make(angle));
  const score=(g:number)=>{const p=at(checkedFold(this.state,make(g)),m);return want==='up'?p[0]:want==='down'?-p[0]:p[2];};
  return this.fold(make(score(angle)>=score(-angle)?angle:-angle));
 }
 done(extra:Partial<FoldRecipe>={}):FoldRecipe{return {...this.recipe,...extra,operations:this.operations};}
}

const W=.2159,H=.2794;
interface BasicOptions{w?:number;h?:number;rootL?:number;rootR?:number;dihedralL?:number;dihedralR?:number;noseFolds?:number[]}
/** The Basic, with independently authored left and right wings. */
function basic(id:string,name:string,o:BasicOptions={}){
 const w=o.w??W,h=o.h??H,half=w/2,rootL=o.rootL??w/4,rootR=o.rootR??w/4,dl=o.dihedralL??8,dr=o.dihedralR??8;
 const b=new Builder({id,name,source:'https://www.foldnfly.com/0.html',width:w,height:h,gsm:80,operations:[],fixedWings:true});
 b.fold(op('centre-guide','Mark the centre crease',[[0,0,0],[0,h,0]],[-1,0,0],0));
 b.fold(op('left-corner','Bring in the left corner',[[0,0,0],[-half,half,0]],[-1,-1,0])).fold(op('right-corner','Bring in the right corner',[[0,0,0],[half,half,0]],[1,-1,0]));
 (o.noseFolds??[]).forEach((c,i)=>b.fold(op(`nose-${i}`,i?'Roll the nose again':'Roll the nose under',[[-half,c,0],[half,c,0]],[0,-1,0])));
 b.fold(op('left-stack','Close the centre fold',[[0,0,0],[0,h,0]],[-1,0,0]));
 b.fold(op('left-wing','Open the first wing',[[rootL,0,0],[rootL,h,0]],[1,0,0],90-dl,{includeTag:'left-stack'}));
 b.fold(op('right-wing','Open the opposite wing',[[rootR,0,0],[rootR,h,0]],[1,0,0],-(90-dr),{excludeTag:'left-stack'}));
 return b;
}
const sides=[{sign:1,tag:'right-wing',name:'right'},{sign:-1,tag:'left-wing',name:'left'}];
function winglets(b:Builder,dir:'up'|'down',inset=.025){
 const half=b.recipe.width/2,h=b.recipe.height;
 for(const {sign,tag,name} of sides){const x=sign*(half-inset);b.material(`winglet-${name}`,`Turn the ${name} wingtip ${dir}`,[x,Math.abs(x)+.001],[x,h],[sign*(half-.003),h-.01],90,{includeTag:tag},dir);}
 return b;
}
function concertina(b:Builder,n=5,angle=14){
 const half=b.recipe.width/2,h=b.recipe.height,root=b.recipe.width/4;
 for(const {sign,tag,name} of sides){let previous=tag;for(let k=1;k<=n;k++){const x=sign*(root+k*(half-root)/(n+1)),id=`pleat-${name}-${k}`;
  b.material(id,`Pleat the ${name} wing (${k}/${n})`,[x,Math.abs(x)+.001],[x,h],[sign*(half-.002),h-.01],angle,{includeTag:previous},k%2?'up':'down');previous=id;}}
 return b;
}
function ring(id:string,name:string,noseWraps:number,chord=.07,faces=6){
 const half=W/2,b=new Builder({id,name,source:'',width:W,height:chord,gsm:80,operations:[],fixedWings:true});
 for(let i=0;i<noseWraps;i++)b.fold(op(`wrap-${i}`,'Wrap the leading edge',[[-half,.012*(i+1),0],[half,.012*(i+1),0]],[0,-1,0]));
 // Parallel creases share one rotation direction: a constant signed angle curls
 // the strip consistently into a closed polygon instead of a zigzag.
 let previous:string|undefined;
 for(let k=1;k<faces;k++){const x=-half+k*W/faces,fid=`ring-${k}`;b.material(fid,`Bend the ring (${k}/${faces-1})`,[x,chord],[x,chord*.999],[x+.005,chord*.9],360/faces,previous?{includeTag:previous}:undefined);previous=fid;}
 return b.done();
}

export interface FunDesign {recipe:FoldRecipe;detail:string}
export const funDesigns:FunDesign[]=[
 {recipe:winglets(basic('fun-winglets','Winglets'),'up').done(),detail:'Tips up · the only fix for the roll'},
 {recipe:basic('fun-hammerhead','Hammerhead',{noseFolds:[.03,.045,.06,.075]}).done(),detail:'Heavy rolled nose'},
 {recipe:basic('fun-wonky-span','Wonky (short wing)',{rootR:W/4+.02}).done(),detail:'Right wing 2 cm short · spirals'},
 {recipe:basic('fun-wonky-angle','Wonky (odd angles)',{dihedralL:20,dihedralR:-10}).done(),detail:'One up, one down'},
 {recipe:basic('fun-bat','The Bat',{dihedralL:-20,dihedralR:-20}).done(),detail:'Droopy −20° wings'},
 {recipe:winglets(basic('fun-winglets-down','Droop tips'),'down').done(),detail:'Tips down'},
 {recipe:concertina(basic('fun-concertina','Concertina')).done(),detail:'Five pleats per wing'},
 {recipe:basic('fun-receipt','Receipt',{w:.08,h:.3}).done(),detail:'80 × 300 mm till roll'},
 {recipe:basic('fun-banknote','Banknote',{w:.066,h:.156}).done(),detail:'66 × 156 mm'},
 {recipe:basic('fun-sticky','Sticky Note',{w:.076,h:.076}).done(),detail:'76 mm square'},
 {recipe:basic('fun-backwards','Backwards Basic').done({pose:'backwards'}),detail:'Thrown tail-first'},
 {recipe:{id:'fun-flat',name:'Just Throw It',source:'',width:W,height:H,gsm:80,operations:[],fixedWings:true,pose:'flat'},detail:'No folds at all'},
 {recipe:ring('fun-ring','Ring glider',0),detail:'Open hexagonal tube'},
 {recipe:ring('fun-ring-nose','Ring + nose wraps',2),detail:'Weighted leading edge'},
];
