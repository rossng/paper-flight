import { createDart, type PaperMesh } from '../geometry/dart';
import type { Parameters } from '../physics/simulation';
import { flightRecipe, supportsElevator, supportsWingSpan } from './design';
import { build } from './model';
import { cross, dot, sub, unit } from '../physics/math';

type PreviewMesh = Pick<PaperMesh,'rest'|'faces'|'hinges'> & {edges:Pick<PaperMesh['edges'][number],'a'|'b'>[]};

/** Small geometry-derived previews, drawn locally with the current paper settings. */
export function planeThumbnail(mesh:PreviewMesh){
 const points=mesh.rest.map(([x,y,z])=>[x*.65+z*.8,-y*.7-x*.35+z*.3]);
 const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]),minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys);
 const scale=Math.min(70/(maxX-minX),40/(maxY-minY)),project=(i:number)=>`${40+(points[i][0]-(minX+maxX)/2)*scale},${25+(points[i][1]-(minY+maxY)/2)*scale}`;
 const faces=mesh.faces.map(face=>({face,depth:face.reduce((sum,v)=>sum+mesh.rest[v][1],0)})).sort((a,b)=>a.depth-b.depth);
 const counts=new Map<string,number>(),key=(a:number,b:number)=>[Math.min(a,b),Math.max(a,b)].join(':');
 for(const {face} of faces)for(let i=0;i<3;i++){const k=key(face[i],face[(i+1)%3]);counts.set(k,(counts.get(k)??0)+1);}
 const creases=new Set(mesh.hinges.filter(h=>h.crease).map(h=>key(h.ids[2],h.ids[3])));
 const surfaces=faces.map(({face})=>{const [a,b,c]=face.map(i=>mesh.rest[i]),normal=unit(cross(sub(b,a),sub(c,a))),light=Math.abs(dot(normal,unit([.2,1,.4]))),color=`hsl(45 35% ${80+16*light}%)`;return `<polygon points="${face.map(project).join(' ')}" fill="${color}" stroke="${color}" stroke-width=".3"/>`;}).join('');
 const outlines=mesh.edges.filter(e=>counts.get(key(e.a,e.b))===1||creases.has(key(e.a,e.b))).map(e=>`<path d="M${project(e.a)} L${project(e.b)}"/>`).join('');
 return `<svg class="plane-preview" viewBox="0 0 80 50" aria-hidden="true"><ellipse cx="40" cy="41" rx="27" ry="5" fill="#243b4914"/>${surfaces}<g fill="none" stroke="#6d8170" stroke-width=".5" stroke-linejoin="round">${outlines}</g></svg>`;
}

// Keep only the latest shape per design. Size and weight do not change a normalized
// silhouette. Menu previews need no aerodynamic refinement or contact metadata.
const previews=new Map<string,{key:string;svg:string}>();
export function designThumbnail(id:string,parameters:Parameters){
 const span=supportsWingSpan(id)?parameters.span:1,elevator=supportsElevator(id)?parameters.elevator:0,key=`${parameters.dihedral}:${span}:${elevator}`;
 const cached=previews.get(id);if(cached?.key===key)return cached.svg;
 let mesh:PreviewMesh;
 if(id==='legacy')mesh=createDart(parameters.dihedral,8,5,span,1,.005,elevator);
 else{
  const state=build(flightRecipe(id,parameters));
  mesh={rest:[],faces:[],edges:[],hinges:[]};
  for(const panel of state.panels){
   const start=mesh.rest.length;
   mesh.rest.push(...panel.points.map(({position:[x,y,z]})=>[-y,x,z] as [number,number,number]));
   for(let j=1;j<panel.points.length-1;j++)mesh.faces.push([start,start+j,start+j+1]);
   for(let j=0;j<panel.points.length;j++)mesh.edges.push({a:start+j,b:start+(j+1)%panel.points.length});
  }
 }
 const svg=planeThumbnail(mesh);previews.set(id,{key,svg});return svg;
}
