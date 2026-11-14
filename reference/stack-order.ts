/** Stacking order of coincident layers, from the fold sequence.
 *
 * The game's contact picks a side for two coincident layers when they first drift apart,
 * which lets a flap pass through the sheet it was folded onto. A real fold leaves the moved
 * layers on the side they travelled through. Each material point therefore carries a stack
 * coordinate per fold: a flat (180 degree) fold places every moved point on the far side of
 * every layer that stayed, by its distance from the crease; layers that moved together keep
 * their order. Comparing two coincident points fold by fold, latest first, gives their order.
 * Layers are ideal: thickness and the nesting of layers around a crease are not modelled. */
import type { FoldState, FoldOperation } from '../src/folding/model';
import type { PaperMesh } from '../src/geometry/dart';
import { add, cross, dot, length, scale, sub, unit, type V3 } from '../src/physics/math';

/** `layer` is the vertex's own panel that the comparison was made through. */
export interface Side { vertex:number; owner:number; sign:number; layer:number }

const rotate=(v:V3,axis:V3,angle:number):V3=>add(add(scale(v,Math.cos(angle)),scale(cross(axis,v),Math.sin(angle))),scale(axis,dot(axis,v)*(1-Math.cos(angle))));
const flat=(op:FoldOperation)=>Math.abs(Math.abs(op.angleDegrees)-180)<1e-9;

/** Replay the fold sequence for a material point strictly inside a panel, with the fold
 * model's own rules: a fold moves the selected layers on the positive side of its crease. */
export function foldHistory(uv:[number,number],operations:FoldOperation[]){
  let position:V3=[uv[0],uv[1],0],up:V3=[0,0,1];
  const tags:string[]=[];
  const stack=operations.map(op=>{
    const selected=(!op.layers?.includeTag||tags.includes(op.layers.includeTag))&&(!op.layers?.excludeTag||!tags.includes(op.layers.excludeTag));
    const axis=unit(sub(op.axis[1],op.axis[0])),normal=unit(op.movingNormal),angle=op.angleDegrees*Math.PI/180;
    const distance=dot(sub(position,op.axis[0]),normal);
    if(!selected||distance<=0)return 0;
    tags.push(op.id);
    position=add(op.axis[0],rotate(sub(position,op.axis[0]),axis,angle));up=rotate(up,axis,angle);
    // The moved layers come to rest on the side the fold swept through.
    return flat(op)?distance*dot(scale(cross(axis,normal),Math.sign(angle)),up):0;
  });
  return {position,up,stack};
}

function barycentric(p:V3,a:V3,b:V3,c:V3,tolerance:number){
  const ab=sub(b,a),ac=sub(c,a),ap=sub(p,a),n=cross(ab,ac),nl=length(n);
  if(nl<1e-14||Math.abs(dot(ap,n))/nl>1e-7)return;
  const aa=dot(ab,ab),bb=dot(ac,ac),cc=dot(ab,ac),den=aa*bb-cc*cc,u=(dot(ap,ab)*bb-dot(ap,ac)*cc)/den,v=(dot(ap,ac)*aa-dot(ap,ab)*cc)/den;
  return u>=-tolerance&&v>=-tolerance&&u+v<=1+tolerance?[1-u-v,u,v] as V3:undefined;
}

/** Side of each vertex relative to every other material panel it rests on, as a sign along
 * that panel's face normals. Pairs whose order the folds do not determine are left out. */
export function stackingSides(state:FoldState,mesh:PaperMesh,size=1):Side[] {
  if(!mesh.faceOwners||!mesh.materialUV)return [];
  const owners=mesh.faceOwners,uvs=mesh.materialUV,operations=state.recipe.operations,sides:Side[]=[];
  const model=(v:V3):V3=>[-v[1],v[0],v[2]];
  const incident=mesh.rest.map(()=>[] as number[]);mesh.faces.forEach((f,fi)=>f.forEach(v=>incident[v].push(fi)));
  const byOwner=new Map<number,number[]>();owners.forEach((owner,fi)=>byOwner.set(owner,[...(byOwner.get(owner)??[]),fi]));
  const mix=<T extends number[]>(face:number[],weights:V3,values:T[])=>values[face[0]].map((_,d)=>face.reduce((sum,i,j)=>sum+values[i][d]*weights[j],0));
  mesh.rest.forEach((rest,vertex)=>{
    const own=new Set(incident[vertex].map(fi=>owners[fi]));
    for(const [owner,faces] of byOwner){
      if(own.has(owner)||!faces.some(fi=>barycentric(rest,...mesh.faces[fi].map(i=>mesh.rest[i]) as [V3,V3,V3],1e-7)))continue;
      // Compare just inside one of the vertex's own faces that still lies on the panel: the
      // vertex itself usually sits on a crease, where its own panel is ambiguous.
      for(const fi of incident[vertex]){
        const face=mesh.faces[fi],weights:V3=[0,0,0];face.forEach((i,j)=>weights[j]=(i===vertex?1-2e-3:1e-3));
        const point=mix(face,weights,mesh.rest) as V3;
        let target:{face:number[];weights:V3}|undefined;
        for(const other of faces){const w=barycentric(point,...mesh.faces[other].map(i=>mesh.rest[i]) as [V3,V3,V3],-1e-9);if(w){target={face:mesh.faces[other],weights:w};break;}}
        if(!target)continue;
        const a=foldHistory(scale2(mix(face,weights,uvs),1/size),operations);
        const b=foldHistory(scale2(mix(target.face,target.weights,uvs),1/size),operations);
        if(length(sub(scale(model(a.position),size),point))>1e-6||length(sub(a.position,b.position))>1e-6)throw Error('Fold history does not reproduce the flight mesh');
        const alignment=dot(a.up,b.up);if(Math.abs(alignment)<.99)continue;
        let order=0;
        for(let k=operations.length-1;k>=0&&!order;k--){const difference=a.stack[k]*Math.sign(alignment)-b.stack[k];if(Math.abs(difference)>1e-9)order=Math.sign(difference);}
        if(!order)continue;
        const [p0,p1,p2]=target.face.map(i=>mesh.rest[i]),normal=cross(sub(p1,p0),sub(p2,p0));
        sides.push({vertex,owner,sign:order*Math.sign(dot(model(b.up),normal)),layer:owners[fi]});
        break;
      }
    }
  });
  return sides;
}
const scale2=(v:number[],s:number):[number,number]=>[v[0]*s,v[1]*s];
