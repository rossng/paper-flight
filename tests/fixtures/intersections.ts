import type { Face } from '../../src/geometry/dart';
import { cross, dot, length, scale, sub, type V3 } from '../../src/physics/math';

/** Independent endpoint diagnostic: proper transverse triangle intersections.
 * Ignores shared vertices, coplanar overlap and touches within 0.1 micrometre.
 * Depth is the smaller endpoint distance of a crossing edge to the other plane,
 * not a global minimum translation distance or a swept collision certificate. */
export function intersections(p:V3[],faces:Face[]) {
  let pairs=0,depth=0;
  const tolerance=1e-7;
  const bounds=faces.map(f=>[0,1,2].map(d=>[Math.min(...f.map(i=>p[i][d])),Math.max(...f.map(i=>p[i][d]))]));
  const crossing=(a:Face,b:Face)=>{
    const origin=p[b[0]],u=sub(p[b[1]],origin),v=sub(p[b[2]],origin),raw=cross(u,v),norm=length(raw);
    if(norm<1e-12)return 0;
    const n=scale(raw,1/norm),uu=dot(u,u),vv=dot(v,v),uv=dot(u,v),den=uu*vv-uv*uv;
    let result=0;
    for(let k=0;k<3;k++){
      const x=p[a[k]],y=p[a[(k+1)%3]],dx=dot(sub(x,origin),n),dy=dot(sub(y,origin),n);
      if(dx*dy>=0||Math.min(Math.abs(dx),Math.abs(dy))<=tolerance)continue;
      const t=dx/(dx-dy),q=sub(x.map((c,d)=>c+t*(y[d]-c)) as V3,origin),qu=dot(q,u),qv=dot(q,v);
      const s=(qu*vv-qv*uv)/den,r=(qv*uu-qu*uv)/den;
      if(s>1e-7&&r>1e-7&&s+r<1-1e-7)result=Math.max(result,Math.min(Math.abs(dx),Math.abs(dy)));
    }
    return result;
  };
  for(let a=0;a<faces.length;a++)for(let b=a+1;b<faces.length;b++){
    if(faces[a].some(i=>faces[b].includes(i))||bounds[a].some(([lo,hi],d)=>lo>bounds[b][d][1]||hi<bounds[b][d][0]))continue;
    const d=Math.max(crossing(faces[a],faces[b]),crossing(faces[b],faces[a]));
    if(d>0){pairs++;depth=Math.max(depth,d);}
  }
  return {pairs,depth};
}
