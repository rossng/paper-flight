import type { PaperMesh } from '../../src/geometry/dart';
import type { V3 } from '../../src/physics/math';
export function rectangle(span=1,chord=.2,segments=12):PaperMesh {
  const rest:V3[]=[],faces:PaperMesh['faces']=[];
  for(let j=0;j<=segments;j++){const z=span*(j/segments-.5);rest.push([chord/2,0,z],[-chord/2,0,z]);if(j)faces.push([2*j-2,2*j-1,2*j],[2*j,2*j-1,2*j+1]);}
  return {rest,faces,edges:[],hinges:[],masses:rest.map(()=>.005/rest.length),regions:rest.map(()=>0),nose:2*Math.floor(segments/2),tail:2*Math.floor(segments/2)+1,leftRoot:0,rightRoot:2*segments};
}
