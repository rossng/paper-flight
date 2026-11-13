import type { V3 } from './math';

/** Shared by the floor renderer and the infinite, flat collision surface. */
export const GROUND_HEIGHT=-.02;
const CONTACT_TOLERANCE=1e-7;
export interface GroundSettings { groundStaticFriction:number; groundKineticFriction:number }

/** Unilateral XPBD contact against an immovable plane. Multipliers live for
 * one substep and accumulate across solver sweeps without resetting between iterations.
 * The sheet remains deformable: reactions act on individual material vertices. */
export class GroundContactSolver {
  private normal:Float64Array;
  private tangentX:Float64Array;
  private tangentZ:Float64Array;
  touched=false;
  constructor(count:number){
    this.normal=new Float64Array(count);this.tangentX=new Float64Array(count);this.tangentZ=new Float64Array(count);
  }
  reset(){this.normal.fill(0);this.tangentX.fill(0);this.tangentZ.fill(0);this.touched=false;}
  /** Flat xyz coordinates; `old` holds the positions before this substep. */
  solveFlat(x:Float64Array,old:Float64Array,w:ArrayLike<number>,height:number,settings:GroundSettings){
    const muS=Math.max(0,settings.groundStaticFriction),muK=Math.min(muS,Math.max(0,settings.groundKineticFriction));
    const normals=this.normal,tangentX=this.tangentX,tangentZ=this.tangentZ;
    for(let i=0,k=0;i<normals.length;i++,k+=3){
      const gap=x[k+1]-height,previous=normals[i];
      if(gap>0&&previous===0)continue;
      const weight=w[i];if(weight<=0)continue;
      const normal=Math.max(0,previous-gap/weight);
      x[k+1]+=weight*(normal-previous);normals[i]=normal;
      if(normal>0)this.touched=true;
      const tx=tangentX[i]-(x[k]-old[k])/weight,tz=tangentZ[i]-(x[k+2]-old[k+2])/weight;
      const magnitude=Math.hypot(tx,tz);
      const factor=magnitude<=muS*normal?1:Math.min(1,muK*normal/Math.max(magnitude,1e-30));
      const nextX=tx*factor,nextZ=tz*factor;
      x[k]+=weight*(nextX-tangentX[i]);x[k+2]+=weight*(nextZ-tangentZ[i]);
      tangentX[i]=nextX;tangentZ[i]=nextZ;
    }
  }
  /** Position projection leaves a closing velocity when a vertex started above
   * the floor. Remove that residual after velocity reconstruction/internal
   * damping, with friction only for this additional normal impulse. The rest
   * of the friction budget was already spent by the position solve. There is
   * no added restitution; elastic sheet bending can still produce a rebound. */
  solveVelocityFlat(x:Float64Array,v:Float64Array,height:number,settings:GroundSettings){
    const muS=Math.max(0,settings.groundStaticFriction),muK=Math.min(muS,Math.max(0,settings.groundKineticFriction));
    let supported=false;
    for(let i=0,k=0;i<this.normal.length;i++,k+=3){
      if(x[k+1]>height+CONTACT_TOLERANCE||this.normal[i]<=0)continue;
      supported=true;
      const closing=Math.max(0,-v[k+1]);
      if(closing===0)continue;
      v[k+1]=0;
      const speed=Math.hypot(v[k],v[k+2]);
      const reduction=speed<=muS*closing?speed:Math.min(speed,muK*closing);
      const factor=1-reduction/Math.max(speed,1e-30);
      v[k]*=factor;v[k+2]*=factor;
    }
    return supported;
  }
  // Per-vertex array forms of the same kernels.
  solve(p:V3[],old:V3[],w:number[],height:number,settings:GroundSettings){
    const x=pack(p);this.solveFlat(x,pack(old),w,height,settings);unpack(x,p);
  }
  solveVelocity(p:V3[],v:V3[],height:number,settings:GroundSettings){
    const velocity=pack(v),supported=this.solveVelocityFlat(pack(p),velocity,height,settings);unpack(velocity,v);return supported;
  }
}
const pack=(vectors:V3[])=>{const out=new Float64Array(vectors.length*3);for(let i=0;i<vectors.length;i++){out[3*i]=vectors[i][0];out[3*i+1]=vectors[i][1];out[3*i+2]=vectors[i][2];}return out;};
const unpack=(flat:Float64Array,vectors:V3[])=>{for(let i=0;i<vectors.length;i++){vectors[i][0]=flat[3*i];vectors[i][1]=flat[3*i+1];vectors[i][2]=flat[3*i+2];}};
