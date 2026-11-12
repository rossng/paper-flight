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
  solve(p:V3[],old:V3[],w:number[],height:number,settings:GroundSettings){
    const muS=Math.max(0,settings.groundStaticFriction),muK=Math.min(muS,Math.max(0,settings.groundKineticFriction));
    for(let i=0;i<p.length;i++){
      if(w[i]<=0)continue;
      const point=p[i],gap=point[1]-height;
      if(gap>0&&this.normal[i]===0)continue;
      const normal=Math.max(0,this.normal[i]-gap/w[i]);
      point[1]+=w[i]*(normal-this.normal[i]);this.normal[i]=normal;
      if(normal>0)this.touched=true;
      const tx=this.tangentX[i]-(point[0]-old[i][0])/w[i],tz=this.tangentZ[i]-(point[2]-old[i][2])/w[i];
      const magnitude=Math.hypot(tx,tz);
      const factor=magnitude<=muS*normal?1:Math.min(1,muK*normal/Math.max(magnitude,1e-30));
      const nextX=tx*factor,nextZ=tz*factor;
      point[0]+=w[i]*(nextX-this.tangentX[i]);point[2]+=w[i]*(nextZ-this.tangentZ[i]);
      this.tangentX[i]=nextX;this.tangentZ[i]=nextZ;
    }
  }
  /** Position projection leaves a closing velocity when a vertex started above
   * the floor. Remove that residual after velocity reconstruction/internal
   * damping, with friction only for this additional normal impulse. The rest
   * of the friction budget was already spent by the position solve. There is
   * no added restitution; elastic sheet bending can still produce a rebound. */
  solveVelocity(p:V3[],v:V3[],height:number,settings:GroundSettings){
    const muS=Math.max(0,settings.groundStaticFriction),muK=Math.min(muS,Math.max(0,settings.groundKineticFriction));
    let supported=false;
    for(let i=0;i<p.length;i++){
      if(p[i][1]>height+CONTACT_TOLERANCE||this.normal[i]<=0)continue;
      supported=true;
      const closing=Math.max(0,-v[i][1]);
      if(closing===0)continue;
      v[i][1]=0;
      const speed=Math.hypot(v[i][0],v[i][2]);
      const reduction=speed<=muS*closing?speed:Math.min(speed,muK*closing);
      const factor=1-reduction/Math.max(speed,1e-30);
      v[i][0]*=factor;v[i][2]*=factor;
    }
    return supported;
  }
}
