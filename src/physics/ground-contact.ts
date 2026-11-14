import type { V3 } from './math';
import { kernel, F64 } from './solver';

/** Shared by the floor renderer and the infinite, flat collision surface. */
export const GROUND_HEIGHT=-.02;
export interface GroundSettings { groundStaticFriction:number; groundKineticFriction:number }

/** Unilateral XPBD contact against an immovable plane, as the flight solver applies it.
 * Multipliers live for one substep and accumulate across solver sweeps without resetting
 * between iterations. The sheet remains deformable: reactions act on individual vertices. */
export class GroundContactSolver {
  private handle:number;
  /** Whether any vertex has borne on the floor since the last reset. */
  touched=false;
  constructor(count:number){this.handle=kernel.call.ground_create(count);const handle=this.handle;kernel.own(this,solver=>solver.ground_free(handle));}
  reset(){kernel.call.ground_reset(this.handle);this.touched=false;}
  /** Position projection; `old` holds the positions before this substep. */
  solve(p:V3[],old:V3[],w:number[],height:number,settings:GroundSettings){
    kernel.vectors(F64.x,p);kernel.vectors(F64.old,old);kernel.f64(F64.w,w);
    this.touched=kernel.call.ground_solve(this.handle,height,settings.groundStaticFriction,settings.groundKineticFriction)===1;
    kernel.readVectors(F64.x,p);
  }
  /** Position projection leaves a closing velocity when a vertex started above
   * the floor. Remove that residual after velocity reconstruction/internal
   * damping, with friction only for this additional normal impulse. The rest
   * of the friction budget was already spent by the position solve. There is
   * no added restitution; elastic sheet bending can still produce a rebound.
   * Returns whether the floor supports any vertex. */
  solveVelocity(p:V3[],v:V3[],height:number,settings:GroundSettings){
    kernel.vectors(F64.x,p);kernel.vectors(F64.old,v);
    const supported=kernel.call.ground_solve_velocity(this.handle,height,settings.groundStaticFriction,settings.groundKineticFriction)===1;
    kernel.readVectors(F64.old,v);
    return supported;
  }
}
