//! Contact with the ground: an infinite, flat, immovable plane.
use crate::fast::{R, W};
use crate::js;

/// Height of the infinite, flat collision surface. The floor renderer draws the ground
/// at the same height, from `GROUND_HEIGHT` in src/physics/ground-contact.ts.
pub const GROUND_HEIGHT: f64 = -0.02;
/// A vertex within this height (metres) of the plane counts as resting on it.
const CONTACT_TOLERANCE: f64 = 1e-7;

/// Unilateral XPBD contact against an immovable plane. Multipliers live for
/// one substep and accumulate across solver sweeps without resetting between iterations.
/// The sheet remains deformable: reactions act on individual material vertices.
pub struct Ground {
    /// Normal and tangential (x, z) multipliers per vertex.
    normal: Vec<f64>,
    tangent_x: Vec<f64>,
    tangent_z: Vec<f64>,
    /// Whether any vertex has carried a normal reaction since the last reset.
    pub touched: bool,
    pub static_friction: f64,
    pub kinetic_friction: f64,
}

impl Ground {
    pub fn new(count: usize, static_friction: f64, kinetic_friction: f64) -> Self {
        Ground { normal: vec![0.0; count], tangent_x: vec![0.0; count], tangent_z: vec![0.0; count], touched: false, static_friction, kinetic_friction }
    }
    pub fn reset(&mut self) {
        self.normal.fill(0.0);
        self.tangent_x.fill(0.0);
        self.tangent_z.fill(0.0);
        self.touched = false;
    }
    /// One sweep of the position solve; `old` holds the positions before this substep.
    /// Static friction holds a vertex whose tangential multiplier is within the static
    /// cone; otherwise the multiplier is limited to the kinetic cone.
    pub fn solve(&mut self, x: &mut [f64], old: &[f64], w: &[f64], height: f64) {
        let mu_s = js::max(0.0, self.static_friction);
        let mu_k = js::min(mu_s, js::max(0.0, self.kinetic_friction));
        let (mut x, old, w) = (W(x), R(old), R(w));
        for i in 0..self.normal.len() {
            let k = 3 * i;
            let gap = x[k + 1] - height;
            let previous = self.normal[i];
            if gap > 0.0 && previous == 0.0 { continue; }
            let weight = w[i];
            if weight <= 0.0 { continue; }
            let normal = js::max(0.0, previous - gap / weight);
            x[k + 1] += weight * (normal - previous);
            self.normal[i] = normal;
            if normal > 0.0 { self.touched = true; }
            let tx = self.tangent_x[i] - (x[k] - old[k]) / weight;
            let tz = self.tangent_z[i] - (x[k + 2] - old[k + 2]) / weight;
            let magnitude = js::hypot2(tx, tz);
            let factor = if magnitude <= mu_s * normal { 1.0 } else { js::min(1.0, mu_k * normal / js::max(magnitude, 1e-30)) };
            let (next_x, next_z) = (tx * factor, tz * factor);
            x[k] += weight * (next_x - self.tangent_x[i]);
            x[k + 2] += weight * (next_z - self.tangent_z[i]);
            self.tangent_x[i] = next_x;
            self.tangent_z[i] = next_z;
        }
    }
    /// Position projection leaves a closing velocity when a vertex started above
    /// the floor. Remove that residual after velocity reconstruction/internal
    /// damping, with friction only for this additional normal impulse. The rest
    /// of the friction budget was already spent by the position solve. There is
    /// no added restitution; elastic sheet bending can still produce a rebound.
    /// Returns whether any vertex is supported by the floor.
    pub fn solve_velocity(&mut self, x: &[f64], v: &mut [f64], height: f64) -> bool {
        let mu_s = js::max(0.0, self.static_friction);
        let mu_k = js::min(mu_s, js::max(0.0, self.kinetic_friction));
        let mut supported = false;
        for i in 0..self.normal.len() {
            let k = 3 * i;
            if x[k + 1] > height + CONTACT_TOLERANCE || self.normal[i] <= 0.0 { continue; }
            supported = true;
            let closing = js::max(0.0, -v[k + 1]);
            if closing == 0.0 { continue; }
            v[k + 1] = 0.0;
            let speed = js::hypot2(v[k], v[k + 2]);
            let reduction = if speed <= mu_s * closing { speed } else { js::min(speed, mu_k * closing) };
            let factor = 1.0 - reduction / js::max(speed, 1e-30);
            v[k] *= factor;
            v[k + 2] *= factor;
        }
        supported
    }
}
