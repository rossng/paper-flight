//! Elastic and dissipative element energies with analytic gradients. Hessians are exact for
//! the membrane's positive-definite part and Gauss-Newton elsewhere: Newton's residual is
//! always the exact gradient, so the approximation only affects the iteration count.
use crate::linalg::Sparse;
use crate::problem::Problem;
use crate::vec::*;

/// Constant-strain triangle with material-direction shape gradients (see docs/physics-upgrade.md):
/// energy = k_md e_md^2/2 + k_cd e_cd^2/2 + k_s g^2/2 with Green strains e = (|F d|^2 - 1)/2 and
/// engineering shear g = (F md).(F cd). No Poisson coupling, as in the game's model.
pub struct Membrane { pub face: [usize; 3], pub md: V3, pub cd: V3, pub k: V3 }

pub fn build_membranes(p: &Problem) -> Vec<Membrane> {
    let (t, e_md, e_cd, g) = (p.scalar("thickness"), p.scalar("youngMD"), p.scalar("youngCD"), p.scalar("shear"));
    p.faces.iter().map(|&face| {
        let [a, b, c] = face.map(|i| p.rest[i]);
        let (ab, ac) = (sub(b, a), sub(c, a));
        let l = norm(ab);
        let x = dot(ac, scale(ab, 1.0 / l));
        let y = norm(cross(ab, ac)) / l;
        // Shape-function gradients in a local frame: X along ab, Y towards c.
        let gx = [-1.0 / l, 1.0 / l, 0.0];
        let gy = [(x - l) / (l * y), -x / (l * y), 1.0 / y];
        // Material directions from sheet coordinates (u across, v along the grain).
        let uv = face.map(|i| p.uv[i]);
        let (du, dv) = (uv[1][0] - uv[0][0], uv[1][1] - uv[0][1]);
        let ul = du.hypot(dv);
        let turn = du * (uv[2][1] - uv[0][1]) - dv * (uv[2][0] - uv[0][0]);
        let orientation = if turn < 0.0 { -1.0 } else { 1.0 };
        let (cos, sin) = if ul > 1e-12 { (dv / ul, du / ul) } else { (1.0, 0.0) };
        let mut md = [0.0; 3]; let mut cd = [0.0; 3];
        for i in 0..3 { md[i] = cos * gx[i] + orientation * sin * gy[i]; cd[i] = sin * gx[i] - orientation * cos * gy[i]; }
        let area = l * y / 2.0;
        Membrane { face, md, cd, k: [e_md * t * area, e_cd * t * area, g * t * area] }
    }).collect()
}

impl Membrane {
    fn directions(&self, x: &[f64]) -> (V3, V3) {
        let p = self.face.map(|i| get(x, i));
        let mut u = [0.0; 3]; let mut v = [0.0; 3];
        for i in 0..3 { for d in 0..3 { u[d] += self.md[i] * p[i][d]; v[d] += self.cd[i] * p[i][d]; } }
        (u, v)
    }
    pub fn strains(&self, x: &[f64]) -> V3 {
        let (u, v) = self.directions(x);
        [(dot(u, u) - 1.0) / 2.0, (dot(v, v) - 1.0) / 2.0, dot(u, v)]
    }
    pub fn energy(&self, x: &[f64]) -> f64 {
        let s = self.strains(x);
        0.5 * (self.k[0] * s[0] * s[0] + self.k[1] * s[1] * s[1] + self.k[2] * s[2] * s[2])
    }
    pub fn add(&self, x: &[f64], g: &mut [f64], mut h: Option<&mut Sparse>) -> f64 {
        let (u, v) = self.directions(x);
        let s = [(dot(u, u) - 1.0) / 2.0, (dot(v, v) - 1.0) / 2.0, dot(u, v)];
        let (md, cd, k) = (self.md, self.cd, self.k);
        for i in 0..3 {
            let shear = add(scale(v, md[i]), scale(u, cd[i]));
            let force = add(add(scale(u, k[0] * s[0] * md[i]), scale(v, k[1] * s[1] * cd[i])), scale(shear, k[2] * s[2]));
            axpy(g, self.face[i], 1.0, force);
        }
        if let Some(h) = h.as_deref_mut() {
            // Geometric stiffness of a stretched direction is positive; a compressed one is dropped.
            let (tension_u, tension_v) = (k[0] * s[0].max(0.0), k[1] * s[1].max(0.0));
            for i in 0..3 { for j in i..3 {
                let (si, sj) = (add(scale(v, md[i]), scale(u, cd[i])), add(scale(v, md[j]), scale(u, cd[j])));
                let mut block = [[0.0; 3]; 3];
                let (uu, vv, ss) = (outer(u, u), outer(v, v), outer(si, sj));
                for a in 0..3 { for b in 0..3 {
                    block[a][b] = k[0] * md[i] * md[j] * uu[a][b] + k[1] * cd[i] * cd[j] * vv[a][b] + k[2] * ss[a][b];
                } }
                let diagonal = tension_u * md[i] * md[j] + tension_v * cd[i] * cd[j];
                for a in 0..3 { block[a][a] += diagonal; }
                h.add(self.face[i], self.face[j], &block);
            } }
        }
        0.5 * (k[0] * s[0] * s[0] + k[1] * s[1] * s[1] + k[2] * s[2] * s[2])
    }
}

/// Signed dihedral angle and its gradient. Vertex order: two opposite vertices, then the shared edge.
pub fn hinge_angle(p: [V3; 4]) -> f64 {
    let e = sub(p[3], p[2]);
    let n1 = cross(sub(p[2], p[0]), sub(p[3], p[0]));
    let n2 = cross(sub(p[3], p[1]), sub(p[2], p[1]));
    (dot(cross(n1, n2), e) / norm(e).max(1e-12)).atan2(dot(n1, n2))
}
pub fn hinge_gradient(p: [V3; 4]) -> Option<[V3; 4]> {
    let e = sub(p[3], p[2]);
    let l = norm(e);
    let raw1 = cross(sub(p[2], p[0]), sub(p[3], p[0]));
    let raw2 = cross(sub(p[3], p[1]), sub(p[2], p[1]));
    let (a, b) = (dot(raw1, raw1), dot(raw2, raw2));
    if l < 1e-10 || a < 1e-18 || b < 1e-18 { return None; }
    let (n1, n2) = (scale(raw1, 1.0 / a), scale(raw2, 1.0 / b));
    let a2 = -dot(sub(p[0], p[3]), e) / l; let b2 = -dot(sub(p[1], p[3]), e) / l;
    let a3 = -dot(sub(p[2], p[0]), e) / l; let b3 = -dot(sub(p[2], p[1]), e) / l;
    Some([scale(n1, -l), scale(n2, -l), add(scale(n1, a2), scale(n2, b2)), add(scale(n1, a3), scale(n2, b3))])
}

/// Add a Gauss-Newton term k J J^T for a scalar quantity with gradient J.
pub fn add_outer(h: &mut Sparse, ids: &[usize], gradient: &[V3], k: f64) {
    for i in 0..ids.len() { for j in i..ids.len() {
        let mut block = outer(gradient[i], gradient[j]);
        for row in block.iter_mut() { for v in row.iter_mut() { *v *= k; } }
        h.add(ids[i], ids[j], &block);
    } }
}

/// Crease state for the game's rate-independent return mapping with linear hardening.
#[derive(Clone, Copy, Default)]
pub struct Plastic { pub offset: f64, pub accumulated: f64 }
impl Plastic {
    /// Returns the plastic rotation added.
    pub fn update(&mut self, angle: f64, rest: f64, yield_angle: f64, hardening: f64) -> f64 {
        let error = wrap_angle(angle - rest - self.offset);
        let limit = yield_angle + hardening * self.accumulated;
        if error.abs() <= limit { return 0.0; }
        let increment = (error.abs() - limit) / (1.0 + hardening);
        self.offset += error.signum() * increment;
        self.accumulated += increment;
        increment
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn perturbed(points: &[V3], seed: u64, amount: f64) -> Vec<f64> {
        let mut s = seed;
        points.iter().flat_map(|p| {
            let mut out = [0.0; 3];
            for d in 0..3 { s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407); out[d] = p[d] + amount * (((s >> 11) as f64) / ((1u64 << 53) as f64) - 0.5); }
            out
        }).collect()
    }
    #[test]
    fn membrane_gradient_matches_finite_differences() {
        let m = Membrane { face: [0, 1, 2], md: [-9.0, 10.0, -1.0], cd: [-3.0, -4.0, 7.0], k: [300.0, 150.0, 80.0] };
        let x = perturbed(&[[0.0, 0.0, 0.0], [0.1, 0.0, 0.01], [0.02, 0.12, -0.01]], 3, 0.02);
        let mut g = vec![0.0; 9];
        m.add(&x, &mut g, None);
        for k in 0..9 {
            let (mut hi, mut lo) = (x.clone(), x.clone()); hi[k] += 1e-6; lo[k] -= 1e-6;
            let fd = (m.energy(&hi) - m.energy(&lo)) / 2e-6;
            assert!((fd - g[k]).abs() < 1e-6 * (1.0 + g[k].abs()), "dof {k}: {fd} vs {}", g[k]);
        }
    }
    #[test]
    fn membrane_rest_state_is_strain_free_and_rigid_motion_invariant() {
        let rest = [[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [0.01, 0.04, 0.0]];
        let mut problem = Problem::parse("paperflight-reference-problem 1\nscalar thickness 1e-4\nscalar youngMD 4e9\nscalar youngCD 2e9\nscalar shear 1e9\nvertices 3\n0 0 0 0 0 0 1 0 0\n0.05 0 0 0 0 0 1 0 0.05\n0.01 0.04 0 0 0 0 1 -0.04 0.01\nfaces 1\n0 1 2 -1\nend\n").unwrap();
        problem.rest = rest.to_vec();
        let m = &build_membranes(&problem)[0];
        let flat: Vec<f64> = rest.iter().flatten().copied().collect();
        assert!(m.strains(&flat).iter().all(|s| s.abs() < 1e-12));
        // Rotate about z by 0.7 rad and translate.
        let (c, s) = (0.7f64.cos(), 0.7f64.sin());
        let moved: Vec<f64> = rest.iter().flat_map(|p| [c * p[0] - s * p[1] + 2.0, s * p[0] + c * p[1] - 1.0, p[2] + 0.3]).collect();
        assert!(m.strains(&moved).iter().all(|s| s.abs() < 1e-12));
        // Stretch 1% along the grain (material v = rest x here): only the MD strain responds.
        let stretched: Vec<f64> = rest.iter().flat_map(|p| [1.01 * p[0], p[1], p[2]]).collect();
        let strain = m.strains(&stretched);
        assert!((strain[0] - (1.01f64 * 1.01 - 1.0) / 2.0).abs() < 1e-12 && strain[1].abs() < 1e-12 && strain[2].abs() < 1e-12, "{strain:?}");
    }
    #[test]
    fn hinge_gradient_matches_finite_differences() {
        let base = [[0.0, 0.05, 0.01], [0.01, -0.04, 0.02], [-0.03, 0.0, 0.0], [0.04, 0.005, -0.004]];
        for seed in 1..6 {
            let x = perturbed(&base, seed, 0.01);
            let points = |x: &[f64]| [get(x, 0), get(x, 1), get(x, 2), get(x, 3)];
            let g = hinge_gradient(points(&x)).unwrap();
            for k in 0..12 {
                let (mut hi, mut lo) = (x.clone(), x.clone()); hi[k] += 1e-7; lo[k] -= 1e-7;
                let fd = wrap_angle(hinge_angle(points(&hi)) - hinge_angle(points(&lo))) / 2e-7;
                assert!((fd - g[k / 3][k % 3]).abs() < 1e-5 * (1.0 + fd.abs()), "seed {seed} dof {k}: {fd} vs {}", g[k / 3][k % 3]);
            }
        }
    }
    #[test]
    fn crease_yields_hardens_and_keeps_its_set() {
        let mut p = Plastic::default();
        assert_eq!(p.update(0.3, 0.0, 0.35, 0.08), 0.0);
        let added = p.update(0.5, 0.0, 0.35, 0.08);
        assert!((added - 0.15 / 1.08).abs() < 1e-12 && (p.offset - added).abs() < 1e-12);
        assert_eq!(p.update(0.2, 0.0, 0.35, 0.08), 0.0);
    }
}
