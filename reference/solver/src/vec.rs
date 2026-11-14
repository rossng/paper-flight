//! Small 3-vector helpers. SI units throughout.
pub type V3 = [f64; 3];
pub type M3 = [[f64; 3]; 3];

#[inline] pub fn add(a: V3, b: V3) -> V3 { [a[0] + b[0], a[1] + b[1], a[2] + b[2]] }
#[inline] pub fn sub(a: V3, b: V3) -> V3 { [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
#[inline] pub fn scale(a: V3, s: f64) -> V3 { [a[0] * s, a[1] * s, a[2] * s] }
#[inline] pub fn dot(a: V3, b: V3) -> f64 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
#[inline] pub fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
#[inline] pub fn norm(a: V3) -> f64 { dot(a, a).sqrt() }
#[inline] pub fn unit(a: V3) -> V3 { scale(a, 1.0 / norm(a).max(1e-12)) }
#[inline] pub fn outer(a: V3, b: V3) -> M3 {
    [[a[0] * b[0], a[0] * b[1], a[0] * b[2]], [a[1] * b[0], a[1] * b[1], a[1] * b[2]], [a[2] * b[0], a[2] * b[1], a[2] * b[2]]]
}
#[inline] pub fn get(x: &[f64], i: usize) -> V3 { [x[3 * i], x[3 * i + 1], x[3 * i + 2]] }
#[inline] pub fn axpy(x: &mut [f64], i: usize, s: f64, v: V3) {
    x[3 * i] += s * v[0]; x[3 * i + 1] += s * v[1]; x[3 * i + 2] += s * v[2];
}
pub fn wrap_angle(a: f64) -> f64 { a.sin().atan2(a.cos()) }
pub fn smoothstep(lo: f64, hi: f64, x: f64) -> f64 { let t = ((x - lo) / (hi - lo)).clamp(0.0, 1.0); t * t * (3.0 - 2.0 * t) }
