//! Number semantics the solver's arithmetic was written against (it began as JavaScript):
//! `Math.max`/`Math.min` with NaN and signed zeros, `Math.sign`, and 32-bit integer
//! conversion for the spatial hash.

/// Euclidean norm. Overflow is impossible at the solver's magnitudes.
#[inline(always)]
pub fn hypot3(a: f64, b: f64, c: f64) -> f64 { (a * a + b * b + c * c).sqrt() }
#[inline(always)]
pub fn hypot2(a: f64, b: f64) -> f64 { (a * a + b * b).sqrt() }

/// The larger of two numbers: NaN if either is NaN; +0 beats -0. Ordered operands take
/// the first two comparisons.
#[inline(always)]
pub fn max(a: f64, b: f64) -> f64 {
    if a > b { a } else if b > a { b } else if a == b { if a == 0.0 && a.is_sign_negative() { b } else { a } } else { f64::NAN }
}
/// The smaller of two numbers: NaN if either is NaN; -0 beats +0.
#[inline(always)]
pub fn min(a: f64, b: f64) -> f64 {
    if a < b { a } else if b < a { b } else if a == b { if a == 0.0 && a.is_sign_positive() { b } else { a } } else { f64::NAN }
}
#[inline]
pub fn clamp(x: f64, lo: f64, hi: f64) -> f64 { max(lo, min(hi, x)) }
/// Sign as -1, 0 or 1: zero (and NaN) map to themselves.
#[inline]
pub fn sign(x: f64) -> f64 {
    if x > 0.0 { 1.0 } else if x < 0.0 { -1.0 } else { x }
}
/// An angle wrapped to (-pi, pi].
#[inline]
pub fn wrap_angle(a: f64) -> f64 { a.sin().atan2(a.cos()) }
/// A whole number reduced modulo 2^32 to a signed 32-bit integer.
#[inline]
pub fn to_int32(v: f64) -> i32 {
    if !v.is_finite() { return 0; }
    let m = v.trunc() % 4294967296.0;
    let m = if m < 0.0 { m + 4294967296.0 } else { m };
    (m as u64 as u32) as i32
}
