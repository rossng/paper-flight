//! One source for a kernel's scalar and two-at-a-time forms. A kernel written against
//! `Lanes` runs on `f64` for one element and on `F2` (WebAssembly's 128-bit SIMD) for two
//! independent elements at once. Every operation is the IEEE operation on each lane, in
//! the order the kernel's source gives, so both forms produce the same bits.
use core::arch::wasm32::*;
use std::ops::{Add, BitAnd, BitOr, Div, Mul, Neg, Not, Sub};

pub trait Lanes: Copy + Add<Output = Self> + Sub<Output = Self> + Mul<Output = Self> + Div<Output = Self> + Neg<Output = Self> {
    /// One element index per lane.
    type Idx: Copy;
    /// One truth value per lane.
    type Mask: Copy + BitAnd<Output = Self::Mask> + BitOr<Output = Self::Mask> + Not<Output = Self::Mask>;
    /// Whether this form carries two elements.
    const PAIR: bool;
    fn splat(v: f64) -> Self;
    /// `p[scale * i + offset]` for each lane's `i`.
    unsafe fn get(p: *const f64, i: Self::Idx, scale: usize, offset: usize) -> Self;
    unsafe fn set(self, p: *mut f64, i: Self::Idx, scale: usize, offset: usize);
    /// `p[scale * i + offset]` as an index, for each lane's `i`.
    unsafe fn id(p: *const i32, i: Self::Idx, scale: usize, offset: usize) -> Self::Idx;
    fn sqrt(self) -> Self;
    fn abs(self) -> Self;
    fn atan2(self, o: Self) -> Self;
    fn lt(self, o: Self) -> Self::Mask;
    fn le(self, o: Self) -> Self::Mask;
    fn gt(self, o: Self) -> Self::Mask;
    /// `a` where the mask is set, else `b`.
    fn select(m: Self::Mask, a: Self, b: Self) -> Self;
    fn any(m: Self::Mask) -> bool;
    fn all(m: Self::Mask) -> bool;
}

impl Lanes for f64 {
    type Idx = usize;
    type Mask = bool;
    const PAIR: bool = false;
    #[inline(always)]
    fn splat(v: f64) -> f64 { v }
    #[inline(always)]
    unsafe fn get(p: *const f64, i: usize, scale: usize, offset: usize) -> f64 { *p.add(scale * i + offset) }
    #[inline(always)]
    unsafe fn set(self, p: *mut f64, i: usize, scale: usize, offset: usize) { *p.add(scale * i + offset) = self; }
    #[inline(always)]
    unsafe fn id(p: *const i32, i: usize, scale: usize, offset: usize) -> usize { *p.add(scale * i + offset) as usize }
    #[inline(always)]
    fn sqrt(self) -> f64 { f64::sqrt(self) }
    #[inline(always)]
    fn abs(self) -> f64 { f64::abs(self) }
    #[inline(always)]
    fn atan2(self, o: f64) -> f64 { f64::atan2(self, o) }
    #[inline(always)]
    fn lt(self, o: f64) -> bool { self < o }
    #[inline(always)]
    fn le(self, o: f64) -> bool { self <= o }
    #[inline(always)]
    fn gt(self, o: f64) -> bool { self > o }
    #[inline(always)]
    fn select(m: bool, a: f64, b: f64) -> f64 { if m { a } else { b } }
    #[inline(always)]
    fn any(m: bool) -> bool { m }
    #[inline(always)]
    fn all(m: bool) -> bool { m }
}

/// Two `f64` lanes.
#[derive(Clone, Copy)]
pub struct F2(pub v128);
/// Two lane masks, all ones or all zeros each.
#[derive(Clone, Copy)]
pub struct M2(pub v128);

impl Add for F2 { type Output = F2; #[inline(always)] fn add(self, o: F2) -> F2 { F2(f64x2_add(self.0, o.0)) } }
impl Sub for F2 { type Output = F2; #[inline(always)] fn sub(self, o: F2) -> F2 { F2(f64x2_sub(self.0, o.0)) } }
impl Mul for F2 { type Output = F2; #[inline(always)] fn mul(self, o: F2) -> F2 { F2(f64x2_mul(self.0, o.0)) } }
impl Div for F2 { type Output = F2; #[inline(always)] fn div(self, o: F2) -> F2 { F2(f64x2_div(self.0, o.0)) } }
impl Neg for F2 { type Output = F2; #[inline(always)] fn neg(self) -> F2 { F2(f64x2_neg(self.0)) } }
impl BitAnd for M2 { type Output = M2; #[inline(always)] fn bitand(self, o: M2) -> M2 { M2(v128_and(self.0, o.0)) } }
impl Not for M2 { type Output = M2; #[inline(always)] fn not(self) -> M2 { M2(v128_not(self.0)) } }
impl BitOr for M2 { type Output = M2; #[inline(always)] fn bitor(self, o: M2) -> M2 { M2(v128_or(self.0, o.0)) } }

impl F2 {
    #[inline(always)]
    pub fn lane(self, i: usize) -> f64 { if i == 0 { f64x2_extract_lane::<0>(self.0) } else { f64x2_extract_lane::<1>(self.0) } }
}

impl Lanes for F2 {
    type Idx = [usize; 2];
    type Mask = M2;
    const PAIR: bool = true;
    #[inline(always)]
    fn splat(v: f64) -> F2 { F2(f64x2_splat(v)) }
    #[inline(always)]
    unsafe fn get(p: *const f64, i: [usize; 2], scale: usize, offset: usize) -> F2 { F2(f64x2(*p.add(scale * i[0] + offset), *p.add(scale * i[1] + offset))) }
    #[inline(always)]
    unsafe fn set(self, p: *mut f64, i: [usize; 2], scale: usize, offset: usize) {
        *p.add(scale * i[0] + offset) = f64x2_extract_lane::<0>(self.0);
        *p.add(scale * i[1] + offset) = f64x2_extract_lane::<1>(self.0);
    }
    #[inline(always)]
    unsafe fn id(p: *const i32, i: [usize; 2], scale: usize, offset: usize) -> [usize; 2] { [*p.add(scale * i[0] + offset) as usize, *p.add(scale * i[1] + offset) as usize] }
    #[inline(always)]
    fn sqrt(self) -> F2 { F2(f64x2_sqrt(self.0)) }
    #[inline(always)]
    fn abs(self) -> F2 { F2(f64x2_abs(self.0)) }
    #[inline(always)]
    fn atan2(self, o: F2) -> F2 { F2(f64x2(self.lane(0).atan2(o.lane(0)), self.lane(1).atan2(o.lane(1)))) }
    #[inline(always)]
    fn lt(self, o: F2) -> M2 { M2(f64x2_lt(self.0, o.0)) }
    #[inline(always)]
    fn le(self, o: F2) -> M2 { M2(f64x2_le(self.0, o.0)) }
    #[inline(always)]
    fn gt(self, o: F2) -> M2 { M2(f64x2_gt(self.0, o.0)) }
    #[inline(always)]
    fn select(m: M2, a: F2, b: F2) -> F2 { F2(v128_bitselect(a.0, b.0, m.0)) }
    #[inline(always)]
    fn any(m: M2) -> bool { v128_any_true(m.0) }
    #[inline(always)]
    fn all(m: M2) -> bool { i64x2_all_true(m.0) }
}

/// An order for one Gauss–Seidel sweep over constraints that gives the same result as
/// index order, arranged so that neighbours in the order are independent where the mesh
/// allows it. Two constraints that share no vertex commute, so any order that keeps every
/// vertex-sharing pair in index order is equivalent. Returns slots of two constraint
/// indices, the second −1 for a slot of one; the constraints of a slot share no vertex.
/// The slots reversed are equivalent to reverse index order.
pub fn sweep_order(ids: &[i32], size: usize) -> Vec<i32> {
    let count = ids.len() / size;
    let vertices = ids.iter().map(|&v| v as usize + 1).max().unwrap_or(0);
    // The nearest earlier constraint on each of a constraint's vertices must precede it.
    let mut last = vec![usize::MAX; vertices];
    let mut waiting = vec![0u32; count];
    let mut followers: Vec<Vec<u32>> = vec![Vec::new(); count];
    for c in 0..count {
        for j in 0..size {
            let v = ids[c * size + j] as usize;
            let before = last[v];
            if before != usize::MAX && before != c && !followers[before].contains(&(c as u32)) {
                followers[before].push(c as u32);
                waiting[c] += 1;
            }
            last[v] = c;
        }
    }
    // Longest chain of followers from each constraint: the most urgent goes first.
    let mut height = vec![1u32; count];
    for c in (0..count).rev() {
        for &f in &followers[c] { height[c] = height[c].max(height[f as usize] + 1); }
    }
    let mut ready: Vec<u32> = (0..count as u32).filter(|&c| waiting[c as usize] == 0).collect();
    let mut slots = Vec::with_capacity(2 * count);
    let take = |ready: &mut Vec<u32>| -> Option<u32> {
        let mut best: Option<usize> = None;
        for (at, &c) in ready.iter().enumerate() {
            let better = match best {
                None => true,
                Some(b) => { let o = ready[b]; height[c as usize] > height[o as usize] || (height[c as usize] == height[o as usize] && c < o) }
            };
            if better { best = Some(at); }
        }
        best.map(|at| ready.swap_remove(at))
    };
    while !ready.is_empty() {
        // Both are taken before either releases its followers, so they are independent.
        let first = take(&mut ready).unwrap();
        let second = take(&mut ready);
        slots.push(first as i32);
        slots.push(second.map_or(-1, |c| c as i32));
        for c in [Some(first), second].into_iter().flatten() {
            for &f in &followers[c as usize] {
                waiting[f as usize] -= 1;
                if waiting[f as usize] == 0 { ready.push(f); }
            }
        }
    }
    debug_assert_eq!(slots.iter().filter(|&&c| c >= 0).count(), count);
    slots
}
