//! Unchecked indexing for hot kernels. Indices come from the TypeScript set-up (mesh ids,
//! precomputed slots), so they are in range by construction; debug builds still check.
use std::ops::{Index, IndexMut};

#[derive(Clone, Copy)]
pub struct R<'a, T>(pub &'a [T]);
impl<T> Index<usize> for R<'_, T> {
    type Output = T;
    #[inline(always)]
    fn index(&self, i: usize) -> &T {
        debug_assert!(i < self.0.len());
        unsafe { self.0.get_unchecked(i) }
    }
}
impl<T> R<'_, T> {
    #[inline(always)]
    pub fn len(&self) -> usize { self.0.len() }
}
pub struct W<'a, T>(pub &'a mut [T]);
impl<T> Index<usize> for W<'_, T> {
    type Output = T;
    #[inline(always)]
    fn index(&self, i: usize) -> &T {
        debug_assert!(i < self.0.len());
        unsafe { self.0.get_unchecked(i) }
    }
}
impl<T> IndexMut<usize> for W<'_, T> {
    #[inline(always)]
    fn index_mut(&mut self, i: usize) -> &mut T {
        debug_assert!(i < self.0.len());
        unsafe { self.0.get_unchecked_mut(i) }
    }
}
