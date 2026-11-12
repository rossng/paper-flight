/** Small allocation-light vector helpers; no renderer dependency. Units: SI. */
export type V3 = [number, number, number];
export const add = (a: V3, b: V3): V3 => [a[0]+b[0], a[1]+b[1], a[2]+b[2]];
export const sub = (a: V3, b: V3): V3 => [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
export const scale = (a: V3, s: number): V3 => [a[0]*s, a[1]*s, a[2]*s];
export const dot = (a: V3, b: V3) => a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
export const cross = (a: V3, b: V3): V3 => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
export const length = (a: V3) => Math.hypot(a[0],a[1],a[2]);
export const unit = (a: V3): V3 => scale(a, 1 / Math.max(length(a), 1e-12));
export const clamp = (x: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, x));
export const wrapAngle = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));
export const radians = (a: number) => a * Math.PI / 180;
