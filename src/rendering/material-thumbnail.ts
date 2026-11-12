import type { MaterialId } from '../physics/materials';

/** Tiny craft-supply illustrations, shared by the tile and its picker. */
export function materialThumbnail(material:MaterialId){
 const sheets:Record<MaterialId,string>={
  paper:'<path d="M15 10 46 7 51 43 20 47Z" fill="#fff9e7" stroke="#a79976"/><path d="m23 19 17-2m-16 9 18-2m-17 9 13-2" stroke="#d5c9a6"/>',
  tissue:'<path d="M9 17Q21 4 33 13T55 12L50 41Q37 49 27 41T10 45Z" fill="#fffdfb" fill-opacity=".7" stroke="#a5b8b0"/><path d="M18 14q-6 16 0 29m15-28q-8 10 0 26m13-27q-6 12-2 27" fill="none" stroke="#c8d6cd"/>',
  foil:'<path d="m11 12 33-6 20 12-5 28-35 3L7 30Z" fill="#c7d6dd" stroke="#788d9b"/><path d="m11 12 20 11 2-17 6 25 14-13-5 28-16-7-19 10 18-26L7 30" fill="#edf4f4" stroke="#9bacb6"/><path d="m31 23 8 8-7 8" fill="#8c9eab"/>',
  steel:'<path d="m11 15 40-6 3 35-40 6Z" fill="#536977" stroke="#425966"/><path d="m9 10 40-6 3 35-40 6Z" fill="#9daeb6" stroke="#536977"/><path d="m20 12 23-3m-24 9 23-3" stroke="#c9d5d9"/><g fill="#536977"><circle cx="15" cy="15" r="2"/><circle cx="44" cy="10" r="2"/><circle cx="18" cy="39" r="2"/><circle cx="46" cy="34" r="2"/></g>',
 };
 return `<svg class="material-preview" viewBox="0 0 64 56" fill="none" stroke-linejoin="round" stroke-width="1.5" aria-hidden="true">${sheets[material]}</svg>`;
}
