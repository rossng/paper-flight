import { sheetMaterial, bendingRigidity } from './sheet-material';
export type MaterialId='paper'|'tissue'|'foil'|'steel';
/** UI stiffness ratios are derived from nominal constitutive properties.
 * The solver uses SI properties; these ratios preserve the existing control API. */
const ratio=(id:MaterialId,gsm:number)=>bendingRigidity(sheetMaterial(id,gsm))/bendingRigidity(sheetMaterial('paper',80));
export const materials={
 paper:{name:'Paper',gsm:80,stiffness:1,crease:1,color:0xfffcf1,metalness:0,roughness:.9,opacity:1,note:'Your everyday flying sheet.'},
 tissue:{name:'Tissue paper',gsm:18,stiffness:ratio('tissue',18),crease:1,color:0xfffdfb,metalness:0,roughness:1,opacity:.7,note:'Featherlight, fluttery, and holds a fold.'},
 foil:{name:'Kitchen foil',gsm:135,stiffness:ratio('foil',135),crease:1,color:0xc7d6dd,metalness:.65,roughness:.35,opacity:1,note:'Shiny wings and soft creases.'},
 steel:{name:'Steel plate',gsm:7850,stiffness:ratio('steel',7850),crease:1,color:0x879aa8,metalness:.8,roughness:.4,opacity:1,note:'A 1 mm flying doorstop. Good luck.'},
} as const;
export function materialSettings(material:MaterialId,paperWeight=80){
 const preset=materials[material];
 return {material,paperWeight:material==='paper'?paperWeight:preset.gsm,stiffness:material==='paper'?(paperWeight/80)**3:preset.stiffness,crease:preset.crease};
}
