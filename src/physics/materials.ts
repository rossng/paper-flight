export type MaterialId='paper'|'tissue'|'foil'|'steel';
/** Playful effective sheets, not calibrated material or plastic-folding models. */
export const materials={
 paper:{name:'Paper',gsm:80,stiffness:1,crease:1,color:0xfffcf1,metalness:0,roughness:.9,note:'Your everyday flying sheet.'},
 tissue:{name:'Tissue paper',gsm:18,stiffness:.015,crease:.12,color:0xfffdfb,metalness:0,roughness:1,note:'Featherlight. Very floppy. Loves a breeze.'},
 foil:{name:'Kitchen foil',gsm:135,stiffness:3,crease:.2,color:0xc7d6dd,metalness:.65,roughness:.35,note:'Shiny wings and soft creases.'},
 steel:{name:'Steel plate',gsm:7850,stiffness:70000,crease:70000,color:0x879aa8,metalness:.8,roughness:.4,note:'A 1 mm flying doorstop. Good luck.'},
} as const;
export function materialSettings(material:MaterialId,paperWeight=80){
 const preset=materials[material];
 return {material,paperWeight:material==='paper'?paperWeight:preset.gsm,stiffness:material==='paper'?(paperWeight/80)**3:preset.stiffness,crease:preset.crease};
}
