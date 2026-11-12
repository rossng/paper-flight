import {recipes} from './recipes';
import type {FoldRecipe} from './model';

export interface PlaneDesign {
 id:string; name:string; detail:string; note:string;
 kind:'sheet'|'effective'; adjustableSpan:boolean; recipe?:FoldRecipe;
}
const details:Record<string,{name:string;detail:string;note:string;adjustableSpan?:boolean}>={
 basic:{name:'The Basic',detail:'Broad wings',note:'Independent reconstruction of a conventional corner-fold plane.',adjustableSpan:true},
 dart:{name:'Folded Dart',detail:'Swept dart',note:'Independent reconstruction with two successive shoulder folds. Authored wing width.'},
 stable:{name:'The Stable',detail:'Locking flap',note:'Selective nose flap; the geometry does not certify a physical friction lock.',adjustableSpan:true},
 'heavy-nose':{name:'Heavy Nose',detail:'Pleated nose · dart',note:'Three nose reversals concentrate material at the front. Independently chosen pleat and wing-root dimensions.'},
 hunting:{name:'Hunting',detail:'Rolled edge · winglets',note:'Independent rolled-edge study with 25.4 mm bands and 12 mm winglets. Winglets are folded before deployment; optional tape is omitted.'},
 buzz:{name:'The Buzz',detail:'Nose hem · glider',note:'Independent nose-reversal and underfold study. Tail trim and optional tape are omitted.'},
 'v-wing':{name:'V-Wing',detail:'Square sheet · tapered wings',note:'Starts with a complete 215.9 mm square, rather than simulating the reference’s trimming of a rectangle. No staple or tail trim.'},
 underside:{name:'Underside',detail:'Landscape sheet · broad wing',note:'Independent landscape-sheet reconstruction with a doubled leading band. Corner geometry is approximate; optional tape is omitted.'},
 raven:{name:'Raven',detail:'Layered nose · selective flap',note:'Independent layered-dart and returning-flap study. No tearing or finite-thickness lock is represented.'},
};
export const designs:PlaneDesign[]=[
 {id:'legacy',name:'Original dart',detail:'Effective mesh · quick starter',note:'An effective flight surface, not a complete folded sheet. Choose a sheet design for a .fold download.',kind:'effective',adjustableSpan:true},
 ...recipes.map(recipe=>({id:recipe.id,...details[recipe.id],kind:'sheet' as const,adjustableSpan:details[recipe.id].adjustableSpan??false,recipe})),
];
export const sheetDesigns=designs.filter(d=>d.kind==='sheet');
export function findDesign(id:string){return designs.find(d=>d.id===id);}
export const reuseBasis='Independently authored Paper Flight coordinates and instructions, Blue Oak Model License 1.0.0. Reference links are attribution only; no third-party templates, photographs or downloaded geometry are included.';
