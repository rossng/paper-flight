import './style.css';
import { DEFAULTS, FixedClock, Simulation, type Parameters } from './physics/simulation';
import { materials, materialSettings, type MaterialId } from './physics/materials';
import { airVelocity } from './physics/weather';
import { elasticPull, pullTravel, springSpeed, springReleaseDuration, springRemaining } from './physics/launcher';
import { FlightRecording } from './rendering/flight-recording';
import { designMesh,flightRecipe } from './folding/design';
import { FoldSequence, FoldAnimator } from './folding/animation';
import { FoldingView } from './folding/view';
import { planeThumbnail } from './folding/thumbnail';
import { Viewport } from './rendering/viewport';
const $=<T extends HTMLElement=HTMLElement>(id:string)=>document.getElementById(id) as T;
const icon='<svg viewBox="0 0 32 32" fill="none" aria-hidden="true"><path d="M28 4 4 13l10 5 5 10L28 4Z M14 18 28 4" stroke="currentColor" stroke-width="1.8" stroke-linejoin="round"/></svg>';
$('app').innerHTML=`
<main class="game" data-mode="workbench"><section class="stage" aria-label="Paper Flight game">
<div id="viewport"></div>
<div class="game-top"><div class="game-logo">${icon}<span>PAPER<br><b>FLIGHT!</b></span></div><nav aria-label="Game mode"><button id="bench-tab" class="selected" aria-pressed="true">✂ <span>FOLD</span></button><button id="throw-tab" aria-pressed="false">➤ <span>FLY</span></button></nav><button id="about" class="round-button" aria-label="How to play">?</button></div>
<div class="stage-top"><span id="location">THE FOLD ZONE</span><div id="design-picker"><span>PLANE</span><button id="plane-design" aria-haspopup="listbox" aria-expanded="false" aria-controls="plane-options"></button><div id="plane-options" role="listbox" aria-label="Choose your plane" hidden></div></div><button id="camera" class="small-button">Reset view ↺</button></div>
<div id="sheet-wrap"><div class="desk-pencil" aria-hidden="true"></div><div class="desk-sticker" aria-hidden="true">100%<br>PAPER<br><span>0% boring</span></div><div class="fold-progress" aria-label="Folding progress"><span>FLIGHT CHECK</span><div id="fold-checks"></div></div><div id="sheet" aria-label="Interactive 3D folding preview"></div><div class="mat-label" aria-hidden="true">✂ FOLD LAB</div></div>
<div id="fold-prompt"><div><span id="step-count">FOLD 1 / 3</span><h2 id="fold-title">Fold the corners!</h2><p id="fold-description">Drag to inspect. Tap to make your first fold.</p></div><div class="fold-buttons"><div class="fold-step-navigation"><button id="undo-fold" class="secondary" aria-label="Back: unfold the last step" disabled>← Back</button><button id="fold" class="secondary">Next fold →</button></div><button id="finish-fold" class="primary">Fold plane ✓</button><button id="go-throw" class="primary launch-button" hidden>LET’S FLY! ➤</button></div></div>
<div id="workbench-tools" class="tool-dock"><div class="tool paper-tool"><h3>PAPER PICK</h3><div class="material-choices"><div class="choices" id="paper-choices"><button data-weight="60"><i class="paper-swatch light"></i>Light<span>60 gsm</span></button><button data-weight="80" class="selected"><i class="paper-swatch"></i>Classic<span>80 gsm</span></button><button data-weight="110"><i class="paper-swatch heavy"></i>Heavy<span>110 gsm</span></button></div><label class="special-material" id="special-tile"><span aria-hidden="true">✦</span><select id="special-material" aria-label="Special material"><option value="paper">Special…</option><option value="tissue">Tissue paper</option><option value="foil">Kitchen foil</option><option value="steel">Steel plate</option></select></label></div><p id="paper-note" hidden>Classic paper.</p></div><div class="tool"><h3>SHEET SIZE</h3><div class="segmented" id="size-choices"><button data-size="0.85">S</button><button data-size="1" class="selected">M</button><button data-size="1.15">L</button></div><button id="unfold" class="small-button">↺ New sheet</button></div><div class="tool wing-tool"><h3>WING SHOP</h3><div class="wing-controls"><button id="narrow" aria-label="Narrower wings">−</button><span id="wing-label">Classic wings</span><button id="wide" aria-label="Wider wings">+</button></div><div class="segmented" id="fold-choices"><button data-dihedral="0">Flat</button><button data-dihedral="8" class="selected">Little V</button><button data-dihedral="16">Big V</button></div></div></div>
<div id="launch-overlay" hidden><div class="field-note"><span id="flight-state">READY, PILOT?</span><p id="launch-help">Grab the plane. Pull back. Let it rip.</p></div><svg id="bands" aria-hidden="true"><line id="band-left"/><line id="band-right"/></svg><div id="anchor-left" class="sling-anchor"></div><div id="anchor-right" class="sling-anchor"></div><button id="sling" aria-label="Drag this plane to launch. Arrow keys aim and set power; Space launches."><span>GRAB ME</span></button>
<div id="launch-controls"><div class="speed-gauge"><span>LAUNCH SPEED</span><strong id="launch-speed">0.0 <small>m/s</small></strong><div class="power-track"><div id="power-fill"></div></div><p id="power-label">PULL THE PLANE BACK</p></div><div class="pitch-control"><span>PLANE PITCH</span><button id="pitch-dial" role="slider" aria-valuemin="-180" aria-valuemax="180" aria-valuenow="15" aria-label="Plane pitch: full rotation. Drag up or down; arrow keys adjust, Page Up or Down makes larger turns, Home levels, End inverts."><i id="pitch-needle">➤</i><b id="pitch-value">15°</b></button><div class="pitch-buttons"><button id="pitch-down" aria-label="Decrease pitch">−</button><small>nose down / up</small><button id="pitch-up" aria-label="Increase pitch">+</button></div></div></div></div>
<button id="weather-button" class="weather-button" aria-haspopup="dialog" aria-label="Launch weather options" hidden><span id="wind-arrow" aria-hidden="true">↑</span><span><b id="wind-readout">Still air</b><small>Weather · ↑ downfield</small></span><span aria-hidden="true">⚙</span></button><div id="field-tools" hidden><div class="flight-stats"><div class="distance-stat"><span>DISTANCE</span><strong id="distance">0.0 <small>m</small></strong></div><div><span>AIR TIME</span><strong id="time">0.0 <small>s</small></strong></div><div><span>HEIGHT</span><strong id="height">1.8 <small>m</small></strong></div><div class="best-stat"><span>★ BEST</span><strong id="best">0.0 <small>m</small></strong></div></div><div class="flight-actions"><button id="edit" class="secondary">✂ Re-fold</button><button id="pause" class="secondary" disabled>Ⅱ Pause</button><button id="reset" class="secondary">↺ Retry</button></div></div>
<div id="result" hidden aria-live="polite"><div id="result-stars" class="result-stars">★ ★ ★</div><p id="result-label">TOUCHDOWN!</p><h2><span id="result-distance">0</span><small>metres</small></h2><p id="result-detail"></p><button id="review-flight" class="primary">EXPLORE FLIGHT ◎</button><button id="again" class="secondary">GO AGAIN! ↻</button><button id="refold" class="secondary">✂ Tweak my plane</button></div>
<section id="review-controls" hidden aria-label="Flight replay controls"><div class="review-heading"><span>✦ FLIGHT EXPLORER</span><button id="review-summary" class="small-button">Score card ↗</button></div><div class="review-timeline"><button id="review-play" class="secondary">▶ Replay</button><label for="review-time" class="sr-only">Flight replay time</label><input id="review-time" type="range" min="0" max="1" step="0.001" value="1"><output id="review-clock">0.0 / 0.0 s</output></div><div class="review-camera"><button id="review-fit" class="small-button">◎ Full path</button><button id="review-start" class="small-button">↗ Takeoff</button><button id="review-end" class="small-button">⚑ Landing</button><button id="review-plane" class="small-button">➤ Find plane</button></div><div class="review-flex"><button id="review-flex" class="small-button" aria-pressed="false">↟ Flex arrows</button><label>Arrows <select id="flex-scale" aria-label="Flex arrow exaggeration"><option value="5">5×</option><option value="10" selected>10×</option><option value="25">25×</option><option value="100">100×</option><option value="250">250×</option></select></label><button id="review-follow" class="small-button" aria-pressed="false">Follow plane</button><button id="review-layers" class="small-button" aria-pressed="false">Paper layers</button><button id="review-mass" class="small-button" aria-pressed="false">⊕ Centre of mass</button><button id="review-lift" class="small-button" aria-pressed="false" title="Estimated from the signed lift loads at this replay time">⊕ Centre of lift</button></div><p id="flex-readout" hidden></p><p id="centres-readout" hidden></p><p>Drag to orbit · right-drag to pan · scroll to zoom<br>WASD to fly · Q/E down/up · touch: drag or pinch with two fingers</p></section><div class="stage-bottom"><span id="stage-hint">Drag to orbit · scroll to zoom · tap for the next fold</span><span id="stage-tag" hidden></span></div><div id="celebration" aria-hidden="true"></div>
</section></main>
<dialog id="notes" aria-labelledby="help-title"><button id="close" class="close" aria-label="Close help">×</button><h2 id="help-title">Ready, pilot?</h2><p><strong>✂ Fold</strong><br>Drag to look around. Next fold and Back move one step; Fold plane finishes it. Fly takes you straight to the launcher.</p><p><strong>➤ Throw</strong><br>Pull the plane back, aim, then release. The pitch dial sets the nose angle.</p><p><strong>◎ Explore</strong><br>After landing, scrub the replay and drag to orbit. Try the flex and balance markers.</p><a href="./folding-study.html" target="_blank" rel="noreferrer">Open the folding study ↗</a></dialog>
<dialog id="weather-options" class="weather-dialog" aria-labelledby="weather-title"><button id="weather-close" class="close" aria-label="Close weather options">×</button><h2 id="weather-title">Weather machine</h2><p>Give your plane something to play with.</p><div class="weather-strength" aria-label="Breeze strength"><button data-breeze="0" aria-pressed="true">Still air</button><button data-breeze="1.5" aria-pressed="false">Breeze</button><button data-breeze="3" aria-pressed="false">Blustery</button></div><h3>Air blows toward</h3><div class="wind-rose"><div class="wind-compass" aria-hidden="true"><span id="weather-arrow">↑</span></div><div class="wind-directions"><button data-wind-direction="0" aria-pressed="true">↑ Downfield</button><button data-wind-direction="90" aria-pressed="false">→ Right</button><button data-wind-direction="180" aria-pressed="false">↓ Upfield</button><button data-wind-direction="270" aria-pressed="false">← Left</button></div></div><label class="weather-toggle"><input id="weather-gusts" type="checkbox"><span><b>Passing gusts</b><small>Bursts that build and fade</small></span></label><label class="weather-toggle"><input id="weather-turbulence" type="checkbox"><span><b>Turbulent air</b><small>Swirls and little updrafts</small></span></label><button id="weather-done" class="primary">Let’s try it ➤</button></dialog>`;
const incoming=new URLSearchParams(location.search);let design=['basic','dart','stable'].includes(incoming.get('design')??'')?incoming.get('design')!:'legacy';
let paperChoice=80,breeze=0,windHeading=0;
let parameters:Parameters={...DEFAULTS,dihedral:incoming.get('dihedral')==='0'?0:DEFAULTS.dihedral};
function createSimulation(){return new Simulation(parameters,design==='legacy'?undefined:designMesh(design,parameters));}
function foldLimit(){return design==='legacy'?3:flightRecipe(design,parameters).operations.length;}
let simulation=createSimulation();let mode:'workbench'|'throw'='workbench',step=0,paused=false,best=0;
try{best=Number(localStorage.getItem('paper-flight-best'))||0;}catch{/* Storage is optional. */}
let viewport:Viewport;
try{viewport=new Viewport($('viewport'));viewport.setSimulations([simulation]);}catch(error){document.querySelector('.stage')!.innerHTML='<div class="webgl-error"><h2>The field needs WebGL.</h2><p>Enable browser hardware acceleration, then reload to fold and fly.</p></div>';throw error;}
const clock=new FixedClock();let previous=performance.now(),lastUI=0,completed=false;
let recording=new FlightRecording(),reviewTime=0,replayPlaying=false;const reviewKeys=new Set<string>();
function rebuild(){recording=new FlightRecording();replayPlaying=false;reviewKeys.clear();$('review-controls').hidden=true;document.querySelector('main')!.classList.remove('reviewing');$('review-flight').hidden=true;simulation=createSimulation();viewport.setSimulations([simulation]);clock.reset();completed=false;paused=false;}
const foldAnimator=new FoldAnimator(),foldingView=new FoldingView($('sheet'));let foldSequence:FoldSequence|undefined,previewKey='';
function preview(){
 const recipe=flightRecipe(design==='legacy'?'dart':design,parameters),key=JSON.stringify(recipe);
 if(key!==previewKey){previewKey=key;foldSequence=new FoldSequence(recipe);foldingView.setSequence(foldSequence);}
 return foldSequence!;
}
// The original quick-start groups the dart demonstration into corners, nose and wings.
function foldPosition(index:number){return design==='legacy'?[0,3,5,8][index]:index;}
function drawFoldPosition(progress:number,active?:number){preview();foldingView.draw(progress,parameters.size,parameters.paperWeight,active,parameters.material);}
function cancelFolding(){foldAnimator.cancel();$('sheet').setAttribute('aria-busy','false');$<HTMLButtonElement>('fold').disabled=false;}
const foldTitles=['Fold the corners!','Sharpen that nose!','Wings out!','Cleared for takeoff!'];
const foldDescriptions=['Bring the top corners to the centre crease.','Fold the shoulders inward to make the nose.','Fold down both wings and open them into a gentle V.','Orbit to inspect your plane, or head out to the field.'];
function drawSheet(){
 cancelFolding();const limit=foldLimit(),recipe=preview().recipe;
 drawFoldPosition(foldPosition(step));
 $('step-count').textContent=step===limit?'':`FOLD ${step+1} / ${limit}`;
 $('fold-title').textContent=design==='legacy'?foldTitles[step]:step===limit?'Cleared for takeoff!':recipe.operations[step].label;
 $('fold-description').textContent=design==='legacy'?foldDescriptions[step]:step===limit?'Your folded sheet is ready. Try it in the field.':'Tap to make the next fold.';
 $('fold').hidden=step===limit;$('finish-fold').hidden=step===limit;$('fold').textContent='Next fold →';$('go-throw').hidden=step<limit;
 $<HTMLButtonElement>('undo-fold').disabled=step===0;
 $('fold-checks').innerHTML=design==='legacy'?['Corners','Nose','Wings'].map((label,i)=>`<div class="${step>i?'done':''}"><b>${step>i?'✓':i+1}</b>${label}</div>`).join(''):`<div class="done"><b>✓</b>Real sheet</div><div class="${step?'done':''}"><b>${step}</b>Folds</div><div class="${step===limit?'done':''}"><b>${step===limit?'✓':'→'}</b>Flight</div>`;
 $('wing-label').textContent=parameters.span<.9?'Narrow wings':parameters.span>1.1?'Broad wings':'Classic wings';updatePicker();
}
function animateFold(target:number){
 if(foldAnimator.running||target<0||target>foldLimit())return;
 const from=foldPosition(step),to=foldPosition(target);$('sheet').setAttribute('aria-busy','true');
 $<HTMLButtonElement>('fold').disabled=true;$<HTMLButtonElement>('undo-fold').disabled=true;$('go-throw').hidden=true;
 $('fold-description').textContent=target>step?'Follow the orange crease…':'Opening the fold…';
 foldAnimator.play(from,to,progress=>{
  const active=Math.min(preview().recipe.operations.length-1,Math.max(0,Math.floor(progress===to&&to>from?progress-1:progress)));
  drawFoldPosition(progress,active);
 },()=>{step=target;drawSheet();});
}
function fold(){if(step<foldLimit())animateFold(step+1);}
$('fold').onclick=fold;$('undo-fold').onclick=()=>animateFold(step-1);
$('sheet').onclick=fold;
$('finish-fold').onclick=()=>{cancelFolding();step=foldLimit();drawSheet();};
if(design!=='legacy')step=foldLimit();
const designs=[{id:'legacy',name:'Original dart',detail:'Quick three-step starter'},{id:'basic',name:'The Basic',detail:'Broad wings · 6 folds'},{id:'dart',name:'Folded Dart',detail:'Pointed nose · 8 folds'},{id:'stable',name:'The Stable',detail:'Locking nose · 10 folds'}];
function updatePicker(){
 const selected=designs.find(item=>item.id===design)!;
 $('plane-design').innerHTML=`${planeThumbnail(design==='legacy'?new Simulation(parameters).mesh:designMesh(design,parameters))}<span>${selected.name}</span><span aria-hidden="true">▾</span>`;
 $('plane-design').setAttribute('aria-label',`Choose plane: ${selected.name}`);
}
function closePicker(){ $('plane-options').hidden=true;$('plane-design').setAttribute('aria-expanded','false'); }
function openPicker(){
 $('plane-options').innerHTML=designs.map(item=>`<button role="option" aria-selected="${item.id===design}" data-design="${item.id}" tabindex="-1">${planeThumbnail(item.id==='legacy'?new Simulation(parameters).mesh:designMesh(item.id,parameters))}<span><b>${item.name}</b><small>${item.detail}</small></span><span class="plane-selected" aria-hidden="true">${item.id===design?'✓':''}</span></button>`).join('');
 $('plane-options').hidden=false;$('plane-design').setAttribute('aria-expanded','true');
 $('plane-options').querySelector<HTMLElement>('[aria-selected="true"]')?.focus();
}
$('plane-design').onclick=()=>{$('plane-options').hidden?openPicker():closePicker();};
$('plane-design').onkeydown=e=>{if(['ArrowDown','ArrowUp'].includes(e.key)){e.preventDefault();openPicker();}};
$('plane-options').onclick=e=>{const option=(e.target as Element).closest<HTMLButtonElement>('[data-design]');if(!option)return;cancelFolding();design=option.dataset.design!;step=0;rebuild();drawSheet();foldingView.home();updatePicker();closePicker();$('plane-design').focus();};
$('plane-options').onkeydown=e=>{const options=Array.from($('plane-options').querySelectorAll<HTMLButtonElement>('[data-design]')),index=options.indexOf(document.activeElement as HTMLButtonElement);if(e.key==='Escape'){e.preventDefault();closePicker();$('plane-design').focus();}else if(['ArrowDown','ArrowUp','Home','End'].includes(e.key)){e.preventDefault();const next=e.key==='Home'?0:e.key==='End'?options.length-1:(index+(e.key==='ArrowDown'?1:-1)+options.length)%options.length;options[next].focus();}else if(e.key==='Tab')closePicker();};
document.addEventListener('pointerdown',e=>{if(!$('design-picker').contains(e.target as Node))closePicker();});
updatePicker();
function changeSpan(delta:number){parameters.span=Math.max(0.7,Math.min(1.35,parameters.span+delta));drawSheet();rebuild();}
$('narrow').onclick=()=>changeSpan(-0.1);$('wide').onclick=()=>changeSpan(0.1);
function choice(group:string,attribute:string,apply:(value:number)=>void){$(group).querySelectorAll<HTMLButtonElement>('button').forEach(button=>{button.setAttribute('aria-pressed',String(button.classList.contains('selected')));button.onclick=()=>{apply(Number(button.dataset[attribute]));$(group).querySelectorAll('button').forEach(b=>{b.classList.toggle('selected',b===button);b.setAttribute('aria-pressed',String(b===button));});rebuild();drawSheet();};});}
function updateMaterial(){
 $<HTMLSelectElement>('special-material').value=parameters.material;$('special-tile').classList.toggle('selected',parameters.material!=='paper');$('special-tile').title=materials[parameters.material].note;
 $('paper-choices').querySelectorAll<HTMLButtonElement>('button').forEach(button=>{const selected=parameters.material==='paper'&&Number(button.dataset.weight)===paperChoice;button.classList.toggle('selected',selected);button.setAttribute('aria-pressed',String(selected));});
 document.querySelector('.desk-sticker')!.innerHTML=`100%<br>${parameters.material==='paper'?'PAPER':parameters.material.toUpperCase()}<br><span>${parameters.material==='steel'?'0% sensible':'0% boring'}</span>`;
}
choice('paper-choices','weight',weight=>{paperChoice=weight;Object.assign(parameters,materialSettings('paper',weight));updateMaterial();});
$('special-material').onchange=()=>{Object.assign(parameters,materialSettings($<HTMLSelectElement>('special-material').value as MaterialId,paperChoice));updateMaterial();rebuild();drawSheet();};

choice('size-choices','size',size=>parameters.size=size);choice('fold-choices','dihedral',angle=>parameters.dihedral=angle);
$('fold-choices').querySelectorAll<HTMLButtonElement>('button').forEach(button=>{const selected=Number(button.dataset.dihedral)===parameters.dihedral;button.classList.toggle('selected',selected);button.setAttribute('aria-pressed',String(selected));});
function switchMode(next:typeof mode){cancelFolding();if(next==='throw')step=foldLimit();drawSheet();closePicker();$('celebration').replaceChildren();mode=next;document.querySelector('main')!.dataset.mode=mode;rebuild();viewport.mode=mode;viewport.controls.enabled=mode==='workbench';
 $('sheet-wrap').hidden=mode==='throw';$('fold-prompt').hidden=mode==='throw';$('launch-overlay').hidden=mode==='workbench';$('workbench-tools').hidden=mode==='throw';$('field-tools').hidden=mode==='workbench';$('weather-button').hidden=mode==='workbench';$('result').hidden=true;
 $('bench-tab').classList.toggle('selected',mode==='workbench');$('throw-tab').classList.toggle('selected',mode==='throw');$('bench-tab').setAttribute('aria-pressed',String(mode==='workbench'));$('throw-tab').setAttribute('aria-pressed',String(mode==='throw'));
 $('location').textContent=mode==='workbench'?'THE FOLD ZONE':'CLOVER FIELD';$('camera').textContent=mode==='workbench'?'Reset view ↺':'Camera: chase';viewport.options.follow=true;
 $('stage-hint').textContent=mode==='workbench'?'Drag to orbit · scroll to zoom · tap for the next fold':'Drag plane to launch · W/S adjusts pitch · R retries';$('stage-tag').textContent=mode==='workbench'?'NO GLUE. JUST POSSIBILITY.':'LANDMARKS EVERY 5 METRES';resetSling();updateUI();
}
$('bench-tab').onclick=$('edit').onclick=$('refold').onclick=()=>switchMode('workbench');$('throw-tab').onclick=$('go-throw').onclick=()=>switchMode('throw');
$('unfold').onclick=()=>{cancelFolding();step=0;drawSheet();foldingView.home();rebuild();};
$('camera').onclick=()=>{if(viewport.reviewing){showFullPath();return;}if(mode==='workbench')foldingView.home();else{viewport.options.follow=!viewport.options.follow;$('camera').textContent=viewport.options.follow?'Camera: chase':'Camera: launch point';}};
let pullX=0,pullY=0,dragging=false,startX=0,startY=0,aimPitch=15,releasing=false;let slingPointer:number|null=null;
function pullAmount(){return Math.hypot(pullX,pullY);}
function releaseSpeed(){return springSpeed(pullAmount(),simulation.mesh.masses.reduce((a,b)=>a+b,0));}
function setPitch(value:number){if(simulation.status!=='ready'||releasing)return;aimPitch=((value+180)%360+360)%360-180;paintSling();}
$('pitch-down').onclick=()=>setPitch(aimPitch-2);$('pitch-up').onclick=()=>setPitch(aimPitch+2);
let pitchPointer:number|null=null,pitchStartY=0,pitchStart=15;
$('pitch-dial').onpointerdown=e=>{pitchPointer=e.pointerId;pitchStartY=e.clientY;pitchStart=aimPitch;$('pitch-dial').setPointerCapture(e.pointerId);};
$('pitch-dial').onpointermove=e=>{if(pitchPointer===e.pointerId)setPitch(pitchStart+pitchStartY-e.clientY);};
$('pitch-dial').onpointerup=$('pitch-dial').onpointercancel=()=>{pitchPointer=null;};
$('pitch-dial').onkeydown=e=>{const steps:Record<string,number>={ArrowUp:2,ArrowRight:2,ArrowDown:-2,ArrowLeft:-2,PageUp:15,PageDown:-15};if(e.key in steps){e.preventDefault();setPitch(aimPitch+steps[e.key]*(e.shiftKey?5:1));}else if(e.key==='Home'||e.key==='End'){e.preventDefault();setPitch(e.key==='Home'?0:180);}};
function paintSling(){
 if(mode!=='throw'||simulation.status!=='ready')return;
 simulation.parameters.heading=-pullX*0.7;simulation.parameters.pitch=aimPitch;simulation.reset();
 const anchor=viewport.launcherPoint();viewport.previewPull(pullX,pullY);
 const sling=$('sling');sling.style.left=`${anchor.x+pullX}px`;sling.style.top=`${anchor.y+pullY}px`;
 for(const [id,offset,post] of [['band-left',-85,'anchor-left'],['band-right',85,'anchor-right']] as const){const line=$(id);line.setAttribute('x1',String(anchor.x+offset));line.setAttribute('y1',String(anchor.y+25));line.setAttribute('x2',String(anchor.x+pullX));line.setAttribute('y2',String(anchor.y+pullY));$(post).style.left=`${anchor.x+offset}px`;$(post).style.top=`${anchor.y+25}px`;}
 $('launch-speed').innerHTML=`${releaseSpeed().toFixed(1)} <small>m/s</small>`;$('pitch-value').textContent=`${Math.round(aimPitch)}°`;$('pitch-dial').setAttribute('aria-valuenow',String(Math.round(aimPitch)));$('pitch-dial').setAttribute('aria-valuetext',`${Math.round(aimPitch)} degrees pitch`);$('pitch-needle').style.transform=`rotate(${-aimPitch}deg)`;
 const tension=pullAmount();$('power-fill').style.width=`${tension}%`;$('power-label').textContent=tension>0?`${Math.round(tension)}% POWER · AIM ${Math.round(-pullX*0.7)}°`:'PULL THE PLANE BACK';
 for(const id of ['band-left','band-right']){$(id).style.strokeWidth=String(8-2.5*tension/100);$(id).style.stroke=`hsl(${28-12*tension/100} 65% ${55-12*tension/100}%)`;}
}
function resetSling(){slingPointer=null;dragging=false;releasing=false;pullX=0;pullY=0;paintSling();$('launch-controls').hidden=false;$('sling').hidden=false;$('bands').removeAttribute('hidden');$('anchor-left').hidden=false;$('anchor-right').hidden=false;$('flight-state').textContent='READY, PILOT?';$('launch-help').textContent='Grab the plane. Pull back. Let it rip.';}
function launch(){if(mode!=='throw'||simulation.status!=='ready'||pullAmount()<12)return;
 parameters.speed=releaseSpeed();parameters.heading=-pullX*0.7;parameters.elevation=4+pullY*0.12;parameters.pitch=aimPitch;
 rebuild();simulation.launch();recording.capture(simulation);$('launch-controls').hidden=true;$('sling').hidden=true;$('bands').setAttribute('hidden','');$('anchor-left').hidden=true;$('anchor-right').hidden=true;$('flight-state').textContent='WHEEEE!';$('launch-help').textContent='Next stop: a new record?';
}
function release(){if(releasing)return;releasing=true;const x=pullX,y=pullY,start=performance.now(),duration=springReleaseDuration(simulation.mesh.masses.reduce((a,b)=>a+b,0));function snap(now:number){if(!releasing)return;const t=Math.min(1,(now-start)/duration),remaining=springRemaining(t);pullX=x*remaining;pullY=y*remaining;paintSling();if(t<1)requestAnimationFrame(snap);else{pullX=x;pullY=y;releasing=false;launch();}}requestAnimationFrame(snap);}
$('sling').onpointerdown=e=>{if(simulation.status!=='ready'||releasing||dragging)return;dragging=true;slingPointer=e.pointerId;const travel=pullTravel(pullX,pullY);startX=e.clientX-travel.x;startY=e.clientY-travel.y;$('sling').setPointerCapture(e.pointerId);e.preventDefault();};
$('sling').onpointermove=e=>{if(!dragging||e.pointerId!==slingPointer)return;const pull=elasticPull(e.clientX-startX,e.clientY-startY);pullX=pull.x;pullY=pull.y;paintSling();};
$('sling').onpointerup=e=>{if(!dragging||e.pointerId!==slingPointer)return;slingPointer=null;dragging=false;if(pullAmount()>=12)release();else resetSling();};$('sling').onpointercancel=e=>{if(e.pointerId===slingPointer)resetSling();};
$('sling').onkeydown=e=>{if(simulation.status!=='ready'||releasing)return;if(['ArrowLeft','ArrowRight','ArrowUp','ArrowDown',' ','Enter'].includes(e.key))e.preventDefault();const travel=pullTravel(pullX,pullY);if(e.key==='ArrowLeft')travel.x+=15;if(e.key==='ArrowRight')travel.x-=15;if(e.key==='ArrowDown')travel.y+=15;if(e.key==='ArrowUp')travel.y=Math.max(0,travel.y-15);const pull=elasticPull(travel.x,travel.y);pullX=pull.x;pullY=pull.y;paintSling();if(e.key===' '||e.key==='Enter'){if(pullAmount()<12){pullX=0;pullY=65;}release();}};
function updateWeather(){
 const angle=windHeading*Math.PI/180;parameters.wind=[breeze*Math.cos(angle),0,breeze*Math.sin(angle)];parameters.windHeading=windHeading;
 parameters.gusts=$<HTMLInputElement>('weather-gusts').checked?1.5:0;parameters.turbulence=$<HTMLInputElement>('weather-turbulence').checked?1:0;
 Object.assign(simulation.parameters,{wind:[...parameters.wind],windHeading,gusts:parameters.gusts,turbulence:parameters.turbulence});
 document.querySelectorAll<HTMLButtonElement>('[data-breeze]').forEach(b=>b.setAttribute('aria-pressed',String(Number(b.dataset.breeze)===breeze)));
 document.querySelectorAll<HTMLButtonElement>('[data-wind-direction]').forEach(b=>b.setAttribute('aria-pressed',String(Number(b.dataset.windDirection)===windHeading)));
 $('weather-arrow').style.transform=`rotate(${windHeading}deg)`;updateWindIndicator();
}
function updateWindIndicator(){
 const frame=viewport.reviewing?viewport.reviewFrame:undefined,wind=airVelocity(simulation.parameters,frame?.time??simulation.time,frame?.center??simulation.center()),speed=Math.hypot(...wind);
 const angle=Math.atan2(wind[2],wind[0])*180/Math.PI;$('wind-arrow').style.transform=`rotate(${angle}deg)`;$('wind-arrow').style.opacity=speed<.05?'.25':'1';
 $('wind-readout').textContent=speed<.05&&!parameters.gusts&&!parameters.turbulence?'Still air':`${parameters.turbulence?'Swirly':parameters.gusts?'Gusty':'Breeze'} · ${speed.toFixed(1)} m/s`;
 $<HTMLButtonElement>('weather-button').disabled=simulation.status!=='ready'||releasing;
}
$('weather-button').onclick=()=>{if(simulation.status==='ready'&&!releasing)$<HTMLDialogElement>('weather-options').showModal();};
$('weather-close').onclick=$('weather-done').onclick=()=> $<HTMLDialogElement>('weather-options').close();
 document.querySelectorAll<HTMLButtonElement>('[data-breeze]').forEach(button=>button.onclick=()=>{breeze=Number(button.dataset.breeze);updateWeather();});
 document.querySelectorAll<HTMLButtonElement>('[data-wind-direction]').forEach(button=>button.onclick=()=>{windHeading=Number(button.dataset.windDirection);updateWeather();});
$('weather-gusts').onchange=$('weather-turbulence').onchange=updateWeather;
function updateReview(){const frame=recording.sample(reviewTime);if(!frame)return;viewport.setReviewFrame(frame);$<HTMLInputElement>('review-time').value=String(reviewTime);$('review-clock').textContent=`${reviewTime.toFixed(2)} / ${recording.duration.toFixed(2)} s`;$('review-play').textContent=replayPlaying?'Ⅱ Pause replay':'▶ Replay';}
function enterReview(){updateOverlayControls();if(recording.frames.length<2)return;viewport.beginReview();reviewTime=recording.duration;replayPlaying=false;$('result').hidden=true;$('celebration').replaceChildren();$('review-controls').hidden=false;document.querySelector('main')!.classList.add('reviewing');$('camera').textContent='◎ Fit flight';$('stage-hint').textContent='Explore your recorded flight';$<HTMLInputElement>('review-time').max=String(recording.duration);updateReview();$('review-play').focus();}
$('review-flight').onclick=enterReview;
$('review-play').onclick=()=>{if(reviewTime>=recording.duration)reviewTime=0;replayPlaying=!replayPlaying;updateReview();};
$('review-time').oninput=()=>{reviewTime=Number($<HTMLInputElement>('review-time').value);replayPlaying=false;updateReview();};
function showFullPath(){viewport.options.flex=false;viewport.options.layers=false;viewport.options.massCenter=false;viewport.options.liftCenter=false;viewport.reviewFollow=false;updateOverlayControls();viewport.fitReview();}
// Seek before framing: follow mode must use the same time as the camera target.
function seekReview(time:number){replayPlaying=false;reviewTime=time;updateReview();viewport.focusReview('plane',.85);updateOverlayControls();}
$('review-fit').onclick=showFullPath;$('review-start').onclick=()=>seekReview(0);$('review-end').onclick=()=>seekReview(recording.duration);$('review-plane').onclick=()=>viewport.focusReview('plane',.65);
function updateOverlayControls(){$('review-flex').setAttribute('aria-pressed',String(viewport.options.flex));$('review-follow').setAttribute('aria-pressed',String(viewport.reviewFollow));$('review-layers').setAttribute('aria-pressed',String(viewport.options.layers));$('flex-readout').hidden=!(viewport.options.flex||viewport.options.layers);$('review-mass').setAttribute('aria-pressed',String(viewport.options.massCenter));$('review-lift').setAttribute('aria-pressed',String(viewport.options.liftCenter));$('centres-readout').hidden=!(viewport.options.massCenter||viewport.options.liftCenter);}
$('review-flex').onclick=()=>{viewport.options.flex=!viewport.options.flex;if(viewport.options.flex){viewport.reviewFollow=true;viewport.focusReview('plane',.65);}updateOverlayControls();};
$('flex-scale').onchange=()=>viewport.options.flexScale=Number($<HTMLSelectElement>('flex-scale').value);
$('review-follow').onclick=()=>{viewport.reviewFollow=!viewport.reviewFollow;if(viewport.reviewFollow)viewport.focusReview('plane',.65);updateOverlayControls();};
$('review-layers').onclick=()=>{viewport.options.layers=!viewport.options.layers;if(viewport.options.layers){viewport.reviewFollow=true;viewport.focusReview('plane',.65);}updateOverlayControls();};
for(const [id,option] of [['review-mass','massCenter'],['review-lift','liftCenter']] as const){$(id).onclick=()=>{viewport.options[option]=!viewport.options[option];if(viewport.options[option]){viewport.reviewFollow=true;viewport.focusReview('plane',.65);}updateOverlayControls();};}
$('review-summary').onclick=()=>{replayPlaying=false;updateReview();$('result').hidden=false;};
window.addEventListener('keyup',e=>reviewKeys.delete(e.key.toLowerCase()));window.addEventListener('blur',()=>reviewKeys.clear());
function reset(){$('celebration').replaceChildren();rebuild();$('camera').textContent=viewport.options.follow?'Camera: chase':'Camera: launch point';$('stage-hint').textContent='Drag plane to launch · W/S adjusts pitch · R retries';$('result').hidden=true;resetSling();updateUI();}
$('again').onclick=$('reset').onclick=reset;
$('pause').onclick=()=>{paused=!paused;clock.reset();$('pause').textContent=paused?'Resume flight':'Pause flight';};
window.addEventListener('keydown',e=>{if(document.querySelector('dialog[open]'))return;if(viewport.reviewing&&$('result').hidden){const key=e.key.toLowerCase();if(['w','a','s','d','q','e'].includes(key)&&!(e.target as HTMLElement).matches('input')){e.preventDefault();reviewKeys.add(key);return;}}if(mode==='throw'&&['w','s'].includes(e.key.toLowerCase())){setPitch(aimPitch+(e.key.toLowerCase()==='w'?2:-2));return;}if((e.target as HTMLElement).matches('button,input'))return;if(mode==='throw'){if(e.key.toLowerCase()==='r')reset();}});
let pausedBeforeNotes=false;
$('about').onclick=()=>{$<HTMLDialogElement>('notes').showModal();pausedBeforeNotes=paused;paused=true;};$('close').onclick=()=> $<HTMLDialogElement>('notes').close();$('notes').addEventListener('close',()=>{paused=pausedBeforeNotes;clock.reset();});
function celebrate(){if(matchMedia('(prefers-reduced-motion: reduce)').matches)return;for(let i=0;i<28;i++){const piece=document.createElement('i');piece.className='confetti';piece.style.background=['#ffda55','#ff8663','#91c7ba','#fff4d8'][i%4];$('celebration').append(piece);const x=Math.cos(i*2.4)*(140+i*7),y=-180-(i%5)*40;piece.animate([{transform:'translate(0,0) rotate(0deg)',opacity:1},{transform:`translate(${x}px,${y}px) rotate(${i*47}deg)`,opacity:1,offset:.45},{transform:`translate(${x*1.3}px,300px) rotate(${i*90}deg)`,opacity:0}],{duration:1100+i%5*100,easing:'ease-out'}).onfinish=()=>piece.remove();}}
function updateUI(){updateWindIndicator();const m=simulation.metrics();if(viewport.reviewing){$('centres-readout').textContent=[viewport.options.massCenter?'Blue = centre of mass, including folded layers.':'',viewport.options.liftCenter?(viewport.views[0].liftCenter?'Pink = estimated centre of lift at this moment.':'Centre of lift hidden: net lift is too small or cancels out.'):''].filter(Boolean).join(' ');const rms=viewport.views[0].flexRms*1000;$('flex-readout').textContent=(viewport.options.flex?`Flex RMS ${rms.toFixed(2)} mm · displacement arrows ${viewport.options.flexScale}× · blue = undeformed reference. `:'')+(viewport.options.layers?'Layers: teal = 1 · orange = 8+ (local mass & stiffness).':'');}if(simulation.status==='ready'){m.distance=0;m.altitude=parameters.height;}if(viewport.reviewing&&viewport.reviewFrame){const frame=viewport.reviewFrame;m.time=frame.time;m.altitude=frame.center[1];m.distance=Math.hypot(frame.center[0]-simulation.start[0],frame.center[2]-simulation.start[2]);}$('distance').innerHTML=`${m.distance.toFixed(1)} <small>m</small>`;$('height').innerHTML=`${Math.max(0,m.altitude).toFixed(1)} <small>m</small>`;$('time').innerHTML=`${m.time.toFixed(1)} <small>s</small>`;$('best').innerHTML=`${best.toFixed(1)} <small>m</small>`;$<HTMLButtonElement>('pause').disabled=simulation.status!=='flying';$('pause').textContent=paused?'Resume flight':'Pause flight';
 if(!completed&&!['ready','flying'].includes(simulation.status)){completed=true;$('review-flight').hidden=recording.frames.length<2||simulation.status==='error';if(simulation.status!=='error')celebrate();const record=m.distance>best;if(simulation.status!=='error'){best=Math.max(best,m.distance);try{localStorage.setItem('paper-flight-best',String(best));}catch{}}
 $('result-stars').textContent=simulation.status==='error'?'✕':'★'.repeat(Math.min(3,Math.floor(m.distance/5)))+'☆'.repeat(3-Math.min(3,Math.floor(m.distance/5)));$('result').hidden=false;$('result-label').textContent=simulation.status==='error'?'FLIGHT INTERRUPTED':simulation.status==='finished'?'FLIGHT TIME LIMIT':record?'NEW RECORD!':'TOUCHDOWN!';$('result-distance').textContent=m.distance.toFixed(1);$('result-detail').textContent=simulation.status==='error'?'The solver could not finish this flight. Try another fold.':`${m.time.toFixed(1)}s airtime · ${m.distance>=15?'Ace pilot!':m.distance>=10?'Nice flight!':m.distance>=5?'Getting the hang of it!':'Try more power or a different pitch.'}`;$('flight-state').textContent='NICE LANDING!';$('launch-help').textContent='';
 }
}
function animate(now:number){const elapsed=(now-previous)/1000;previous=now;if(!paused&&simulation.status==='flying')clock.advance(elapsed,()=>{simulation.step();recording.capture(simulation);});if(viewport.reviewing&&!$<HTMLDialogElement>('notes').open&&$('result').hidden){if(replayPlaying){reviewTime=Math.min(recording.duration,reviewTime+Math.min(elapsed,0.1));if(reviewTime>=recording.duration)replayPlaying=false;updateReview();}viewport.moveReview(Number(reviewKeys.has('w'))-Number(reviewKeys.has('s')),Number(reviewKeys.has('d'))-Number(reviewKeys.has('a')),Number(reviewKeys.has('e'))-Number(reviewKeys.has('q')),elapsed);}if(now-lastUI>100){updateUI();lastUI=now;}if(mode==='throw'&&simulation.status==='ready')paintSling();viewport.render();if(mode==='workbench')foldingView.render();requestAnimationFrame(animate);}
new ResizeObserver(()=>paintSling()).observe($('launch-overlay'));
document.addEventListener('visibilitychange',()=>{reviewKeys.clear();previous=performance.now();clock.reset();});drawSheet();updateUI();requestAnimationFrame(animate);
