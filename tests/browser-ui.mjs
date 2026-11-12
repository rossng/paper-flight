// Dependency-free UI regression checks. Run Vite and isolated Chrome with remote
// debugging enabled, then: node tests/browser-ui.mjs --cdp 9297 --url http://127.0.0.1:5198/
import assert from 'node:assert/strict';
const option=(name,fallback)=>process.argv.includes(name)?process.argv[process.argv.indexOf(name)+1]:fallback;
const port=option('--cdp','9297'),url=option('--url','http://127.0.0.1:5197/');
const target=await(await fetch(`http://127.0.0.1:${port}/json/new?about:blank`,{method:'PUT'})).json();
const ws=new WebSocket(target.webSocketDebuggerUrl);
await new Promise(resolve=>ws.addEventListener('open',resolve));
let id=0;const pending=new Map(),errors=[];
ws.addEventListener('message',event=>{
 const message=JSON.parse(event.data);
 if(message.id){const p=pending.get(message.id);pending.delete(message.id);message.error?p.reject(message.error):p.resolve(message.result);}
 if(message.method==='Runtime.exceptionThrown')errors.push(message.params.exceptionDetails);
});
const send=(method,params={})=>new Promise((resolve,reject)=>{const n=++id;pending.set(n,{resolve,reject});ws.send(JSON.stringify({id:n,method,params}));});
const evaluate=async expression=>{const result=await send('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});if(result.exceptionDetails)throw Error(JSON.stringify(result.exceptionDetails));return result.result.value;};
const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const wait=async expression=>{const start=Date.now();while(!await evaluate(expression)){if(Date.now()-start>120000)throw Error(`Timeout: ${expression}`);await sleep(100);}};
const resize=async(width,height)=>{await send('Emulation.setDeviceMetricsOverride',{width,height,deviceScaleFactor:1,mobile:false});await evaluate('new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve)))');};
const layoutCheck=`(()=>{
 const rect=id=>document.getElementById(id).getBoundingClientRect();
 const title=rect('fold-title'),description=rect('fold-description'),prompt=rect('fold-prompt'),sheet=rect('sheet'),tools=rect('workbench-tools');
 const range=document.createRange();range.selectNodeContents(document.getElementById('fold-title'));const text=range.getBoundingClientRect();
 const failures=[];
 const pencil=document.querySelector('.desk-pencil'),sticker=document.querySelector('.desk-sticker');
 if(getComputedStyle(pencil).display!=='none'){
  const a=pencil.getBoundingClientRect(),b=sticker.getBoundingClientRect();
  if(a.left<b.right&&a.right>b.left&&a.top<b.bottom&&a.bottom>b.top)failures.push('pencil overlaps sticker');
  if(a.bottom>sheet.bottom+1)failures.push('pencil spills below mat');
 }

 if(text.bottom>description.top+1)failures.push('title overlaps description');
 if(sheet.bottom>prompt.top+1)failures.push('sheet overlaps instructions');
 if(prompt.bottom>tools.top+1)failures.push('instructions overlap tools');
 if(document.documentElement.scrollWidth>innerWidth)failures.push('horizontal overflow');
 for(const id of ['fold-prompt','workbench-tools','plane-design']){const r=rect(id);if(r.left<0||r.right>innerWidth+1)failures.push(id+' outside viewport');}
 for(const button of document.querySelectorAll('#fold-prompt button:not([hidden])')){const r=button.getBoundingClientRect();if(r.bottom>prompt.bottom+1||r.right>prompt.right+1)failures.push('fold action outside instructions');}
 return failures;
})()`;
try{
 await send('Runtime.enable');await send('Page.enable');
 await send('Emulation.setEmulatedMedia',{features:[{name:'prefers-reduced-motion',value:'reduce'}]});
 await resize(1280,800);await send('Page.navigate',{url});
 await wait("!!document.querySelector('#plane-design svg')");await evaluate('document.fonts.ready');
 const picker=await evaluate(`(()=>{
  const options=document.querySelector('#plane-options'),nodes=[...options.children],times=[];
  for(let i=0;i<6;i++){const start=performance.now();document.querySelector('#plane-design').click();times.push(performance.now()-start);if(options.hidden)throw Error('Picker did not open');if(document.activeElement.getAttribute('aria-selected')!=='true')throw Error('Selected option not focused');document.querySelector('#plane-design').click();}
  if(nodes.some((node,i)=>node!==options.children[i]))throw Error('Picker rebuilt its options');
  return times;
 })()`);
 await evaluate(`document.querySelector('#plane-design').dispatchEvent(new KeyboardEvent('keydown',{key:'ArrowDown',bubbles:true}));document.activeElement.dispatchEvent(new KeyboardEvent('keydown',{key:'End',bubbles:true}));`);
 assert.equal(await evaluate('document.activeElement.dataset.design'),'raven');
 await evaluate(`document.activeElement.dispatchEvent(new KeyboardEvent('keydown',{key:'Escape',bubbles:true}))`);
 assert.equal(await evaluate('document.activeElement.id'),'plane-design');
 const designIds=await evaluate("[...document.querySelectorAll('[data-design]')].map(e=>e.dataset.design)");
 assert.deepEqual(designIds,['legacy','basic','dart','stable','heavy-nose','hunting','buzz','v-wing','underside','raven']);
 let layouts=0;
 for(const [width,height] of [[1600,900],[1280,800],[1024,768],[901,650],[900,700],[768,800],[601,800],[600,800],[390,844],[320,568],[844,390]]){
  await resize(width,height);
  for(const design of designIds){
   await evaluate("document.querySelector('#plane-design').click()");
   const target=await evaluate(`(()=>{const e=document.querySelector('[data-design="${design}"]');e.scrollIntoView({block:'nearest'});const r=e.getBoundingClientRect(),x=r.x+r.width/2,y=r.y+r.height/2;return {x,y,reachable:document.elementFromPoint(x,y)?.closest('[data-design]')===e};})()`);
   assert.equal(target.reachable,true,`${width}×${height}: ${design} is visible and clickable`);
   await send('Input.dispatchMouseEvent',{type:'mousePressed',x:target.x,y:target.y,button:'left',clickCount:1});
   await send('Input.dispatchMouseEvent',{type:'mouseReleased',x:target.x,y:target.y,button:'left',clickCount:1});
   assert.equal(await evaluate("document.querySelector('[data-design][aria-selected=true]').dataset.design"),design);
   for(let step=0;step<20;step++){
    assert.deepEqual(await evaluate(layoutCheck),[],`${width}×${height}, ${design}, fold ${step}`);layouts++;
    if(await evaluate("document.querySelector('#fold').hidden"))break;
    await evaluate("document.querySelector('#fold').click()");
   }
   await evaluate("document.querySelector('#undo-fold').click()");
   assert.deepEqual(await evaluate(layoutCheck),[]);
  }
  await evaluate(`document.querySelector('#fold-title').textContent='Bring both corners carefully toward the centre crease, then fold the long pointed nose underneath the wings';document.querySelector('#fold-description').textContent='Inspect the crease from both sides before continuing. '+ 'LongUnbrokenInstruction'.repeat(8)`);
  assert.deepEqual(await evaluate(layoutCheck),[],`long text at ${width}×${height}`);
  await evaluate("document.querySelector('#special-material').click()");
  assert.equal(await evaluate(`(()=>{const r=document.querySelector('#material-menu').getBoundingClientRect();return r.left>=0&&r.right<=innerWidth;})()`),true,'material menu fits');
  await evaluate("document.querySelector('#special-material').click()");
 }
 // Instrument the dev modules, without shipping debug hooks in the application.
 await resize(1280,800);
 await evaluate(`(async()=>{
  const loaded=path=>performance.getEntriesByType('resource').findLast(e=>new URL(e.name).pathname===path)?.name??path;
  const {Viewport}=await import(loaded('/src/rendering/viewport.ts'));
  const {FoldingView}=await import(loaded('/src/folding/view.ts'));
  window.flightRenders=0;
  const flight=Viewport.prototype.render;Viewport.prototype.render=function(){window.flightRenders++;return flight.call(this);};
  const paper=FoldingView.prototype.render;FoldingView.prototype.render=function(){window.paperView=this;return paper.call(this);};
 })()`);
 await sleep(500);
 const frame=await evaluate('window.paperView.renderer.info.render.frame');await sleep(250);
 assert.equal(await evaluate('window.paperView.renderer.info.render.frame'),frame,'idle workbench does not redraw');
 assert.equal(await evaluate('window.flightRenders'),0,'hidden flight scene does not render');
 await evaluate("document.querySelector('#camera').click()");await sleep(100);
 assert.ok(await evaluate('window.paperView.renderer.info.render.frame')>frame,'reset view redraws');
 const beforeZoom=await evaluate('window.paperView.renderer.info.render.frame');
 await evaluate("document.querySelector('#sheet canvas').dispatchEvent(new WheelEvent('wheel',{deltaY:100,bubbles:true,cancelable:true}))");await sleep(100);
 assert.ok(await evaluate('window.paperView.renderer.info.render.frame')>beforeZoom,'zoom redraws');
 await send('Emulation.setEmulatedMedia',{features:[{name:'prefers-reduced-motion',value:'no-preference'}]});
 await evaluate("document.querySelector('#unfold').click()");await sleep(100);
 const beforeFold=await evaluate('window.paperView.renderer.info.render.frame');
 await evaluate("document.querySelector('#fold').click()");
 assert.equal(await evaluate("document.querySelector('#sheet').getAttribute('aria-busy')"),'true');
 await wait("document.querySelector('#sheet').getAttribute('aria-busy')==='false'");
 assert.ok(await evaluate('window.paperView.renderer.info.render.frame')>beforeFold+1,'animated folds redraw');
 await send('Emulation.setEmulatedMedia',{features:[{name:'prefers-reduced-motion',value:'reduce'}]});
 await evaluate(`document.querySelector('#plane-design').click();document.querySelector('[data-design="legacy"]').click();document.querySelector('#finish-fold').click();document.querySelector('#go-throw').click();`);
 await sleep(250);
 assert.ok(await evaluate('window.flightRenders')>0,'flight renderer resumes');
 await evaluate(`document.querySelector('#sling').dispatchEvent(new KeyboardEvent('keydown',{key:' '}))`);
 await wait("!document.querySelector('#result').hidden");
 assert.notEqual(await evaluate("document.querySelector('#result-label').textContent"),'FLIGHT INTERRUPTED');
 await evaluate("document.querySelector('#review-flight').click();document.querySelector('#review-play').click()");await sleep(250);
 assert.equal(await evaluate("document.querySelector('#review-controls').hidden"),false);
 await evaluate("document.querySelector('#bench-tab').click()");
 assert.deepEqual(await evaluate(layoutCheck),[]);
 assert.deepEqual(errors,[],'uncaught browser errors');
 console.log(JSON.stringify({layouts,pickerOpenMs:picker,idleRendering:'passed',flightAndReplay:'passed',errors},null,2));
}finally{
 try{await send('Page.close');}finally{ws.close();}
}
