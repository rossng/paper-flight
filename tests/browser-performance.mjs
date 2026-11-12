// Dependency-free Chrome DevTools runner. Start isolated Chrome and Vite first;
// see docs/performance.md. No application debug hooks or production telemetry.
import { writeFileSync } from 'node:fs';
const option=(name,fallback)=>process.argv.includes(name)?process.argv[process.argv.indexOf(name)+1]:fallback;
const port=option('--cdp','9297'),url=option('--url','http://127.0.0.1:5197/');
const page=await(await fetch(`http://127.0.0.1:${port}/json/new?about:blank`,{method:'PUT'})).json();
const ws=new WebSocket(page.webSocketDebuggerUrl);
await new Promise(resolve=>ws.addEventListener('open',resolve));
let id=0;const pending=new Map(),errors=[];
ws.addEventListener('message',event=>{const m=JSON.parse(event.data);if(m.id){const p=pending.get(m.id);pending.delete(m.id);m.error?p.reject(m.error):p.resolve(m.result);}if(m.method==='Runtime.exceptionThrown')errors.push(m.params.exceptionDetails);});
const send=(method,params={})=>new Promise((resolve,reject)=>{const n=++id;pending.set(n,{resolve,reject});ws.send(JSON.stringify({id:n,method,params}));});
const evaluate=async expression=>{const r=await send('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value;};
const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const wait=async(expression,timeout=120000)=>{const start=Date.now();while(!await evaluate(expression)){if(Date.now()-start>timeout)throw Error(`Timeout: ${expression}`);await sleep(100);}};
const stats=a=>{a=[...a].sort((a,b)=>a-b);return {count:a.length,median:a[Math.floor(a.length*.5)],p95:a[Math.min(a.length-1,Math.floor(a.length*.95))],max:a.at(-1)};};
await send('Runtime.enable');await send('Page.enable');
await send('Emulation.setDeviceMetricsOverride',{width:1280,height:800,deviceScaleFactor:1,mobile:false});
await send('Emulation.setEmulatedMedia',{features:[{name:'prefers-reduced-motion',value:'reduce'}]});
await send('Page.addScriptToEvaluateOnNewDocument',{source:`
 (()=>{
 window.workerStats=[];
 const NativeWorker=window.Worker;
 window.Worker=class extends NativeWorker {
  constructor(...args){super(...args);this.stats={created:performance.now(),terminated:false,time:0,status:'starting',requests:0,targets:0,lastTarget:0,launch:null};window.workerStats.push(this.stats);this.addEventListener('message',e=>{if(e.data.type==='state'){this.stats.time=e.data.time;this.stats.status=e.data.status;this.stats.workMs=e.data.workMs;}});}
  postMessage(message,...args){this.stats.requests++;if(message.type==='launch')this.stats.launch=message.parameters;if(message.type==='target'){this.stats.targets++;this.stats.lastTarget=message.time;}return super.postMessage(message,...args);}
  terminate(){this.stats.terminated=true;return super.terminate();}
 };
 window.measure={frames:[],longTasks:[],active:false,steps:0,physics:0,render:0,record:0,metrics:0,deformation:0,last:0};
 const raf=requestAnimationFrame;window.requestAnimationFrame=callback=>raf.call(window,time=>{
  if(callback.name!=='animate'){callback(time);return;}
  const m=window.measure,active=m.active&&window.profileSim?.status==='flying',start=performance.now();
  m.steps=0;m.physics=0;m.render=0;m.record=0;m.metrics=0;m.deformation=0;
  callback(time);
  if(active)m.frames.push({duration:performance.now()-start,interval:m.last?time-m.last:0,steps:m.steps,physics:m.physics,render:m.render,record:m.record,metrics:m.metrics,deformation:m.deformation,time:window.profileSim.time});
  m.last=time;
 });
 new PerformanceObserver(list=>{if(window.measure.active)window.measure.longTasks.push(...list.getEntries().map(e=>({start:e.startTime,duration:e.duration})));}).observe({type:'longtask',buffered:false});
 })();
`});
await send('Page.navigate',{url});await wait("!!document.querySelector('#finish-fold')");await sleep(1500);
if(!process.argv.includes('--production'))await evaluate(`(async()=>{
 const loaded=path=>performance.getEntriesByType('resource').findLast(e=>new URL(e.name).pathname===path)?.name??path;
 const {Simulation}=await import(loaded('/src/physics/simulation.ts'));
 const {Viewport}=await import(loaded('/src/rendering/viewport.ts'));
 const {FlightRecording}=await import(loaded('/src/rendering/flight-recording.ts'));
 const wrap=(prototype,name,key)=>{const original=prototype[name];prototype[name]=function(...args){const t=performance.now();try{return original.apply(this,args);}finally{window.measure[key]+=performance.now()-t;}};};
 const method=Simulation.prototype.stepUntil?'stepUntil':'step';wrap(Simulation.prototype,method,'physics');wrap(Simulation.prototype,'metrics','metrics');wrap(Simulation.prototype,'deformation','deformation');
 wrap(FlightRecording.prototype,'capture','record');wrap(Viewport.prototype,'render','render');
 const step=Simulation.prototype[method];Simulation.prototype[method]=function(...args){window.profileSim=this;window.measure.steps++;return step.apply(this,args);};
 const render=Viewport.prototype.render;Viewport.prototype.render=function(){window.profileViewport=this;window.profileSim=this.views[0]?.simulation;return render.call(this);};
})()`);
const report={context:{date:new Date().toISOString(),browser:await send('Browser.getVersion'),viewport:'1280x800 DPR1',url,warmup:(process.argv.includes('--forecast-only')||process.argv.includes('--production'))?'functional checks; no timing repeats':`${option('--warmups','1')} complete warm-up flight(s), then ${option('--runs','2')} measured complete flights per case; a fresh worker per launch`,note:'Headless Chrome; rAF/JS and long tasks, not physical display presentation or GPU timings.'},cases:[]};
try{
 for(const name of (process.argv.includes('--forecast-only')||process.argv.includes('--production')?[]:option('--cases','legacy-paper,dart-paper,legacy-tissue,dart-tissue').split(','))){
  const [design,material]=name.split('-');
  await evaluate(`document.querySelector('#bench-tab').click();document.querySelector('#plane-design').click();document.querySelector('[data-design="${design}"]').click();document.querySelector('[data-weight="80"]').click();${material==='tissue'?"document.querySelector('#special-material').click();document.querySelector('[data-material=\"tissue\"]').click();":''}document.querySelector('#finish-fold').click();document.querySelector('#go-throw').click();`);
  if(material==='tissue')await evaluate(`document.querySelector('#weather-button').click();document.querySelector('[data-breeze="3"]').click();document.querySelector('[data-wind-direction="90"]').click();for(const id of ['weather-gusts','weather-turbulence']){const e=document.getElementById(id);e.checked=true;e.dispatchEvent(new Event('change'));}document.querySelector('#weather-done').click();`);
  else await evaluate(`document.querySelector('#weather-button').click();document.querySelector('[data-breeze="0"]').click();for(const id of ['weather-gusts','weather-turbulence']){const e=document.getElementById(id);e.checked=false;e.dispatchEvent(new Event('change'));}document.querySelector('#weather-done').click();`);
  const runs=[];
  for(let run=-Number(option('--warmups','1'));run<Number(option('--runs','2'));run++){
   if(run>=0)await evaluate("document.querySelector('#reset').click()");
   await evaluate(`Object.assign(window.measure,{frames:[],longTasks:[],active:true,last:0});${material==='tissue'?"for(let i=0;i<20;i++)document.querySelector('#sling').dispatchEvent(new KeyboardEvent('keydown',{key:'ArrowDown'}));":''}document.querySelector('#sling').dispatchEvent(new KeyboardEvent('keydown',{key:' '}));`);
   await wait("window.profileSim?.status==='flying'");
   await wait("!document.querySelector('#result').hidden");
   const measured=await evaluate(`(()=>{window.measure.active=false;const s=window.profileSim;return {...window.measure,final:{status:s.status,time:s.time,center:s.center(),peakDeformation:s.peakDeformation},worker:window.workerStats.at(-1),result:document.querySelector('#result').innerText};})()`);
   if(measured.final.status!=='landed')throw Error(measured.result);
   if(run>=0){const frames=measured.frames;const summary={final:measured.final,worker:measured.worker,frames:frames.length,frameMs:stats(frames.map(f=>f.duration)),intervalMs:stats(frames.map(f=>f.interval)),physicsMs:stats(frames.map(f=>f.physics)),renderMs:stats(frames.map(f=>f.render)),recordMs:stats(frames.map(f=>f.record)),metricsMs:stats(frames.map(f=>f.metrics)),deformationMs:stats(frames.map(f=>f.deformation)),steps:stats(frames.map(f=>f.steps)),longTasks:stats(measured.longTasks.map(t=>t.duration)),realTimeRatio:measured.final.time/(frames.reduce((s,f)=>s+f.interval,0)/1000)};runs.push(summary);console.log(name,run,JSON.stringify(summary));}
  }
  // Detached replay: seek, play, overlays, camera, then reset and pause/resume.
  await evaluate(`document.querySelector('#review-flight').click();var slider=document.querySelector('#review-time');slider.value=Number(slider.max)/2;slider.dispatchEvent(new Event('input'));for(const id of ['review-flex','review-layers','review-lift-meter','review-stall-meter','review-follow'])document.getElementById(id).click();document.querySelector('#review-play').click();`);
  await sleep(500);
  const replay=await evaluate(`({clock:document.querySelector('#review-clock').textContent,flex:document.querySelector('#flex-readout').textContent,lift:document.querySelector('#lift-newtons').textContent,stall:document.querySelector('#stall-percent').textContent})`);
  await evaluate("document.querySelector('#reset').click();document.querySelector('#sling').dispatchEvent(new KeyboardEvent('keydown',{key:' '}))");
  await wait("window.profileSim?.status==='flying'");
  await wait("!document.querySelector('#pause').disabled");
  await evaluate("document.querySelector('#pause').click()");
  const paused=await evaluate('window.profileSim.time');await sleep(250);
  if(await evaluate('window.profileSim.time')!==paused)throw Error('Pause advanced physics');
  await evaluate("document.querySelector('#pause').click()");await wait(`window.profileSim.time>${paused}`);
  await evaluate("document.querySelector('#reset').click()");
  if(!await evaluate("window.profileViewport.views[0].simulation.status==='ready' && document.querySelector('#review-controls').hidden"))throw Error('Reset failed');
  report.cases.push({name,runs,replay,pauseResumeReset:true});
 }

 if(process.argv.includes('--forecast-only')){
  await evaluate(`document.querySelector('#bench-tab').click();document.querySelector('#plane-design').click();document.querySelector('[data-design="legacy"]').click();document.querySelector('[data-weight="80"]').click();document.querySelector('#finish-fold').click();document.querySelector('#go-throw').click();`);
  await evaluate(`const h=document.querySelector('#launch-height');h.value=.3;h.dispatchEvent(new Event('input'));document.querySelector('#sling').dispatchEvent(new KeyboardEvent('keydown',{key:'ArrowDown'}));document.querySelector('#sling').dispatchEvent(new KeyboardEvent('keydown',{key:'ArrowDown'}));`);
  await wait('window.workerStats.length===1 && window.workerStats[0].time>=.249');
  const prepared=await evaluate(`({time:window.profileSim.time,status:window.profileSim.status,trail:window.profileSim.trail.length,worker:window.workerStats[0]})`);
  if(prepared.time!==0||prepared.status!=='ready'||prepared.trail!==1)throw Error('Forecast leaked into launcher');
  await sleep(250);if(await evaluate('window.workerStats[0].time')!==prepared.worker.time)throw Error('Forecast ran beyond its bounded horizon');
  await evaluate(`document.querySelector('#sling').dispatchEvent(new KeyboardEvent('keydown',{key:' '}));`);
  await wait("window.profileSim.status==='flying'");
  if(await evaluate('window.workerStats.length')!==1)throw Error('Matching release did not reuse worker');
  await wait("!document.querySelector('#result').hidden");
  const comparison=await evaluate(`(async()=>{const loaded=path=>performance.getEntriesByType('resource').findLast(e=>new URL(e.name).pathname===path)?.name??path;const {Simulation}=await import(loaded('/src/physics/simulation.ts'));const expected=new Simulation(window.workerStats[0].launch);expected.launch();while(expected.status==='flying')expected.step();const actual=window.profileSim;return {equal:JSON.stringify([actual.time,actual.positions,actual.velocities,actual.peakDeformation])===JSON.stringify([expected.time,expected.positions,expected.velocities,expected.peakDeformation]),time:actual.time};})()`);
  if(!comparison.equal)throw Error('Adopted flight differs from direct solve');
  await evaluate(`document.querySelector('#reset').click();for(let i=0;i<2;i++)document.querySelector('#sling').dispatchEvent(new KeyboardEvent('keydown',{key:'ArrowDown'}));`);
  await wait('window.workerStats.length===2');await evaluate("document.querySelector('#pitch-up').click()");
  await wait('window.workerStats.length===3');
  if(!await evaluate('window.workerStats[1].terminated && window.workerStats[2].launch.pitch!==window.workerStats[1].launch.pitch'))throw Error('Changed launch reused stale work');
  await evaluate("document.querySelector('#reset').click()");await sleep(250);
  if(!await evaluate("window.workerStats[2].terminated && window.profileSim.time===0 && window.profileSim.status==='ready'"))throw Error('Reset did not cancel forecast');
  await evaluate(`window.savedWorker=window.Worker;window.Worker=class{constructor(){throw Error('Test: workers unavailable');}};document.querySelector('#sling').dispatchEvent(new KeyboardEvent('keydown',{key:' '}));`);
  await wait("!document.querySelector('#result').hidden");
  const fallback=await evaluate(`({status:window.profileSim.status,time:window.profileSim.time})`);if(fallback.status!=='landed')throw Error('Worker fallback failed');
  await evaluate('window.Worker=window.savedWorker');
  await evaluate("document.querySelector('#reset').click()");
  // Reset changes the camera on the next render; hit-test the settled sling.
  await sleep(100);
  const pointer=await evaluate(`(()=>{const r=document.querySelector('#sling').getBoundingClientRect();return {x:r.x+r.width/2,y:r.y+r.height/2,workers:window.workerStats.length};})()`);
  await send('Input.dispatchMouseEvent',{type:'mousePressed',x:pointer.x,y:pointer.y,button:'left',clickCount:1});
  await send('Input.dispatchMouseEvent',{type:'mouseMoved',x:pointer.x+15,y:pointer.y+65,button:'left',buttons:1});
  await wait(`window.workerStats.length===${pointer.workers+1} && window.workerStats.at(-1).time>=.249`);
  if(!await evaluate("window.profileSim.status==='ready' && window.profileSim.time===0"))throw Error('Held pointer forecast moved displayed plane');
  await send('Input.dispatchMouseEvent',{type:'mouseReleased',x:pointer.x+15,y:pointer.y+65,button:'left',clickCount:1});
  await wait("window.profileSim.status==='flying'");
  if(await evaluate('window.workerStats.length')!==pointer.workers+1)throw Error('Pointer release did not reuse held forecast');
  await wait("!document.querySelector('#result').hidden");
  report.forecast={prepared,comparison,canceledOnParameterChange:true,canceledOnReset:true,fallback,pointerHoldReleaseReuse:true};
 }

 if(process.argv.includes('--production')){
  await evaluate(`document.querySelector('#finish-fold').click();document.querySelector('#go-throw').click();document.querySelector('#sling').dispatchEvent(new KeyboardEvent('keydown',{key:' '}));`);
  await wait("!document.querySelector('#result').hidden");
  const flight=await evaluate(`({result:document.querySelector('#result').innerText,workers:window.workerStats})`);
  if(flight.workers.length!==1||flight.workers[0].status!=='landed'||!flight.workers[0].terminated)throw Error('Production worker did not complete flight');
  await evaluate(`document.querySelector('#review-flight').click();const slider=document.querySelector('#review-time');slider.value=Number(slider.max)/2;slider.dispatchEvent(new Event('input'));document.querySelector('#review-play').click();`);await sleep(250);
  const replay=await evaluate("document.querySelector('#review-clock').textContent");
  await evaluate("document.querySelector('#reset').click()");
  if(!await evaluate("document.querySelector('#review-controls').hidden && !document.querySelector('#sling').hidden"))throw Error('Production reset failed');
  report.production={flight,replay,reset:true};
 }
 report.errors=errors;if(errors.length)throw Error(JSON.stringify(errors));
 writeFileSync(option('--output','/private/tmp/physics-performance-browser.json'),JSON.stringify(report,null,2)+'\n');
}finally{try{await send('Page.navigate',{url:'about:blank'});await send('Page.close');}catch{/* Target may close before acknowledging Page.close. */}ws.close();}
