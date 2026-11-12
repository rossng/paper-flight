// Dependency-free Chrome DevTools runner. Start isolated Chrome and Vite first;
// see docs/TASK_REPORT.md. No application debug hooks or production telemetry.
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
await evaluate(`(async()=>{
 const loaded=path=>performance.getEntriesByType('resource').findLast(e=>new URL(e.name).pathname===path)?.name??path;
 const {Simulation}=await import(loaded('/src/physics/simulation.ts'));
 const {Viewport}=await import(loaded('/src/rendering/viewport.ts'));
 const {FlightRecording}=await import(loaded('/src/rendering/flight-recording.ts'));
 const wrap=(prototype,name,key)=>{const original=prototype[name];prototype[name]=function(...args){const t=performance.now();try{return original.apply(this,args);}finally{window.measure[key]+=performance.now()-t;}};};
 const method=Simulation.prototype.stepUntil?'stepUntil':'step';wrap(Simulation.prototype,method,'physics');wrap(Simulation.prototype,'metrics','metrics');wrap(Simulation.prototype,'deformation','deformation');
 wrap(FlightRecording.prototype,'capture','record');wrap(Viewport.prototype,'render','render');
 const step=Simulation.prototype[method];Simulation.prototype[method]=function(...args){window.profileSim=this;window.measure.steps++;return step.apply(this,args);};
 const render=Viewport.prototype.render;Viewport.prototype.render=function(){window.profileViewport=this;return render.call(this);};
})()`);
const report={context:{date:new Date().toISOString(),browser:await send('Browser.getVersion'),viewport:'1280x800 DPR1',url,warmup:'one complete flight per case, then two measured complete flights',note:'Headless Chrome; rAF/JS and long tasks, not physical display presentation or GPU timings.'},cases:[]};
try{
 for(const name of ['legacy-paper','dart-paper','legacy-tissue','dart-tissue']){
  const [design,material]=name.split('-');
  await evaluate(`document.querySelector('#bench-tab').click();document.querySelector('#plane-design').click();document.querySelector('[data-design="${design}"]').click();document.querySelector('[data-weight="80"]').click();${material==='tissue'?"document.querySelector('#special-material').click();document.querySelector('[data-material=\"tissue\"]').click();":''}document.querySelector('#finish-fold').click();document.querySelector('#go-throw').click();`);
  if(material==='tissue')await evaluate(`document.querySelector('#weather-button').click();document.querySelector('[data-breeze="3"]').click();document.querySelector('[data-wind-direction="90"]').click();for(const id of ['weather-gusts','weather-turbulence']){const e=document.getElementById(id);e.checked=true;e.dispatchEvent(new Event('change'));}document.querySelector('#weather-done').click();`);
  else await evaluate(`document.querySelector('#weather-button').click();document.querySelector('[data-breeze="0"]').click();for(const id of ['weather-gusts','weather-turbulence']){const e=document.getElementById(id);e.checked=false;e.dispatchEvent(new Event('change'));}document.querySelector('#weather-done').click();`);
  const runs=[];
  for(let run=-1;run<2;run++){
   if(run>=0)await evaluate("document.querySelector('#reset').click()");
   await evaluate(`Object.assign(window.measure,{frames:[],longTasks:[],active:true,last:0});${material==='tissue'?"for(let i=0;i<20;i++)document.querySelector('#sling').dispatchEvent(new KeyboardEvent('keydown',{key:'ArrowDown'}));":''}document.querySelector('#sling').dispatchEvent(new KeyboardEvent('keydown',{key:' '}));`);
   await wait("window.profileSim?.status==='flying'");
   await wait("!document.querySelector('#result').hidden");
   const measured=await evaluate(`(()=>{window.measure.active=false;const s=window.profileSim;return {...window.measure,final:{status:s.status,time:s.time,center:s.center(),peakDeformation:s.peakDeformation},result:document.querySelector('#result').innerText};})()`);
   if(measured.final.status!=='landed')throw Error(measured.result);
   if(run>=0){const frames=measured.frames;const summary={final:measured.final,frames:frames.length,frameMs:stats(frames.map(f=>f.duration)),intervalMs:stats(frames.map(f=>f.interval)),physicsMs:stats(frames.map(f=>f.physics)),renderMs:stats(frames.map(f=>f.render)),recordMs:stats(frames.map(f=>f.record)),metricsMs:stats(frames.map(f=>f.metrics)),deformationMs:stats(frames.map(f=>f.deformation)),steps:stats(frames.map(f=>f.steps)),longTasks:stats(measured.longTasks.map(t=>t.duration)),realTimeRatio:measured.final.time/(frames.reduce((s,f)=>s+f.interval,0)/1000)};runs.push(summary);console.log(name,run,JSON.stringify(summary));}
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
 report.errors=errors;if(errors.length)throw Error(JSON.stringify(errors));
 writeFileSync(option('--output','/private/tmp/physics-performance-browser.json'),JSON.stringify(report,null,2)+'\n');
}finally{try{await send('Page.navigate',{url:'about:blank'});await send('Page.close');}catch{/* Target may close before acknowledging Page.close. */}ws.close();}
