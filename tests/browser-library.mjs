// Optional browser acceptance check; point PLAYWRIGHT_MODULE at an installed Playwright module.
import assert from 'node:assert/strict';
import {readFile,mkdir} from 'node:fs/promises';
import {designs,sheetDesigns} from '../src/folding/library.ts';
import {designFold,foldFilename} from '../src/folding/downloads.ts';
import {DEFAULTS} from '../src/physics/simulation.ts';
const {chromium}=await import(process.env.PLAYWRIGHT_MODULE||'playwright');
const base=process.env.LIBRARY_URL||'http://127.0.0.1:5193/paper-flight/';
const output=process.env.LIBRARY_ARTIFACTS||'/tmp/plane-library-browser';
await mkdir(output,{recursive:true});
const browser=await chromium.launch({headless:true,executablePath:process.env.CHROME_PATH||'/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const context=await browser.newContext({viewport:{width:1280,height:900},acceptDownloads:true,reducedMotion:'reduce'});
const page=await context.newPage(),errors=[],badRequests=[];let workers=0;
page.on('worker',()=>workers++);
page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.url().startsWith(base)&&r.status()>=400)badRequests.push(`${r.status()} ${r.url()}`);});
async function download(button){const pending=page.waitForEvent('download');await page.locator(button).click();const d=await pending;await d.saveAs(`${output}/${d.suggestedFilename()}`);return {name:d.suggestedFilename(),data:JSON.parse(await readFile(await d.path(),'utf8'))};}
try{
 await page.goto(base);await page.locator('#plane-design').waitFor();
 await page.locator('#library-downloads').click();assert.equal(await page.locator('#download-fold').isDisabled(),true);assert.equal(await page.locator('#design-source').isVisible(),false);await page.locator('#downloads-close').click();
 if(!process.argv.includes('--layout-only'))for(const design of designs){
  await page.locator('#bench-tab').click();await page.locator('#plane-design').click();assert.equal(await page.locator('[data-design]').count(),10);
  await page.locator(`[data-design="${design.id}"]`).click();
  if(design.recipe){await page.locator('#library-downloads').click();const file=await download('#download-fold');assert.equal(file.name,foldFilename(design.id,DEFAULTS));assert.deepEqual(file.data,JSON.parse(JSON.stringify(designFold(design.id,DEFAULTS))));await page.locator('#downloads-close').click();}
  await page.locator('#finish-fold').click();await page.locator('#go-throw').click();await page.locator('#sling').press('Space');
  await page.waitForFunction(()=>!document.querySelector('#result').hidden,{},{timeout:90000});
  assert.ok(['TOUCHDOWN!','NEW RECORD!'].includes(await page.locator('#result-label').innerText()));console.log('launch',design.id,await page.locator('#result-distance').innerText());
 }
 if(!process.argv.includes('--layout-only'))assert.ok(workers>=designs.length,`Expected worker launches for all designs, saw ${workers}`);
 await page.locator('#bench-tab').click();await page.locator('#library-downloads').click();const collection=await download('#download-collection');assert.equal(collection.name,'paper-flight-collection.fold');assert.equal(collection.data.file_frames.length+1,18);await page.locator('#downloads-close').click();
 // Every sheet keeps its source and downloads the current workbench settings.
 const settings={...DEFAULTS,size:.85,span:1.35,dihedral:16,paperWeight:110,stiffness:(110/80)**3};
 for(const design of sheetDesigns){
  await page.goto(`${base}?design=${design.id}&size=0.85&span=1.35&dihedral=16&paperWeight=110`);
  assert.equal(await page.locator('[data-size="0.85"]').getAttribute('aria-pressed'),'true');
  await page.locator('#library-downloads').click();
  assert.equal(await page.locator('#design-source').getAttribute('href'),design.recipe.source);
  const file=await download('#download-fold');assert.equal(file.name,foldFilename(design.id,settings));
  assert.deepEqual(file.data,JSON.parse(JSON.stringify(designFold(design.id,settings))));
  await page.locator('#downloads-close').click();
 }
 await page.locator('#plane-design').click();await page.screenshot({path:`${output}/desktop-picker.png`});await page.locator('[aria-selected="true"][data-design]').press('Escape');
 // Keyboard reaches both ends of the scrollable picker at narrow widths.
 await page.setViewportSize({width:390,height:844});await page.locator('#plane-design').click();await page.locator('[aria-selected="true"][data-design]').press('Home');assert.equal(await page.locator(':focus').getAttribute('data-design'),'legacy');await page.locator(':focus').press('End');assert.equal(await page.locator(':focus').getAttribute('data-design'),'raven');assert.equal(await page.locator('[data-design="raven"]').evaluate(el=>{const r=el.getBoundingClientRect();return document.elementFromPoint(r.x+r.width/2,r.y+r.height/2)?.closest('[data-design]')===el;}),true);await page.screenshot({path:`${output}/mobile-picker.png`});await page.locator('[data-design="raven"]').click();
 assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);
 await page.locator('#library-downloads').click();assert.equal(await page.locator('#download-fold').isVisible(),true);await page.screenshot({path:`${output}/mobile-downloads.png`});
 assert.deepEqual(errors,[]);assert.deepEqual(badRequests,[]);console.log(process.argv.includes('--layout-only')?'PASS: focused layout/settings/source/download recheck; no runtime errors or local HTTP failures.':'PASS: 10 worker launches, 9 main downloads, 9 downloads with custom settings, collection, settings, keyboard, mobile, /paper-flight/ paths; no runtime errors or local HTTP failures.');
}finally{await browser.close();}
