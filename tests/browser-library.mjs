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
const page=await context.newPage(),errors=[],badRequests=[];
page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.url().startsWith(base)&&r.status()>=400)badRequests.push(`${r.status()} ${r.url()}`);});
async function download(button){const pending=page.waitForEvent('download');await page.locator(button).click();const d=await pending;await d.saveAs(`${output}/${d.suggestedFilename()}`);return {name:d.suggestedFilename(),data:JSON.parse(await readFile(await d.path(),'utf8'))};}
try{
 await page.goto(base);await page.locator('#plane-design').waitFor();
 await page.locator('#library-downloads').click();assert.equal(await page.locator('#download-fold').isDisabled(),true);await page.locator('#downloads-close').click();
 if(!process.argv.includes('--layout-only'))for(const design of designs){
  await page.locator('#bench-tab').click();await page.locator('#plane-design').click();assert.equal(await page.locator('[data-design]').count(),10);
  await page.locator(`[data-design="${design.id}"]`).click();
  if(design.recipe){await page.locator('#library-downloads').click();const file=await download('#download-fold');assert.equal(file.name,foldFilename(design.id,DEFAULTS));assert.deepEqual(file.data,JSON.parse(JSON.stringify(designFold(design.id,DEFAULTS))));await page.locator('#downloads-close').click();}
  await page.locator('#finish-fold').click();await page.locator('#go-throw').click();await page.locator('#sling').press('Space');
  await page.waitForFunction(()=>!document.querySelector('#result').hidden,{},{timeout:90000});
  assert.ok(['TOUCHDOWN!','NEW RECORD!'].includes(await page.locator('#result-label').innerText()));console.log('launch',design.id,await page.locator('#result-distance').innerText());
 }
 await page.locator('#bench-tab').click();await page.locator('#library-downloads').click();const collection=await download('#download-collection');assert.equal(collection.name,'paper-flight-collection.fold');assert.equal(collection.data.file_frames.length+1,18);await page.locator('#downloads-close').click();
 // Non-default geometry and material, from game to study and back.
 await page.goto(`${base}?design=stable&size=0.85&span=1.35&dihedral=16&paperWeight=110`);
 assert.equal(await page.locator('[data-size="0.85"]').getAttribute('aria-pressed'),'true');
 await page.locator('#library-downloads').click();const settingsFile=await download('#download-fold');assert.equal(settingsFile.data['paperflight:parameters'].size,.85);
 await page.locator('#design-study').click();await page.locator('#recipe').waitFor();assert.ok(page.url().startsWith(`${base}folding-study.html?`));
 for(const d of sheetDesigns){await page.locator('#recipe').selectOption(d.id);await page.locator('#prev').click();const file=await download('#export');assert.equal(file.data['paperflight:designId'],d.id);assert.equal(file.data['paperflight:appliedSteps'],d.recipe.operations.length-1);assert.equal(file.data['paperflight:parameters'].size,.85);}
 assert.ok((await page.locator('#back-to-game').getAttribute('href')).includes('design=raven'));await page.locator('#fly').click();assert.ok(page.url().startsWith(base));assert.ok(page.url().includes('design=raven'));assert.equal(await page.locator('[data-dihedral="16"]').getAttribute('aria-pressed'),'true');
 await page.locator('#plane-design').click();await page.screenshot({path:`${output}/desktop-picker.png`});await page.locator('[aria-selected="true"][data-design]').press('Escape');
 // Keyboard reaches both ends of the scrollable picker at narrow widths.
 await page.setViewportSize({width:390,height:844});await page.locator('#plane-design').click();await page.locator('[aria-selected="true"][data-design]').press('Home');assert.equal(await page.locator(':focus').getAttribute('data-design'),'legacy');await page.locator(':focus').press('End');assert.equal(await page.locator(':focus').getAttribute('data-design'),'raven');assert.equal(await page.locator('[data-design="raven"]').evaluate(el=>{const r=el.getBoundingClientRect();return document.elementFromPoint(r.x+r.width/2,r.y+r.height/2)?.closest('[data-design]')===el;}),true);await page.screenshot({path:`${output}/mobile-picker.png`});await page.locator('[data-design="raven"]').click();
 assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);
 await page.locator('#library-downloads').click();assert.equal(await page.locator('#download-fold').isVisible(),true);await page.screenshot({path:`${output}/mobile-downloads.png`});
 assert.deepEqual(errors,[]);assert.deepEqual(badRequests,[]);console.log(process.argv.includes('--layout-only')?'PASS: focused layout/settings/study/download recheck; no runtime errors or local HTTP failures.':'PASS: 10 launches, 9 main downloads, 9 partial study downloads, collection, settings, keyboard, mobile, /paper-flight/ paths; no runtime errors or local HTTP failures.');
}finally{await browser.close();}
