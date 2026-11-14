import test from 'node:test';
import assert from 'node:assert/strict';
import {challengeVerdict,encodeShare,readChallenge,shareText,shareUrl,type SharedFlight} from '../src/share';
import {readDesignSettings,readLaunchSettings} from '../src/folding/settings';

const flight:SharedFlight={
 design:'stable',parameters:{material:'paper',paperWeight:110,size:1.15,span:1.2000000000000002,dihedral:16,elevator:8,height:5},
 throwAngle:-32.4,trim:3,breeze:1.5,windHeading:90,gusts:true,turbulence:false,distance:12.345,time:3.21,
};

test('a shared link opens the same plane and throw through the game’s own URL settings',()=>{
 const url=new URL(shareUrl(flight,'https://example.com/paper-flight/?design=dart#about'));
 assert.equal(url.pathname,'/paper-flight/');assert.equal(url.hash,'');assert.equal(url.searchParams.get('design'),'stable');
 const settings=readDesignSettings(url.searchParams);
 assert.deepEqual({material:settings.material,paperWeight:settings.paperWeight,size:settings.size,span:settings.span,dihedral:settings.dihedral,elevator:settings.elevator,height:settings.height},{...flight.parameters,span:1.2});
 assert.deepEqual(readLaunchSettings(url.searchParams),{throwAngle:-32,trim:3,field:true});
 assert.deepEqual(readChallenge(url.searchParams),{breeze:1.5,windHeading:90,gusts:true,turbulence:false,distance:12.3,time:3.2});
});

test('special materials, calm weather and unflown planes keep the link short',()=>{
 const query=encodeShare({...flight,parameters:{...flight.parameters,material:'foil',paperWeight:135,elevator:0},breeze:0,gusts:false,distance:undefined,time:undefined});
 for(const key of ['paperWeight','elevator','breeze','wind','gusts','swirl','dist','time'])assert.equal(query.has(key),false,key);
 assert.equal(readDesignSettings(query).material,'foil');
 assert.deepEqual(readChallenge(query),{breeze:0,windHeading:0,gusts:false,turbulence:false});
});

test('hand-edited challenges fall back to still air and no distance',()=>{
 assert.deepEqual(readChallenge(new URLSearchParams('breeze=40&wind=45&dist=NaN&time=-1&gusts=yes')),{breeze:0,windHeading:0,gusts:false,turbulence:false});
 assert.deepEqual(readChallenge(new URLSearchParams('breeze=3&wind=45')),{breeze:3,windHeading:0,gusts:false,turbulence:false});
});

test('the verdict compares distances as they are displayed',()=>{
 assert.equal(challengeVerdict(5.64,5.6),'Dead heat with your friend’s 5.6 m!');
 assert.equal(challengeVerdict(5.66,5.6),'You beat your friend’s 5.6 m!');
 assert.equal(challengeVerdict(5.54,5.6),'Your friend’s plane managed 5.6 m. Keep trying!');
});

test('share text names the plane and the distance to beat',()=>{
 assert.equal(shareText(flight),'My paper plane (The Stable) flew 12.3 m in Paper Flight. Can you beat it?');
 assert.equal(shareText({...flight,design:'legacy',distance:undefined}),'Try my paper plane (Original dart) in Paper Flight!');
});

test('planets, special air and the cannon travel with the link',()=>{
 const url=new URL(shareUrl({...flight,world:'moon',windMode:'tornado',cannon:true},'https://example.com/'));
 assert.deepEqual(readChallenge(url.searchParams),{breeze:1.5,windHeading:90,gusts:true,turbulence:false,world:'moon',windMode:'tornado',cannon:true,distance:12.3,time:3.2});
 const everyday=encodeShare({...flight,world:'earth',windMode:'none',cannon:false});
 for(const key of ['world','air','cannon'])assert.equal(everyday.has(key),false,key);
 assert.deepEqual(readChallenge(new URLSearchParams('world=venus&air=hurricane&cannon=yes')),{breeze:0,windHeading:0,gusts:false,turbulence:false});
});
