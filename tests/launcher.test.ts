import test from 'node:test';
import assert from 'node:assert/strict';
import {elasticPull,pullTravel,launchSpeed,springRemaining} from '../src/physics/launcher';

test('equal extra hand travel produces progressively less spring extension',()=>{
 const steps=[0,30,60,90,120,150].map(y=>elasticPull(0,y).y);
 for(let i=2;i<steps.length;i++){assert.ok(steps[i]>steps[i-1]);assert.ok(steps[i]-steps[i-1]<steps[i-1]-steps[i-2]);}
 assert.ok(elasticPull(0,100).y<70);assert.equal(elasticPull(0,10000).y,100);
});
test('elastic drag preserves direction, stays bounded, and can be re-grabbed continuously',()=>{
 for(const [x,y] of [[0,0],[25,50],[-70,40],[0,100]]){
  const travel=pullTravel(x,y),pull=elasticPull(travel.x,travel.y);
  assert.ok(Math.abs(pull.x-x)<1e-10);assert.ok(Math.abs(pull.y-y)<1e-10);
 }
 const pull=elasticPull(-100,100);assert.ok(Math.abs(pull.x/pull.y+1)<1e-10);assert.ok(Math.hypot(pull.x,pull.y)<=100);
 assert.deepEqual(elasticPull(0,-50),{x:0,y:0});
});
test('pull controls a bounded speed range and the cosmetic release accelerates',()=>{
 assert.equal(launchSpeed(-10),0);assert.equal(launchSpeed(0),0);
 assert.equal(launchSpeed(50),5.5);assert.equal(launchSpeed(100),11);assert.equal(launchSpeed(200),11);
 assert.equal(springRemaining(0),1);assert.ok(springRemaining(1)<1e-12);
 assert.ok(1-springRemaining(.2)<springRemaining(.8)-springRemaining(1));
});
