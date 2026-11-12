import test from 'node:test';
import assert from 'node:assert/strict';
import {liftSummary} from '../src/physics/flight-loads';
import type {PanelLoad} from '../src/physics/aerodynamics';
import type {V3} from '../src/physics/math';
const panel=(lift:V3):PanelLoad=>({lift,force:[10,20,30],center:[0,0,0],alpha:0,area:1,stalled:false});
test('lift meter sums signed lift, excluding drag and opposing panel loads',()=>{
 const result=liftSummary([panel([0,.08,.02]),panel([0,-.03,-.02])],.005,10);
 assert.deepEqual(result.net,[0,.05,0]);assert.equal(result.magnitude,.05);
 assert.equal(result.upward,.05);assert.equal(result.weight,.05);assert.equal(result.supportRatio,1);
 assert.equal(liftSummary([panel([0,1,0]),panel([0,-1,0])],.005,10).magnitude,0);
});
test('banked and inverted lift retain vertical support sign; zero weight is undefined',()=>{
 const banked=liftSummary([panel([.03,.04,0])],.005,10);
 assert.equal(banked.magnitude,.05);assert.ok(Math.abs(banked.supportRatio!-.8)<1e-12);
 assert.equal(liftSummary([panel([0,-.1,0])],.005,10).supportRatio,-2);
 assert.equal(liftSummary([panel([0,1,0])],.005,0).supportRatio,null);
 assert.equal(liftSummary([],1,10).supportRatio,0);
});
