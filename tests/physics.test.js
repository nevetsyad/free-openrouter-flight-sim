import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { resetFlightState, stepFlight, rotateBody, axes, aerodynamicForces, controlAuthority } from '../js/physics.js';
const run = (s, seconds, input = {}) => { for (let i = 0; i < seconds * 120; i++) stepFlight(s, input, 1 / 120); return s; };
const level = () => { const s = resetFlightState(); s.quaternion.identity(); s.position.y = 2000; return s; };
test('positive local X and up control pitch -Z nose upward', () => {
  const s = level(); rotateBody(s, 0.1, 0); assert.ok(axes(s).forward.y > 0);
  const control = level(); stepFlight(control, { pitch: 1 }, 1 / 60); assert.ok(axes(control).forward.y > 0);
});
test('signed AoA, perpendicular lift, dissipative drag', () => {
  for (const angle of [-0.2, 0.2, 0.8]) {
    const s = level(); rotateBody(s, angle, 0); const a = aerodynamicForces(s);
    assert.ok(Math.abs(a.alpha - angle) < 1e-10);
    assert.ok(Math.abs(a.lift.dot(s.velocity)) < 1e-8);
    assert.ok(a.drag.dot(s.velocity) <= 0 && a.side.dot(s.velocity) <= 0);
  }
  const s = level(); rotateBody(s, 0.25, 0); const peak = aerodynamicForces(s);
  rotateBody(s, 0.55, 0); const stalled = aerodynamicForces(s);
  assert.ok(stalled.lift.length() < peak.lift.length() / 5);
  assert.ok(stalled.drag.length() > peak.drag.length());
});
test('banked pull turns left and right with correct velocity and nose direction', () => {
  for (const bank of [-1, 1]) {
    const s = level(); rotateBody(s, 0, bank * Math.PI / 3);
    run(s, 2, { pitch: 0.35 });
    assert.ok(s.velocity.x * bank < -1, `bank ${bank}: vx ${s.velocity.x}`);
    assert.ok(axes(s).forward.x * bank < 0, `bank ${bank}: nose ${axes(s).forward.x}`);
  }
});
test('low speed nose-high stall loses height and recovers by unloading with power', () => {
  const s = level(); s.velocity.set(0, 0, -15); s.throttle = 0; rotateBody(s, 0.5, 0);
  const start = s.position.y; run(s, 2);
  assert.ok(s.position.y < start - 10); assert.ok(s.velocity.y < -5); assert.ok(s.stalled);
  s.throttle = 1; run(s, 6);
  assert.ok(s.airspeed > 30, `speed ${s.airspeed}`);
  assert.equal(s.stalled, false); assert.ok(Math.abs(s.angleOfAttack) < 0.3);
  run(s, 5, { pitch: 0.3 });
  assert.ok(s.velocity.y > 0, `recovered climb ${s.velocity.y}`);
  assert.equal(s.stalled, false); assert.ok(s.position.y > 1500);
});
test('full local loops and rolls stay continuous and normalized, including inverted pull', () => {
  for (const mode of ['pitch', 'roll']) {
    const s = level(), start = s.quaternion.clone();
    for (let i = 0; i < 720; i++) {
      const previous = s.quaternion.clone(); rotateBody(s, mode === 'pitch' ? Math.PI / 360 : 0, mode === 'roll' ? Math.PI / 360 : 0);
      assert.ok(previous.angleTo(s.quaternion) < 0.009); assert.ok(Math.abs(s.quaternion.length() - 1) < 1e-12);
    }
    assert.ok(start.angleTo(s.quaternion) < 1e-7);
  }
  const s = level(); rotateBody(s, 0, Math.PI); rotateBody(s, 0.1, 0); assert.ok(axes(s).forward.y < 0);
});
test('landing contact, taxi steering, takeoff and gear-up ground clearance', () => {
  const s = level(); s.position.y = 1.1; s.velocity.set(0, -2, -12); s.throttle = 0;
  run(s, 0.5); assert.equal(s.grounded, true); assert.equal(s.position.y, 0.98); assert.equal(s.velocity.y, 0);
  run(s, 1, { roll: -1 }); assert.ok(axes(s).forward.x > 0);
  s.quaternion.identity(); s.velocity.set(0, 0, -12); s.throttle = 1;
  run(s, 12, { pitch: 0.12 }); assert.equal(s.grounded, false); assert.ok(s.position.y > 5, `height ${s.position.y}`);
  const belly = level(); belly.gearDown = false; belly.position.y = 0; belly.velocity.set(0, -1, 0);
  stepFlight(belly, {}, 1 / 60); assert.equal(belly.position.y, 0.4);
});
test('reset restores all flight state and pathological dt stays finite', () => {
  const s = level(); run(s, 4, { pitch: 1, roll: -1 }); s.gearDown = false; resetFlightState(s);
  assert.deepEqual(s.position.toArray(), [0, 220, 0]); assert.deepEqual(s.velocity.toArray(), [0, 0, -52]);
  assert.equal(s.gearDown, true); assert.equal(s.stalled, false); assert.equal(s.grounded, false); assert.equal(s.throttle, 0.62);
  for (const dt of [0, -1, NaN, Infinity, 1e-8, 1 / 60, 0.25, 1000]) {
    stepFlight(s, { pitch: 1, roll: 1 }, dt);
    assert.ok([...s.position.toArray(), ...s.velocity.toArray(), ...s.quaternion.toArray(), s.airspeed].every(Number.isFinite));
  }
});
test('frame partition consistency over 10 seconds', () => {
  const a = level(), b = level(); run(a, 10, { pitch: 0.1 });
  for (let i = 0; i < 300; i++) stepFlight(b, { pitch: 0.1 }, 1 / 30);
  assert.ok(a.position.distanceTo(b.position) < 1e-7);
});
test('actual sustained flight controls complete loops and rolls without attitude jumps', () => {
  for (const mode of ['pitch', 'roll']) {
    const s = level(); s.velocity.set(0, 0, -75); s.throttle = 1;
    let previousAngle = 0, total = 0;
    for (let i = 0; i < 2400; i++) {
      const old = s.quaternion.clone(); stepFlight(s, { [mode]: 1 }, 1 / 120);
      assert.ok(old.angleTo(s.quaternion) < 0.05);
      const v = mode === 'pitch' ? axes(s).forward : axes(s).up;
      const angle = mode === 'pitch' ? Math.atan2(v.y, -v.z) : Math.atan2(-v.x, v.y);
      total += Math.atan2(Math.sin(angle - previousAngle), Math.cos(angle - previousAngle)); previousAngle = angle;
    }
    assert.ok(total > 2 * Math.PI, `${mode} rotated ${total} radians`);
    assert.ok(!s.grounded);
  }
});

test('safe touchdown is undamaged; hard and belly touchdowns damage components', () => {
  const land = (sink, gear = true) => { const s = level(); s.position.y = gear ? 0.99 : 0.41; s.velocity.set(0, -sink, -30); s.gearDown = gear; stepFlight(s, {}, 1/120); return s; };
  assert.ok(Object.values(land(2).health).every(v => v === 1));
  const hard = land(8); assert.ok(hard.health.gear < 0.5 && hard.health.leftWing < 1); assert.equal(hard.crashed, false);
  const belly = land(2, false); assert.ok(belly.health.engine < 0.7); assert.match(belly.damageCause, /BELLY/);
});
test('overspeed and excessive G need sustained exposure, not a single frame', () => {
  const hold = (s, speed, alpha, seconds) => { for(let i=0;i<seconds*120;i++) { s.velocity.set(0,0,-speed); s.quaternion.identity(); rotateBody(s,alpha,0); stepFlight(s,{},1/120); } };
  const fast = level(); hold(fast, 120, 0, 0.5); assert.equal(fast.health.leftWing,1);
  hold(fast,120,0,1); assert.ok(fast.health.leftWing < 0.95); assert.equal(fast.damageCause,'OVERSPEED');
  for(const alpha of [0.25,-0.25]) { const g=level(); hold(g,90,alpha,0.2); assert.equal(g.health.leftWing,1); hold(g,90,alpha,1); assert.ok(g.health.leftWing<0.9); assert.equal(g.damageCause,'EXCESSIVE G'); }
});
test('asymmetric wing loss halves lift and rolls toward failed wing', () => {
  for(const part of ['leftWing','rightWing']) {
    const s=level(), intact=aerodynamicForces(s).lift.length(); s.health[part]=0;
    assert.ok(Math.abs(aerodynamicForces(s).lift.length()/intact-0.5)<1e-10);
    stepFlight(s,{},1/120); assert.ok(axes(s).right.y * (part==='leftWing'?-1:1)>0);
  }
});
test('dynamic pressure and separated flow limit actual controls; failed surfaces and engine respond less', () => {
  const s=level(); s.velocity.set(0,0,0); assert.equal(controlAuthority(s),0);
  const q=s.quaternion.clone(); stepFlight(s,{pitch:1,roll:1},1/120); assert.ok(s.quaternion.angleTo(q)<1e-8);
  s.velocity.set(0,0,-10); const low=controlAuthority(s); s.velocity.set(0,0,-52); const full=controlAuthority(s); assert.ok(low<full/10);
  rotateBody(s,0.8,0); assert.ok(controlAuthority(s)<full/2);
  const healthy=level(), damaged=level(); damaged.health.elevator=0; damaged.health.ailerons=0; damaged.health.rudder=0;
  stepFlight(healthy,{pitch:1,roll:1},1/120); stepFlight(damaged,{pitch:1,roll:1},1/120);
  assert.ok(axes(healthy).forward.y>0); assert.equal(axes(damaged).forward.y,0); assert.equal(axes(damaged).right.y,0);
  const powered=level(), dead=level(); dead.health.engine=0; run(powered,1); run(dead,1); assert.ok(powered.airspeed>dead.airspeed+2);
});
test('destroyed controls cannot auto-level or taxi-steer; crash latches until reset', () => {
  const s=level(); s.position.y=0.98; s.grounded=true; s.velocity.set(0,0,-8); s.throttle=0; rotateBody(s,0,0.1);
  s.health.elevator=0; s.health.ailerons=0; s.health.rudder=0; const q=s.quaternion.clone(); run(s,0.1,{roll:1}); assert.ok(q.angleTo(s.quaternion)<1e-7);
  const crash=level(); crash.position.y=1; crash.velocity.set(0,-20,-40); stepFlight(crash,{},1/120); assert.equal(crash.crashed,true);
  const pos=crash.position.clone(); crash.gearDown=false; run(crash,10,{throttle:1,pitch:1,roll:1}); assert.equal(crash.position.distanceTo(pos),0); assert.equal(crash.airspeed,0);
  resetFlightState(crash); assert.equal(crash.crashed,false); assert.equal(crash.damageCause,''); assert.equal(crash.gExposure,0); assert.equal(crash.overspeedExposure,0); assert.ok(Object.values(crash.health).every(v=>v===1));
});
test('damage exposure and consequences are frame-partition consistent', () => {
  const a=level(), b=level(); a.velocity.z=b.velocity.z=-140;
  run(a,2); for(let i=0;i<60;i++) stepFlight(b,{},1/30);
  assert.ok(a.health.leftWing<1); assert.deepEqual(a.health,b.health); assert.ok(a.position.distanceTo(b.position)<1e-7);
});
test('rising terrain uses closure speed and severe attitude contact crashes', () => {
  const hill=level(); hill.position.set(0,1,0); hill.velocity.set(0,0,-40);
  stepFlight(hill,{},1/120, (x,z)=>-z); assert.equal(hill.crashed,true);
  const inverted=level(); inverted.position.y=0.99; inverted.velocity.set(0,-2,-30); rotateBody(inverted,0,Math.PI);
  stepFlight(inverted,{},1/120); assert.equal(inverted.crashed,true);
});
test('low-speed and post-stall roll inputs produce less rotation in the actual step', () => {
  const response = (speed, alpha) => {
    const a=level(), b=level(); for(const s of [a,b]) {s.velocity.set(0,0,-speed);rotateBody(s,alpha,0);}
    stepFlight(a,{roll:1},1/120); stepFlight(b,{},1/120); return a.quaternion.angleTo(b.quaternion);
  };
  const attached=response(52,0); assert.ok(response(10,0)<attached/10); assert.ok(response(52,0.8)<attached/2);
});
