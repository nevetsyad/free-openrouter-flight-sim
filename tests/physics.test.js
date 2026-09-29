import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { resetFlightState, stepFlight, rotateBody, axes, aerodynamicForces } from '../js/physics.js';
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
