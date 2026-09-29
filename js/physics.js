import * as THREE from 'three';

const X = new THREE.Vector3(1, 0, 0), Y = new THREE.Vector3(0, 1, 0), Z = new THREE.Vector3(0, 0, 1);
export const SETTINGS = Object.freeze({ gravity: 9.81, pitchRate: 0.7, rollRate: 1.1, stallAngle: 0.30, step: 1 / 120, overspeed: 100, positiveG: 6, negativeG: -3 });
export function resetFlightState(s = {}) {
  s.position = new THREE.Vector3(0, 220, 0);
  s.quaternion = new THREE.Quaternion().setFromAxisAngle(X, 0.04);
  s.velocity = new THREE.Vector3(0, 0, -52);
  s.throttle = 0.62; s.gearDown = true; s.stalled = false; s.grounded = false;
  s.airspeed = 52; s.angleOfAttack = 0;
  s.health = Object.fromEntries(['leftWing', 'rightWing', 'elevator', 'ailerons', 'rudder', 'engine', 'gear'].map(k => [k, 1]));
  s.crashed = false; s.damageCause = ''; s.loadFactor = 1;
  s.overspeedExposure = 0; s.gExposure = 0;
  return s;
}
export function axes(s) {
  return { forward: new THREE.Vector3(0, 0, -1).applyQuaternion(s.quaternion),
    up: Y.clone().applyQuaternion(s.quaternion), right: X.clone().applyQuaternion(s.quaternion) };
}
// Postmultiply: controls rotate around the aircraft's axes, including inverted flight.
// With -Z forward and +Y up, positive LOCAL X pitches the nose UP.
export function rotateBody(s, pitch, roll, yaw = 0) {
  s.quaternion.multiply(new THREE.Quaternion().setFromAxisAngle(X, pitch));
  s.quaternion.multiply(new THREE.Quaternion().setFromAxisAngle(Z, roll));
  s.quaternion.multiply(new THREE.Quaternion().setFromAxisAngle(Y, yaw)).normalize();
}
export function aerodynamicForces(s) {
  const { forward, up, right } = axes(s);
  const speed = s.velocity.length();
  const direction = speed > 1e-8 ? s.velocity.clone().divideScalar(speed) : forward.clone();
  const along = s.velocity.dot(forward);
  // Nose above flight path => positive AoA; do not discard its sign.
  const alpha = Math.atan2(-s.velocity.dot(up), along);
  const absAlpha = Math.abs(alpha);
  const attached = absAlpha <= SETTINGS.stallAngle;
  const retention = attached ? 1 : Math.exp(-(absAlpha - SETTINGS.stallAngle) * 10);
  const cl = THREE.MathUtils.clamp(0.26 + 4.4 * alpha, -1.3, 1.5) * retention;
  // Project wing-up perpendicular to airflow. Lift cannot add kinetic energy.
  const liftDirection = up.clone().addScaledVector(direction, -up.dot(direction));
  if (liftDirection.lengthSq() > 1e-12) liftDirection.normalize();
  const lift = liftDirection.multiplyScalar(0.0085 * speed * speed * cl * (s.health.leftWing + s.health.rightWing) / 2);
  const cd = 0.024 + (2 - s.health.leftWing - s.health.rightWing) * 0.10 + (s.gearDown ? 0.012 : 0) + 0.045 * cl * cl + (1 - retention) * 0.65;
  const drag = direction.clone().multiplyScalar(-0.0085 * speed * speed * cd);
  // Dissipative sideslip damping; coordinated yaw below follows, never redirects velocity.
  const side = right.multiplyScalar(-s.velocity.dot(right) * speed * 0.009);
  return { lift, drag, side, alpha, speed, stalled: !attached || speed < 25, forward };
}
// Health and thresholds are deliberately tuned arcade values, not certification limits.
function damage(s, cause, amounts) {
  for (const [part, amount] of Object.entries(amounts)) s.health[part] = Math.max(0, s.health[part] - amount);
  s.damageCause = cause;
}
export function controlAuthority(s) {
  const a = aerodynamicForces(s);
  const pressure = Math.min(1, (a.speed / 35) ** 2);
  // Separated airflow weakens controls; zero dynamic pressure means zero authority.
  return pressure * (0.22 + 0.78 * Math.exp(-Math.max(0, Math.abs(a.alpha) - SETTINGS.stallAngle) * 5));
}
function substep(s, input, dt, terrainHeightAt) {
  if (s.crashed) {
    s.velocity.set(0, 0, 0); s.throttle = 0; s.airspeed = 0; s.stalled = false; s.grounded = true;
    return;
  }
  const wasGrounded = s.grounded;
  const oldTerrain = terrainHeightAt(s.position.x, s.position.z);
  s.throttle = THREE.MathUtils.clamp(s.throttle + (input.throttle || 0) * dt * 0.28, 0, 1);
  const authority = controlAuthority(s), h = s.health;
  const turn = input.roll || 0;
  const wheels = s.gearDown && h.gear > 0.2;
  rotateBody(s, (input.pitch || 0) * SETTINGS.pitchRate * authority * h.elevator * dt,
    s.grounded ? 0 : turn * SETTINGS.rollRate * authority * h.ailerons * dt,
    s.grounded && wheels ? turn * 0.65 * Math.min(1, s.velocity.length() / 5) * h.gear * h.rudder * dt : 0);
  let a = aerodynamicForces(s);
  s.loadFactor = a.lift.dot(axes(s).up) / SETTINGS.gravity;
  s.overspeedExposure = a.speed > SETTINGS.overspeed ? s.overspeedExposure + dt : Math.max(0, s.overspeedExposure - dt * 2);
  s.gExposure = s.loadFactor > SETTINGS.positiveG || s.loadFactor < SETTINGS.negativeG ? s.gExposure + dt : Math.max(0, s.gExposure - dt * 2);
  if (s.overspeedExposure > 0.75) {
    const d = dt * 0.12 * Math.max(1, (a.speed - SETTINGS.overspeed) / 20);
    damage(s, 'OVERSPEED', { leftWing: d, rightWing: d * 0.85, elevator: d, ailerons: d, rudder: d * 0.5, engine: d * 0.3 });
  }
  if (s.gExposure > 0.35) {
    const d = dt * 0.16 * Math.max(1, Math.abs(s.loadFactor) - 5);
    damage(s, 'EXCESSIVE G', { leftWing: d, rightWing: d * 0.9, elevator: d * 0.4, ailerons: d * 0.6 });
  }
  if (!s.grounded) {
    const { right, forward } = axes(s);
    const slip = Math.atan2(s.velocity.dot(right), Math.max(1, s.velocity.dot(forward)));
    // Differential wing lift rolls toward the failed wing, reversing under negative lift.
    const rollFailure = (h.leftWing - h.rightWing) * 0.8 * Math.min(3, a.speed * a.speed / 2500) * Math.tanh(s.loadFactor);
    rotateBody(s, -a.alpha * (a.stalled ? 1.25 : 0.35) * authority * h.elevator * dt,
      rollFailure * dt, -slip * 1.8 * authority * h.rudder * dt);
    a = aerodynamicForces(s);
  }
  const acceleration = new THREE.Vector3(0, -SETTINGS.gravity, 0)
    .add(a.lift).add(a.drag).add(a.side).addScaledVector(a.forward, s.throttle * 5.5 * h.engine);
  s.velocity.addScaledVector(acceleration, dt);
  if (s.grounded) {
    const friction = wheels ? 0.32 : 3.5;
    const horizontal = Math.hypot(s.velocity.x, s.velocity.z);
    const scale = Math.max(0, 1 - friction * dt / Math.max(horizontal, 1e-8));
    s.velocity.x *= scale; s.velocity.z *= scale;
  }
  s.position.addScaledVector(s.velocity, dt);
  const floor = terrainHeightAt(s.position.x, s.position.z) + (wheels ? 0.98 : 0.4);
  s.grounded = s.position.y <= floor;
  if (s.grounded) {
    const terrainRise = (terrainHeightAt(s.position.x, s.position.z) - oldTerrain) / dt;
    const impact = Math.max(0, terrainRise - s.velocity.y);
    const { up, right, forward } = axes(s);
    const badAttitude = up.y < 0.6 || Math.abs(forward.y) > 0.5;
    if ((!wasGrounded || impact > 4) && impact > 3.5) {
      const d = (impact - 3.5) / 12;
      damage(s, 'HARD IMPACT', { gear: d * 1.6, engine: d * 0.5, elevator: d * 0.5,
        leftWing: d * (right.y > 0 ? 1.5 : 0.7), rightWing: d * (right.y < 0 ? 1.5 : 0.7), ailerons: d, rudder: d * 0.4 });
    }
    if (!wheels && (!wasGrounded || s.velocity.length() > 2)) {
      const d = !wasGrounded ? 0.25 + impact / 20 : dt * 0.08 * Math.min(3, s.velocity.length() / 15);
      damage(s, 'BELLY / GEAR COLLAPSE', { engine: d, elevator: d * 0.5, ailerons: d * 0.4, leftWing: d * 0.3, rightWing: d * 0.3 });
    }
    if (impact > 14 || (badAttitude && s.velocity.length() > 10)) {
      s.crashed = true; s.damageCause = 'CRASH — SPACE TO RESET';
      for (const part of Object.keys(h)) h[part] = 0;
      s.velocity.set(0, 0, 0); s.throttle = 0;
    }
    s.position.y = floor;
    s.velocity.y = Math.max(0, s.velocity.y);
    // Only intact wheels settle a mild touchdown; damaged controls are never auto-levelled.
    if (wheels && !badAttitude && !s.crashed) {
      const heading = Math.atan2(-forward.x, -forward.z);
      const nose = THREE.MathUtils.clamp(Math.asin(THREE.MathUtils.clamp(forward.y, -1, 1)), 0, 0.20);
      const groundQ = new THREE.Quaternion().setFromEuler(new THREE.Euler(nose, heading, 0, 'YXZ'));
      s.quaternion.slerp(groundQ, (1 - Math.exp(-10 * dt)) * Math.min(h.gear, h.elevator, h.ailerons)).normalize();
    }
  }
  const final = aerodynamicForces(s);
  s.airspeed = final.speed; s.angleOfAttack = final.alpha; s.stalled = !s.grounded && final.stalled;
}
export function stepFlight(s, input = {}, dt, terrainHeightAt = () => 0) {
  if (!Number.isFinite(dt) || dt <= 0) return s;
  // Bound catch-up after tab suspension and integrate forces at <=120 Hz.
  const elapsed = Math.min(dt, 0.25), count = Math.ceil(elapsed / SETTINGS.step);
  for (let i = 0; i < count; i++) substep(s, input, elapsed / count, terrainHeightAt);
  return s;
}
export function attitude(s) {
  const { forward, right, up } = axes(s);
  return { pitch: Math.asin(THREE.MathUtils.clamp(forward.y, -1, 1)),
    roll: Math.atan2(right.y, up.y), heading: (Math.atan2(forward.x, -forward.z) + Math.PI * 2) % (Math.PI * 2) };
}
