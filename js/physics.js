import * as THREE from 'three';

const X = new THREE.Vector3(1, 0, 0), Y = new THREE.Vector3(0, 1, 0), Z = new THREE.Vector3(0, 0, 1);
export const SETTINGS = Object.freeze({ gravity: 9.81, pitchRate: 0.7, rollRate: 1.1, stallAngle: 0.30, step: 1 / 120 });
export function resetFlightState(s = {}) {
  s.position = new THREE.Vector3(0, 220, 0);
  s.quaternion = new THREE.Quaternion().setFromAxisAngle(X, 0.04);
  s.velocity = new THREE.Vector3(0, 0, -52);
  s.throttle = 0.62; s.gearDown = true; s.stalled = false; s.grounded = false;
  s.airspeed = 52; s.angleOfAttack = 0;
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
  const lift = liftDirection.multiplyScalar(0.0085 * speed * speed * cl);
  const cd = 0.024 + (s.gearDown ? 0.012 : 0) + 0.045 * cl * cl + (1 - retention) * 0.65;
  const drag = direction.clone().multiplyScalar(-0.0085 * speed * speed * cd);
  // Dissipative sideslip damping; coordinated yaw below follows, never redirects velocity.
  const side = right.multiplyScalar(-s.velocity.dot(right) * speed * 0.009);
  return { lift, drag, side, alpha, speed, stalled: !attached || speed < 25, forward };
}
function substep(s, input, dt, terrainHeightAt) {
  s.throttle = THREE.MathUtils.clamp(s.throttle + (input.throttle || 0) * dt * 0.28, 0, 1);
  const authority = THREE.MathUtils.clamp(s.velocity.length() / 35, 0.12, 1);
  const turn = input.roll || 0;
  // Ground roll inputs steer the nose; airborne roll inputs bank only.
  rotateBody(s, (input.pitch || 0) * SETTINGS.pitchRate * authority * dt,
    s.grounded ? 0 : turn * SETTINGS.rollRate * authority * dt,
    s.grounded ? turn * 0.65 * dt : 0);
  let a = aerodynamicForces(s);
  if (!s.grounded) {
    const { right, forward } = axes(s);
    const slip = Math.atan2(s.velocity.dot(right), Math.max(1, s.velocity.dot(forward)));
    // Positive sideslip is to the right, which requires negative local yaw.
    // Aerodynamic pitch stability unloads high AoA instead of forcing world-up lift.
    rotateBody(s, -a.alpha * (a.stalled ? 1.25 : 0.35) * authority * dt, 0, -slip * 1.8 * authority * dt);
    a = aerodynamicForces(s);
  }
  const acceleration = new THREE.Vector3(0, -SETTINGS.gravity, 0)
    .add(a.lift).add(a.drag).add(a.side).addScaledVector(a.forward, s.throttle * 5.5);
  s.velocity.addScaledVector(acceleration, dt);
  if (s.grounded) {
    const friction = s.gearDown ? 0.32 : 3.5;
    const horizontal = Math.hypot(s.velocity.x, s.velocity.z);
    const scale = Math.max(0, 1 - friction * dt / Math.max(horizontal, 1e-8));
    s.velocity.x *= scale; s.velocity.z *= scale;
  }
  s.position.addScaledVector(s.velocity, dt);
  const floor = terrainHeightAt(s.position.x, s.position.z) + (s.gearDown ? 0.98 : 0.4);
  s.grounded = s.position.y <= floor;
  if (s.grounded) {
    s.position.y = floor;
    s.velocity.y = Math.max(0, s.velocity.y);
    // Arcade contact: settle upright, retain heading and forward momentum.
    const f = axes(s).forward;
    const heading = Math.atan2(-f.x, -f.z);
    const nose = THREE.MathUtils.clamp(Math.asin(THREE.MathUtils.clamp(f.y, -1, 1)), 0, 0.20);
    const groundQ = new THREE.Quaternion().setFromEuler(new THREE.Euler(nose, heading, 0, 'YXZ'));
    s.quaternion.slerp(groundQ, 1 - Math.exp(-10 * dt)).normalize();
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
