# Free OpenRouter Flight Sim

A free browser-based arcade flight simulator with Three.js, procedural terrain, sky, retractable gear, and cockpit/chase cameras. Static GitHub Pages compatible; no build step.

## Controls

| Key | Action |
|-----|--------|
| ↑ / ↓ | Body-local pitch up / down; hold for loops |
| ← / → | Bank left / right; steer while on the ground |
| W / S | Increase / decrease throttle |
| G | Toggle landing gear |
| C | Toggle cockpit/chase camera |
| Space | Reset aircraft, velocity, throttle, gear and camera |

Bank then pull up to turn. Bank remains until counteracted; there is no automatic leveling. In a stall, release the pull or lower the nose and add power. Momentum may briefly carry a stalled aircraft upward, but gravity remains active and lift falls sharply. For takeoff, lower gear, add power, accelerate, and gently pull. Contact with terrain is forgiving rather than a crash simulation.

## Run and test

Serve the repository over HTTP (ES modules should not be opened with `file://`):

```sh
python3 -m http.server 8000
# Browse http://localhost:8000
npm ci
npm test
```

The browser imports Three.js 0.160.0 from jsDelivr. Tests use the same pinned Three.js version and the **same `js/physics.js` module** as the browser, not a duplicate implementation. Nine deterministic Node tests execute pitch signs, signed AoA, perpendicular lift/dissipative drag, stall lift loss, both banked pull directions, low-speed altitude loss and recovery, complete quaternion rotations and sustained-control loops/rolls, landing/taxi/takeoff, gear contact, reset, timestep bounds, and frame partition consistency.

## Physics and limitations

Velocity is the source of truth: gravity, thrust, lift and drag are integrated in SI units with substeps no larger than 1/120 second. Lift is perpendicular to airflow, signed angle of attack determines its coefficient, and exceeding the stall angle reduces lift and increases drag. Body-local quaternion controls use -Z forward/+Y up: positive local X is nose-up. Passive pitch stability and yaw alignment make flight approachable without directly snapping velocity to the nose.

This is an arcade model, not training software: no wind, prop torque, angular inertia, structural limits, damage, wheel suspension, or slope-normal ground dynamics. Ground collision samples aircraft-center terrain height; terrain visuals are a finite tessellated mesh. Large frame delays are capped at 0.25 seconds rather than fully caught up. HUD pitch is nose elevation (±90°), so it folds over naturally during loops while the actual quaternion remains continuous. Aerobatics require adequate speed/altitude. Browser WebGL/CDN availability is separate from physics tests.

## Roadmap

- Environment details and sound
- Multiplayer
- Optional OpenRouter weather/ATC integration

## License

MIT
