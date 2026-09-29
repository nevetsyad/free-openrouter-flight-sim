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

Bank then pull up to turn. Bank remains until counteracted; there is no automatic leveling. In a stall, release the pull or lower the nose and add power. Momentum may briefly carry a stalled aircraft upward, but gravity remains active and lift falls sharply. For takeoff, lower gear, add power, accelerate, and gently pull. Land gently with gear down; damage persists until Space resets. Watch component health, G, stall/overspeed warnings and the last damage cause on the HUD. Damaged wings/tail darken and disappear at failure; collapsed gear disappears.

## Run and test

Serve the repository over HTTP (ES modules should not be opened with `file://`):

```sh
python3 -m http.server 8000
# Browse http://localhost:8000
npm ci
npm test
```

The browser imports Three.js 0.160.0 from jsDelivr. Tests use the same pinned Three.js version and the **same `js/physics.js` module** as the browser, not a duplicate implementation. Seventeen deterministic Node tests execute pitch signs, signed AoA, perpendicular lift/dissipative drag, stall lift loss, both banked pull directions, low-speed altitude loss and recovery, complete quaternion rotations and sustained-control loops/rolls, landing/taxi/takeoff, gear contact, reset, timestep bounds, and frame partition consistency, plus component damage, exposure timers, asymmetric wing failure, dynamic-pressure/post-stall controls, dead engines, crash lockout and damage reset.

## Physics and limitations

Velocity is the source of truth: gravity, thrust, lift and drag are integrated in SI units with substeps no larger than 1/120 second. Lift is perpendicular to airflow, signed angle of attack determines its coefficient, and exceeding the stall angle reduces lift and increases drag. Body-local quaternion controls use -Z forward/+Y up: positive local X is nose-up. Passive pitch stability and yaw alignment make flight approachable without directly snapping velocity to the nose.

This is an arcade model, not training software: no wind, prop torque, angular inertia, wheel suspension, or slope-normal ground dynamics. Damage is a simplified gameplay model, not aircraft-certified realism. Ground collision samples aircraft-center terrain height; terrain visuals are a finite tessellated mesh. Large frame delays are capped at 0.25 seconds rather than fully caught up. HUD pitch is nose elevation (±90°), so it folds over naturally during loops while the actual quaternion remains continuous. Aerobatics require adequate speed/altitude. Browser WebGL/CDN availability is separate from physics tests.

## v7 damage model — gameplay tuning

- Left/right wing, elevator, aileron, rudder, engine and gear health range from 0–100%. Average wing health scales lift; differential lift rolls toward the weaker wing (opposite under negative lift). Damage adds drag. Engine health scales thrust; surface health scales pilot controls and passive pitch/yaw stability.
- Aerodynamic control authority scales with speed squared up to 35 m/s, falls further in separated/post-stall airflow, and is exactly zero at zero airspeed. This is kinematic arcade control, not torque/inertia simulation. Post-stall authority retains 22% of its dynamic-pressure-limited value to permit recovery; that is not a minimum at zero speed.
- Gear-down touchdowns at ≤3.5 m/s relative terrain closure cause no impact damage. Harder contact progressively damages gear, wings and controls. Gear health ≤20% collapses the wheels. Belly contact damages the engine and surfaces immediately, then continues abrasion while moving.
- Relative terrain closure >14 m/s, or terrain contact above 10 m/s while inverted/heavily banked or steeply pitched, latches a crash. Crashed aircraft cannot move, thrust or take off until Space resets. Contact is still sampled at the aircraft center, not individual wing tips; terrain slope enters closure speed but there is no slope-normal rigid-body collision.
- Speed >100 m/s (194 kt) for >0.75 s progressively damages the structure/controls/engine. Signed aerodynamic load above +6 G or below −3 G for >0.35 s damages wings and controls. Both exposure timers drain at twice real time below their limits. These generous limits preserve ordinary aerobatics; they are not limits for any real aircraft.
- Intact wheels settle mild ground contact; settling is scaled by surviving gear/elevator/aileron health. Taxi steering requires working gear and rudder, and forward speed. There is no ground-control shortcut for failed controls. The rudder is passive/ground-steered, not a new keyboard axis.
- All health, exposures and crash state reset. No repairs, debris, fire or multi-point collision simulation are implemented.

## Roadmap

- Environment details and sound
- Multiplayer
- Optional OpenRouter weather/ATC integration

## License

MIT
