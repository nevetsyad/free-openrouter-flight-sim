# Free OpenRouter Flight Sim

A browser-based flight simulator using Three.js, designed to be free and accessible from anywhere.

## Overview

This project provides a lightweight, web-based flight simulator that runs directly in the browser using Three.js for 3D rendering. It's designed to be hosted on GitHub Pages for easy access.

## Features

- **3D Skybox** with dynamic sky colors
- **Terrain** with height-based coloring
- **Player aircraft** with realistic-ish flight physics
- **Keyboard controls** (arrow keys for pitch/yaw, W/S for throttle)
- **Camera views** (cockpit/third-person toggle)

## Controls

| Key | Action |
|-----|--------|
| ↑ / ↓ | Pitch up / down |
| ← / → | Roll left / right |
| W | Increase throttle |
| S | Decrease throttle |
| C | Toggle camera view |
| Space | Reset view |

## Setup

```bash
git clone https://github.com/nevetsyad/free-openrouter-flight-sim.git
cd free-openrouter-flight-sim
# Open index.html in a browser, or serve locally:
npx serve .
```

## Roadmap

- [x] Basic 3D scene with skybox
- [x] Player aircraft model
- [x] Flight physics (pitch, yaw, throttle)
- [ ] Add environment details (trees, buildings)
- [ ] Add sound effects
- [ ] Add multiplayer support
- [ ] Integrate OpenRouter AI for dynamic weather/ATC

## License

MIT
