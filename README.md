# Paper Flight

Fold a paper plane, launch it, and see how far it flies. Try different designs, paper weights, and wind settings, then replay the flight to see how the plane bends in the air.

**[Play Paper Flight](https://rossng.github.io/paper-flight/)** — runs in your browser, with no account or download needed.

## How to play

1. Choose a plane and paper type. Adjust the sheet size and wings to experiment.
2. Use **Next fold** to fold one step at a time, or **Fold plane** to finish. Drag to look around and scroll or pinch to zoom.
3. Go to the launcher. Set the nose angle, release height, and weather, then pull the plane back, aim, and release.
4. After landing, retry or choose **Explore flight** to watch the replay. Open **Inspect** to see bending, paper layers, and lift.

After touchdown, the plane can slide, flex and settle on the textured ground. Replay includes the landing, with separate jumps to touchdown and the end. Distance and airtime stop at first contact; the result also shows ground travel.

Your best distance is saved in your browser when storage is available.

To launch with the keyboard, focus the plane and use the arrow keys to aim and change the pull. Press Space or Enter to launch. Use the pitch dial or its −/+ buttons to change the nose angle.

## Run locally

You need Node.js 22.12 or newer and a modern browser with WebGL 2 support. The project uses Node 24 for automated checks.

From the project folder:

```sh
npm ci
npm run dev
```

Open the address printed in the terminal, usually `http://localhost:5173`. All simulation runs in the browser; no backend or API keys are needed.

## Development

```sh
npm test           # Run the focused tests
npm run test:full  # Include full-flight checks (used by CI)
npm run build      # Check types and build the site into dist/
npm run preview    # Preview the built site locally
```

Run `npm run test:full` when changing physics or folding.

For browser UI checks, start Vite and an isolated Chrome instance with remote
debugging enabled (see [browser setup](docs/performance.md)), then run
`npm run test:browser-ui -- --cdp 9297 --url http://127.0.0.1:5197/` against the dev server.
This checks responsive fold instructions, all ten designs, picker keyboard controls
and opening times, idle rendering, animated folds, and a complete flight/replay cycle.

The [GitHub Pages workflow](.github/workflows/pages.yml) runs tests and builds the site for pull requests and pushes to `main`. Successful builds on `main` are deployed automatically.

## About the simulation

Gravity, air forces, and the paper's flexibility determine the flight; its path is not scripted. Folding follows preset steps.

This is an experiment, and its materials and flight distances have not been checked against real throws.

Technical notes and findings live in [`docs/`](docs/):

- [Physics model and limits](docs/physics-upgrade.md)
- [Physics tests and results](docs/physics-validation.md)
- [Performance report](docs/performance.md)
- [Real-time solver](docs/realtime-solver.md)
- [Plane library and downloads](docs/plane-library.md)
- [Folding model and limits](docs/custom-folding.md)

## License and credits

Copyright (c) 2026 Paper Flight Lab contributors. Project code is licensed under the [Blue Oak Model License 1.0.0](LICENSE). Third-party components keep their own licenses; see [third-party notices](THIRD_PARTY.md).
