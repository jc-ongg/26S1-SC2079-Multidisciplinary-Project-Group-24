<br />
<p align="center">
  <img src="/images/Map.png" alt="Logo" height=150 >
  <h1 align="center">
    MDP Simulator
  </h1>
</p>

## Setup

```bash
npm install
```

Start the app by

```bash
npm run dev
```

The app will be running at `localhost:3000`

The algorithm API must be running on `http://127.0.0.1:8000` to compute routes.

## Map editing and playback

- Select the image facing direction, then click a cell to add an obstacle, or enter X/Y coordinates and press **Add obstacle**.
- IDs increase sequentially and remain stable when another obstacle is removed. **Reset All** restarts numbering at 1.
- Duplicate cells and placements overlapping the robot footprint are rejected. Robot coordinates denote its centre.
- **Compute route** displays the planned route and numbered visit sequence. Obstacles omitted by the planner are listed separately.
- **Play**, **Pause**, **Previous**, and **Next** keep the robot, travelled trace, and completed visits synchronized. Turns are approximate curves between command endpoints, not physical telemetry.
- Editing the map clears the previous route. **Reset Robot** returns to the configured start and clears playback, preserving obstacles. **Reset All** also clears obstacles and restores the default start.
- **Turtle animation** uses the separate backend animation queue; the server needs Turtle enabled to display it.

## Verification

Run `npm test` for visit/trace helpers and `npm run build` for the production build.
With the app running and Playwright available to Node, run `node tests/simulator.browser.cjs`.
This browser check mocks `/path` responses to verify UI behavior independently of YOLO and hardware.
It covers placement, IDs, visit order, playback, resets, delayed/error responses, and desktop/mobile layouts.
Set `SIMULATOR_URL` to override the default `http://127.0.0.1:3000`, or `BROWSER_CHANNEL` to override Chrome.
Screenshots are saved under the ignored `test-results/` directory.
