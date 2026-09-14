import { useEffect, useRef, useState } from "react";
import QueryAPI from "./QueryAPI";
import { rotateDirection, routeSegments, snapshotVisits } from "./simulation.mjs";
import styles from "./Simulator.module.css";

const DIRECTIONS = { 0: "North", 2: "East", 4: "South", 6: "West" };
const ARROWS = { 0: "↑", 2: "→", 4: "↓", 6: "←" };
const INITIAL = { x: 1, y: 1, d: 0, s: -1 };
const insideRobot = (x, y, robot) => Math.abs(x - robot.x) <= 1 && Math.abs(y - robot.y) <= 1;

function DirectionSelect({ label, value, onChange }) {
  return <label>{label}<select value={value} onChange={(event) => onChange(Number(event.target.value))}>
    {Object.entries(DIRECTIONS).map(([value, name]) => <option key={value} value={value}>{ARROWS[value]} {name}</option>)}
  </select></label>;
}

export default function Simulator() {
  const [start, setStart] = useState(INITIAL);
  const [robotInput, setRobotInput] = useState(INITIAL);
  const [obstacles, setObstacles] = useState([]);
  const [obInput, setObInput] = useState({ x: 5, y: 5, d: 0 });
  const [path, setPath] = useState([]);
  const [commands, setCommands] = useState([]);
  const [page, setPage] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [computing, setComputing] = useState(false);
  const [timeTaken, setTimeTaken] = useState(null);
  const [error, setError] = useState("");
  const [notice, setNotice] = useState("");
  const nextId = useRef(1);
  const requestVersion = useRef(0);
  const robot = path[page] || start;
  const locked = computing || playing;
  const visits = snapshotVisits(commands);
  const segments = routeSegments(path);
  const movementCommands = commands.filter((command) => /^(FW|BW|FL|FR|BL|BR)/.test(command));
  const completedVisits = visits.filter((visit) => visit.step <= page);

  useEffect(() => {
    if (!playing) return;
    if (page >= path.length - 1) { setPlaying(false); return; }
    const timer = setTimeout(() => setPage((step) => step + 1), 500);
    return () => clearTimeout(timer);
  }, [playing, page, path.length]);

  useEffect(() => () => { requestVersion.current += 1; }, []);

  function clearRoute() {
    requestVersion.current++;
    setComputing(false);
    setPlaying(false);
    setPath([]);
    setCommands([]);
    setPage(0);
    setTimeTaken(null);
    setError("");
    setNotice("");
  }

  function addObstacle(x, y) {
    if (locked) return;
    if (![x, y].every((value) => value !== "" && Number.isInteger(Number(value)) && Number(value) >= 0 && Number(value) < 20)) {
      setError("Choose whole-number obstacle coordinates from 0 to 19."); return;
    }
    x = Number(x); y = Number(y);
    if (obstacles.some((ob) => ob.x === x && ob.y === y)) {
      setError(`There is already an obstacle at (${x}, ${y}).`); return;
    }
    if (insideRobot(x, y, start) || insideRobot(x, y, robot)) {
      setError("An obstacle cannot overlap the robot footprint."); return;
    }
    const id = nextId.current++;
    clearRoute();
    setObstacles((items) => [...items, { id, x, y, d: obInput.d }]);
    setObInput((input) => ({ ...input, x, y }));
    setNotice(`Added obstacle #${id} at (${x}, ${y}), facing ${DIRECTIONS[obInput.d]}.`);
  }

  function handleCellClick(x, y) {
    if (locked) return;
    const obstacle = obstacles.find((item) => item.x === x && item.y === y);
    if (!obstacle) {
      addObstacle(x, y);
      return;
    }
    const direction = rotateDirection(obstacle.d);
    clearRoute();
    setObstacles((items) => items.map((item) => item.id === obstacle.id ? { ...item, d: direction } : item));
    setNotice(`Rotated obstacle #${obstacle.id} clockwise to face ${DIRECTIONS[direction]}.`);
  }

  function setRobot() {
    const { x, y, d } = robotInput;
    if (![x, y].every((value) => value !== "" && Number.isInteger(Number(value)) && Number(value) >= 1 && Number(value) <= 18)) {
      setError("Choose whole-number robot coordinates from 1 to 18."); return;
    }
    const position = { x: Number(x), y: Number(y), d, s: -1 };
    if (obstacles.some((ob) => insideRobot(ob.x, ob.y, position))) {
      setError("The robot footprint overlaps an obstacle."); return;
    }
    clearRoute();
    setStart(position);
    setNotice("Robot start position updated.");
  }

  function compute() {
    if (locked || !obstacles.length) return;
    clearRoute();
    const version = requestVersion.current;
    const began = Date.now();
    setComputing(true);
    QueryAPI.query(obstacles, start.x, start.y, start.d, true, ({ data, error: apiError }) => {
      if (version !== requestVersion.current) return;
      setComputing(false);
      if (apiError || !Array.isArray(data?.path) || !data.path.length || !Array.isArray(data?.commands)) {
        setError(apiError ? `Route request failed: ${apiError.message}. Check the algorithm server on port 8000.` : "The planner did not return a route. Try adjusting the obstacle positions.");
        return;
      }
      setPath(data.path);
      setCommands(data.commands);
      setTimeTaken(((Date.now() - began) / 1000).toFixed(2));
      setNotice("Route ready. Press Play to follow the robot’s travel trace.");
    });
  }

  async function generateAnimation() {
    const version = requestVersion.current;
    try {
      const response = await fetch("http://localhost:8000/animate", {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ commands, obstacles, robotX: start.x, robotY: start.y, robotDir: start.d }),
      });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      if (version === requestVersion.current) setNotice("Animation queued. The algorithm server must have Turtle enabled to open its window.");
    } catch (err) { if (version === requestVersion.current) setError(`Could not queue the Turtle animation: ${err.message}`); }
  }

  return <main className={styles.simulator}>
    <header className={styles.header}>
      <div><p className={styles.eyebrow}>GROUP 24 · SC2079</p><h1>Algorithm simulator</h1><p>Build your arena. Plan the route. Follow every visit.</p></div>
      <span className={styles.status}>{computing ? "Computing route…" : playing ? "Simulation running" : path.length ? "Route ready" : "Map editor"}</span>
    </header>
    <div className={styles.layout}>
      <section className={styles.panel} aria-label="Arena and playback">
        <div className={styles.panelHeading}><div><h2>Arena</h2><p>20 × 20 cells · origin at bottom left</p></div><span className={styles.counter}>{obstacles.length} obstacles</span></div>
        <p className={styles.hint}>Choose a facing direction, then click an empty cell to add an obstacle. Click an obstacle to rotate it clockwise. Editing the map clears the previous route.</p>
        <div className={styles.mapScroll}>
          <div className={styles.mapFrame}>
            <div className={styles.yAxis}>{Array.from({ length: 20 }, (_, row) => <span key={row}>{19 - row}</span>)}</div>
            <div className={styles.map}>
              <div className={styles.cells}>
                {Array.from({ length: 400 }, (_, index) => {
                  const x = index % 20, y = 19 - Math.floor(index / 20);
                  const obstacle = obstacles.find((ob) => ob.x === x && ob.y === y);
                  const visited = obstacle && completedVisits.some((visit) => visit.id === obstacle.id);
                  return <button key={index} type="button" disabled={locked}
                    aria-label={obstacle ? `Obstacle #${obstacle.id} at (${x}, ${y}), facing ${DIRECTIONS[obstacle.d]}` : `Add obstacle at (${x}, ${y})`}
                    title={obstacle ? `#${obstacle.id} · ${DIRECTIONS[obstacle.d]}` : `(${x}, ${y})`}
                    onClick={() => handleCellClick(x, y)}
                    className={`${styles.cell} ${x < 4 && y < 4 ? styles.startZone : ""} ${obstacle ? styles.obstacle : ""} ${visited ? styles.visited : ""}`}
                    data-facing={obstacle?.d}>
                    {obstacle && <span>{obstacle.id}</span>}
                  </button>;
                })}
              </div>
              <svg className={styles.overlay} viewBox="0 0 20 20" aria-label="Planned route and travelled trace" role="img">
                <g fill="none" strokeLinecap="round" strokeLinejoin="round">
                  {segments.map((segment, i) => <path key={`plan-${i}`} d={segment} className={styles.planned} />)}
                  {segments.slice(0, page).map((segment, i) => <path key={`trace-${i}`} d={segment} className={styles.trace} />)}
                </g>
                {visits.filter((visit) => path[visit.step]).map((visit, i) => {
                  const pose = path[visit.step];
                  return <circle key={i} cx={pose.x + .5} cy={19.5 - pose.y} r=".15" fill={visit.step <= page ? "#0f766e" : "#94a3b8"} />;
                })}
                <g transform={`translate(${robot.x + .5} ${19.5 - robot.y}) rotate(${Number(robot.d) * 45})`}>
                  <rect x="-1.45" y="-1.45" width="2.9" height="2.9" rx=".25" fill="#2563eb" fillOpacity=".22" stroke="#2563eb" strokeWidth=".07" />
                  <circle r=".37" fill="#1d4ed8" />
                  <path d="M 0 -.24 L 0 -1.12 M -.24 -.86 L 0 -1.12 L .24 -.86" stroke="white" strokeWidth=".12" fill="none" strokeLinecap="round" />
                </g>
              </svg>
            </div>
            <div className={styles.xAxis}>{Array.from({ length: 20 }, (_, x) => <span key={x}>{x}</span>)}</div>
          </div>
        </div>
        <div className={styles.legend}><span><i className={styles.plannedKey} /> Planned</span><span><i className={styles.traceKey} /> Travelled</span><span><i className={styles.robotKey} /> Robot</span><span><i className={styles.startKey} /> Start zone</span><span><i className={styles.faceKey} /> Image face</span></div>
        <p className={styles.finePrint}>Trace follows simulated command endpoints. Curved turns are approximate, not measured robot telemetry.</p>
        <div className={styles.playback}>
          <div className={styles.panelHeading}><h2>Playback</h2><span>({robot.x}, {robot.y}) · {DIRECTIONS[robot.d]}</span></div>
          <div className={styles.actions}>
            <button disabled={!path.length || locked || page === 0} onClick={() => setPage(page - 1)}>← Previous</button>
            <button className={styles.primary} disabled={!path.length || computing} onClick={() => {
              if (playing) { setPlaying(false); return; }
              if (page >= path.length - 1) setPage(0);
              setPlaying(true);
            }}>{playing ? "Pause" : page > 0 && page >= path.length - 1 ? "Replay" : "Play"}</button>
            <button disabled={!path.length || locked || page >= path.length - 1} onClick={() => setPage(page + 1)}>Next →</button>
          </div>
          <div className={styles.progress}><span style={{ width: `${path.length > 1 ? page / (path.length - 1) * 100 : 0}%` }} /></div>
          <div className={styles.panelHeading}><span>Movement {page} / {Math.max(0, path.length - 1)}</span><code>{page ? movementCommands[page - 1] : "START"}</code></div>
        </div>
      </section>
      <aside className={styles.sidebar}>
        <section className={styles.panel}><h2>Place obstacles</h2><p>Each obstacle gets a permanent number until Reset All.</p>
          <fieldset disabled={locked}>
            <DirectionSelect label="Image facing direction" value={obInput.d} onChange={(d) => setObInput({ ...obInput, d })} />
            <div className={styles.inputRow}>{["x", "y"].map((axis) => <label key={axis}>{axis.toUpperCase()}<input type="number" min="0" max="19" value={obInput[axis]} onChange={(event) => setObInput({ ...obInput, [axis]: event.target.value })} aria-label={`Obstacle ${axis.toUpperCase()}`} /></label>)}
              <button onClick={() => addObstacle(obInput.x, obInput.y)}>Add obstacle</button></div>
          </fieldset>
          <ul className={styles.obstacleList}>{obstacles.map((ob) => {
            const rank = visits.findIndex((visit) => visit.id === ob.id);
            const done = completedVisits.some((visit) => visit.id === ob.id);
            return <li key={ob.id}><span className={`${styles.number} ${done ? styles.doneNumber : ""}`}>#{ob.id}</span><div><strong>({ob.x}, {ob.y}) {ARROWS[ob.d]}</strong><small>{path.length ? rank < 0 ? "Not in route" : `${done ? "Visited" : "Visit"} ${rank + 1}` : DIRECTIONS[ob.d]}</small></div><button disabled={locked} aria-label={`Remove obstacle ${ob.id}`} onClick={() => { clearRoute(); setObstacles(obstacles.filter((item) => item.id !== ob.id)); }}>×</button></li>;
          })}</ul>
          {!obstacles.length && <div className={styles.empty}>Click the map to place your first obstacle.</div>}
        </section>
        <section className={styles.panel}><h2>Robot start</h2><fieldset disabled={locked}>
          <div className={styles.inputRow}>{["x", "y"].map((axis) => <label key={axis}>{axis.toUpperCase()}<input aria-label={`Robot ${axis.toUpperCase()}`} type="number" min="1" max="18" value={robotInput[axis]} onChange={(event) => setRobotInput({ ...robotInput, [axis]: event.target.value })} /></label>)}<button onClick={setRobot}>Set robot</button></div>
          <DirectionSelect label="Robot facing direction" value={robotInput.d} onChange={(d) => setRobotInput({ ...robotInput, d })} />
        </fieldset></section>
        <section className={styles.panel}><div className={styles.panelHeading}><h2>Visit order</h2><span>{completedVisits.length} / {visits.length}</span></div>
          {path.length ? <><ol className={styles.visitOrder}><li>Start</li>{visits.map((visit, i) => <li key={i} className={visit.step <= page ? styles.visitDone : ""} aria-label={`Visit ${i + 1}: obstacle ${visit.id}${visit.step <= page ? ", completed" : ""}`}><span aria-hidden="true">→ </span><small>{i + 1}.</small> #{visit.id}{visit.step <= page ? " ✓" : ""}</li>)}</ol>
            {obstacles.some((ob) => !visits.some((visit) => visit.id === ob.id)) && <p className={styles.omitted}>Not in route: {obstacles.filter((ob) => !visits.some((visit) => visit.id === ob.id)).map((ob) => `#${ob.id}`).join(", ")}</p>}
          </> : <p className={styles.empty}>Compute a route to see the planned visit sequence.</p>}
          <div className={styles.actions}><button className={styles.primary} disabled={locked || !obstacles.length} onClick={compute}>{computing ? "Computing…" : "Compute route"}</button><button disabled={!path.length || locked} onClick={generateAnimation}>Turtle animation</button></div>
          {timeTaken !== null && <p className={styles.finePrint}>Computed in {timeTaken} seconds.</p>}
        </section>
        <div className={styles.actions}><button onClick={() => { clearRoute(); setRobotInput(start); }}>Reset Robot</button><button className={styles.danger} onClick={() => { clearRoute(); setObstacles([]); nextId.current = 1; setStart(INITIAL); setRobotInput(INITIAL); }}>Reset All</button></div>
        {error && <div role="alert" className={styles.error}>{error}</div>}
        <div role="status" aria-live="polite" className={styles.notice}>{notice}</div>
      </aside>
    </div>
  </main>;
}
