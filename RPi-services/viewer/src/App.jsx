import { useCallback, useEffect, useRef, useState } from 'react'
import './App.css'

const STREAM_URL = '/stream/video_feed'

// Inference measured at ~350-400ms on this machine, so anything under ~300ms
// just means the loop runs back-to-back with no idle gap.
const INTERVALS = [0, 200, 500, 1000, 2000]

// Three possible shapes, normalised into one array so the rest of the app
// doesn't care:
//   Algo /infer:  { ok, n, detections: [{ cls_id, cls_name, conf, xyxy }] }
//   image-api react mode:  { results: [{ image_id, obstacle_id, conf, xyxy }] }
//   image-api legacy mode: a single best guess { image_id, conf, xyxy }
function normalise(data) {
  if (!data) return []
  // The bullseye marks an obstacle face with no target image. It is reported
  // alongside the target rather than as the image_id, so add it as its own box
  // — useful for A.5, where "obstacle but wrong face" must be distinguishable
  // from "nothing in view".
  const extras = []
  if (data.bullseye && Array.isArray(data.bullseye.xyxy)) {
    extras.push({
      image_id: 'BULLSEYE',
      label: 'marker',
      conf: data.bullseye.conf,
      xyxy: data.bullseye.xyxy,
    })
  }
  if (Array.isArray(data.detections)) {
    // cls_id is the mapped image number the checklist asks us to label with;
    // cls_name is the human-readable symbol (A, B, stop, ...).
    return [...extras, ...data.detections.map((d) => ({
      image_id: d.cls_name || String(d.cls_id),
      obstacle_id: d.cls_id,
      conf: d.conf,
      xyxy: d.xyxy,
    }))]
  }
  if (Array.isArray(data.results)) return [...extras, ...data.results]
  if (data.image_id && data.image_id !== 'NA') return [...extras, data]
  return extras
}

export default function App() {
  const imgRef = useRef(null)
  const canvasRef = useRef(null)

  // Loop control lives in refs: the detect loop is self-rescheduling, so reading
  // these from state would capture stale values in the closure.
  const liveRef = useRef(false)
  const inFlightRef = useRef(false)
  const timerRef = useRef(null)
  const intervalRef = useRef(0)
  const framesRef = useRef([])

  const [health, setHealth] = useState(null)
  const [streamUp, setStreamUp] = useState(false)
  const [live, setLive] = useState(false)
  const [minInterval, setMinInterval] = useState(0)
  const [detections, setDetections] = useState([])
  const [raw, setRaw] = useState(null)
  const [error, setError] = useState(null)
  const [history, setHistory] = useState([])
  const [latency, setLatency] = useState(null)
  const [fps, setFps] = useState(0)
  const [frameSize, setFrameSize] = useState(null)

  useEffect(() => { intervalRef.current = minInterval }, [minInterval])

  // Poll image-api health so the header reflects whether it's reachable.
  // Check the status code before parsing: a miss returns an HTML error page,
  // and r.json() would throw the opaque "Unexpected token '<'".
  useEffect(() => {
    let cancelled = false
    const check = async () => {
      try {
        const r = await fetch('/api/health')
        if (!r.ok) throw new Error(`status ${r.status}`)
        const j = await r.json()
        if (!cancelled) setHealth(j)
      } catch {
        if (!cancelled) setHealth(null)
      }
    }
    check()
    const id = setInterval(check, 5000)
    return () => { cancelled = true; clearInterval(id) }
  }, [])

  // One capture -> POST -> render cycle. Returns how long it took.
  const detectOnce = useCallback(async () => {
    const img = imgRef.current
    if (!img || !img.naturalWidth) throw new Error('Stream not ready — no frame available.')

    const started = performance.now()

    // Freeze the current MJPEG frame. Same-origin via the Vite proxy, so the
    // canvas stays untainted and toBlob() is allowed.
    const canvas = canvasRef.current
    canvas.width = img.naturalWidth
    canvas.height = img.naturalHeight
    canvas.getContext('2d').drawImage(img, 0, 0)
    setFrameSize({ w: img.naturalWidth, h: img.naturalHeight })

    const blob = await new Promise((res) => canvas.toBlob(res, 'image/jpeg', 0.8))
    if (!blob) throw new Error('Could not encode frame as JPEG')

    // image-api's /predict looks for a field called "image".
    const body = new FormData()
    body.append('image', blob, 'frame.jpg')

    const res = await fetch('/api/predict', { method: 'POST', body })
    // A wrong path returns an HTML error page, and res.json() then throws the
    // opaque "Unexpected token '<'". Fail with something readable instead.
    const ct = res.headers.get('content-type') || ''
    if (!ct.includes('application/json')) {
      throw new Error(`API returned ${res.status} (${ct.split(';')[0] || 'no type'}) — expected JSON`)
    }
    const data = await res.json()

    const took = Math.round(performance.now() - started)
    setLatency(took)
    setRaw(data)

    const found = normalise(data)
    setDetections(found)

    if (found.length) {
      setHistory((h) => {
        // Collapse consecutive repeats of the same symbol so the log stays readable
        // while running continuously.
        const top = found[0]
        if (h[0] && h[0].image_id === top.image_id) {
          const [, ...rest] = h
          return [{ ...top, at: new Date().toLocaleTimeString(), count: (h[0].count || 1) + 1 }, ...rest]
        }
        return [{ ...top, at: new Date().toLocaleTimeString(), count: 1 }, ...h].slice(0, 25)
      })
    }

    if (!res.ok && !found.length) {
      throw new Error(data.details || data.error || `API returned ${res.status}`)
    }

    // Rolling FPS over the last 10 completed cycles.
    const now = performance.now()
    framesRef.current = [...framesRef.current, now].slice(-10)
    const stamps = framesRef.current
    if (stamps.length > 1) {
      const span = (stamps[stamps.length - 1] - stamps[0]) / 1000
      setFps(span > 0 ? (stamps.length - 1) / span : 0)
    }

    return took
  }, [])

  // Self-pacing loop: schedule the next run only once the previous one settles,
  // so slow inference can never pile up a backlog of requests.
  const runLoop = useCallback(async () => {
    if (!liveRef.current || inFlightRef.current) return
    inFlightRef.current = true

    let took = 0
    try {
      took = await detectOnce()
      setError(null)
    } catch (e) {
      setError(e.message)
      setDetections([])
    } finally {
      inFlightRef.current = false
    }

    if (liveRef.current) {
      const wait = Math.max(0, intervalRef.current - took)
      timerRef.current = setTimeout(runLoop, wait)
    }
  }, [detectOnce])

  const toggleLive = useCallback((on) => {
    setLive(on)
    liveRef.current = on
    if (on) {
      framesRef.current = []
      runLoop()
    } else {
      clearTimeout(timerRef.current)
      setFps(0)
    }
  }, [runLoop])

  // Stop the loop on unmount.
  useEffect(() => () => {
    liveRef.current = false
    clearTimeout(timerRef.current)
  }, [])

  const single = useCallback(async () => {
    try {
      await detectOnce()
      setError(null)
    } catch (e) {
      setError(e.message)
      setDetections([])
    }
  }, [detectOnce])

  const best = detections[0]

  return (
    <div className="app">
      <header>
        <h1>MDP Group 24 — Vision</h1>
        <div className="pills">
          <span className={`pill ${streamUp ? 'ok' : 'bad'}`}>
            stream {streamUp ? 'live' : 'down'}
          </span>
          <span className={`pill ${health ? 'ok' : 'bad'}`}>
            api {health ? health.mode?.toLowerCase() ?? 'up' : 'down'}
          </span>
          {health?.model && <span className="pill muted">model {health.model.split(/[\\/]/).pop()}</span>}
          {live && <span className="pill ok">{fps.toFixed(1)} fps</span>}
          {latency != null && <span className="pill muted">{latency} ms</span>}
        </div>
      </header>

      <section className="panel">
        <h2>
          Live detection
          {frameSize && <span className="timing">{frameSize.w}×{frameSize.h}</span>}
        </h2>

        {/* Boxes are drawn straight onto the live stream, so detections track the
            picture instead of a frozen snapshot. */}
        <div className="frame">
          <img
            ref={imgRef}
            src={STREAM_URL}
            alt="Pi camera stream"
            crossOrigin="anonymous"
            onLoad={() => setStreamUp(true)}
            onError={() => setStreamUp(false)}
          />
          {!streamUp && <div className="overlay">waiting for stream…</div>}

          {frameSize && detections.map((d, i) => {
            const [x1, y1, x2, y2] = d.xyxy ?? []
            if (x1 == null) return null
            const { w: W, h: H } = frameSize
            return (
              <div
                key={i}
                className="bbox"
                style={{
                  left: `${(x1 / W) * 100}%`,
                  top: `${(y1 / H) * 100}%`,
                  width: `${((x2 - x1) / W) * 100}%`,
                  height: `${((y2 - y1) / H) * 100}%`,
                }}
              >
                <span>
                  {d.image_id}{d.label ? ` (${d.label})` : ''} {(d.conf * 100).toFixed(0)}%
                </span>
              </div>
            )
          })}

          {live && <div className="rec">● LIVE</div>}
        </div>

        <div className="controls">
          <button className={live ? 'stop' : ''} onClick={() => toggleLive(!live)}>
            {live ? 'Stop' : 'Start live detection'}
          </button>
          <button className="ghost" onClick={single} disabled={live}>
            Single shot
          </button>
          <label className="toggle">
            min interval
            <select value={minInterval} onChange={(e) => setMinInterval(Number(e.target.value))}>
              {INTERVALS.map((ms) => (
                <option key={ms} value={ms}>{ms === 0 ? 'as fast as possible' : `${ms} ms`}</option>
              ))}
            </select>
          </label>
        </div>

        {error && <p className="error">{error}</p>}
      </section>

      <div className="grid">
        <section className="panel">
          <h2>Current</h2>
          {best ? (
            <div className="result">
              <div className="big">{best.image_id}</div>
              <dl>
                <div><dt>image id</dt><dd>{best.obstacle_id ?? '—'}</dd></div>
                <div><dt>confidence</dt><dd>{(best.conf * 100).toFixed(1)}%</dd></div>
                <div><dt>boxes</dt><dd>{detections.length}</dd></div>
              </dl>
            </div>
          ) : (
            <p className="muted">{raw ? 'No symbol in frame.' : 'Not started.'}</p>
          )}
          {raw && (
            <details>
              <summary>raw response</summary>
              <pre>{JSON.stringify(raw, null, 2)}</pre>
            </details>
          )}
        </section>

        <section className="panel">
          <h2>Recent detections</h2>
          {history.length === 0 ? (
            <p className="muted">Nothing detected yet.</p>
          ) : (
            <table>
              <thead>
                <tr><th>time</th><th>symbol</th><th>id</th><th>conf</th><th>×</th></tr>
              </thead>
              <tbody>
                {history.map((h, i) => (
                  <tr key={i}>
                    <td className="muted">{h.at}</td>
                    <td><strong>{h.image_id}</strong></td>
                    <td>{h.obstacle_id ?? '—'}</td>
                    <td>{(h.conf * 100).toFixed(1)}%</td>
                    <td className="muted">{h.count > 1 ? h.count : ''}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          )}
        </section>
      </div>

      <canvas ref={canvasRef} style={{ display: 'none' }} />
    </div>
  )
}
