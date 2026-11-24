import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  chartPoints, checkReplay, clampFrame, decodePoints, describeFrame, frameAtX, notableFrames, parseReplayQuery,
  replayQuery, SEQUENCES, type ReplayFrames, type Seq,
} from "../lib/replay";
import { ReplayView, type CameraMode, type Layers } from "./ReplayView";

interface Drive { frames: ReplayFrames; map: Float32Array; scans: ArrayBuffer }

const DRIVES: Record<Seq, string> = { "07": "694 m city loop", "04": "394 m country road" };
const base = `${import.meta.env.BASE_URL}replay/`;

async function fetchBuffer(url: string, onBytes: (n: number) => void): Promise<ArrayBuffer> {
  const res = await fetch(url);
  if (!res.ok || !res.body) throw new Error(`${url}: HTTP ${res.status}`);
  const reader = res.body.getReader();
  const parts: Uint8Array[] = [];
  let n = 0;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    parts.push(value);
    n += value.byteLength;
    onBytes(value.byteLength);
  }
  const out = new Uint8Array(n);
  let at = 0;
  for (const p of parts) { out.set(p, at); at += p.byteLength; }
  return out.buffer;
}

async function loadDrive(seq: Seq, onProgress: (fraction: number) => void): Promise<Drive> {
  const res = await fetch(`${base}${seq}/frames.json`);
  if (!res.ok) throw new Error(`frames.json: HTTP ${res.status}`);
  const frames = (await res.json()) as ReplayFrames;
  const total = frames.mapPoints * 6 + frames.frames * frames.scanPoints * 6;
  let got = 0;
  const tick = (n: number) => { got += n; onProgress(Math.min(1, got / total)); };
  const [mapBuf, scans] = await Promise.all([fetchBuffer(`${base}${seq}/map.bin`, tick), fetchBuffer(`${base}${seq}/scans.bin`, tick)]);
  const problem = checkReplay(frames, mapBuf.byteLength, scans.byteLength);
  if (problem) throw new Error(problem);
  return { frames, map: decodePoints(mapBuf, 10), scans };
}

const fmt = (v: number, digits: number) => v.toLocaleString("en-US", { minimumFractionDigits: digits, maximumFractionDigits: digits });

export function ReplayApp() {
  const initial = useMemo(() => parseReplayQuery(window.location.search), []);
  const [seq, setSeq] = useState<Seq>(initial.seq);
  const [drive, setDrive] = useState<Drive | null>(null);
  const [progress, setProgress] = useState(0);
  const [error, setError] = useState<string | null>(null);
  const [frame, setFrame] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState<1 | 4>(1);
  const [mode, setMode] = useState<CameraMode>("chase");
  const [layers, setLayers] = useState<Layers>({ map: true, scan: true, truth: true, kiss: false });
  const [viewError, setViewError] = useState<string | null>(null);
  const host = useRef<HTMLDivElement>(null);
  const view = useRef<ReplayView | null>(null);
  const wantFrame = useRef<number | null>(initial.frame);

  // The 3D view lives as long as the page.
  useEffect(() => {
    if (!host.current) return;
    try {
      view.current = new ReplayView(host.current);
    } catch (e) {
      setViewError(`The 3D view needs WebGL, which this browser did not provide (${(e as Error).message}).`);
    }
    return () => { view.current?.dispose(); view.current = null; };
  }, []);

  // Load a drive whenever the sequence changes.
  useEffect(() => {
    let cancelled = false;
    setDrive(null); setError(null); setProgress(0); setPlaying(false);
    loadDrive(seq, (p) => { if (!cancelled) setProgress(p); })
      .then((d) => {
        if (cancelled) return;
        view.current?.setDrive(d.frames, d.map, d.scans);
        const start = wantFrame.current !== null ? clampFrame(wantFrame.current, d.frames.frames) : 0;
        wantFrame.current = null;
        setDrive(d);
        setFrame(start);
      })
      .catch((e: Error) => { if (!cancelled) setError(e.message); });
    return () => { cancelled = true; };
  }, [seq]);

  useEffect(() => { if (drive) view.current?.setFrame(frame); }, [drive, frame]);
  useEffect(() => { view.current?.setMode(mode); }, [mode]);
  useEffect(() => { view.current?.setLayers(layers); }, [layers]);

  // Keep the address shareable: ?seq=07&frame=476.
  useEffect(() => {
    if (!drive) return;
    const t = setTimeout(() => window.history.replaceState(null, "", replayQuery(seq, frame)), 250);
    return () => clearTimeout(t);
  }, [drive, seq, frame]);

  // Playback at the recorded rate (about 10 scans a second) times the speed.
  useEffect(() => {
    if (!playing || !drive) return;
    const n = drive.frames.frames;
    const t = setInterval(() => {
      setFrame((f) => {
        if (f >= n - 1) { setPlaying(false); return f; }
        return f + 1;
      });
    }, 100 / speed);
    return () => clearInterval(t);
  }, [playing, speed, drive]);

  const f = drive?.frames;
  const n = f?.frames ?? 1;
  const go = useCallback((i: number) => { setPlaying(false); setFrame(clampFrame(i, n)); }, [n]);
  const notable = useMemo(() => (f ? notableFrames(f) : []), [f]);
  const charts = useMemo(() => {
    if (!f) return null;
    const maxStep = Math.max(...f.atlasStepCm, ...f.kissStepCm);
    const stepHi = Math.ceil(maxStep);
    const driftHi = Math.ceil(Math.max(...f.atlasDriftM) * 10) / 10;
    const inl = f.pairs.map((p, i) => (f.sourcePoints[i] ? (100 * p) / f.sourcePoints[i] : 100));
    const inlLo = Math.floor(Math.min(...inl.slice(1)));
    const itHi = Math.max(...f.iterations);
    return {
      stepHi, driftHi, inlLo, itHi,
      atlas: chartPoints(f.atlasStepCm, 0, stepHi, 1000, 96),
      kiss: chartPoints(f.kissStepCm, 0, stepHi, 1000, 96),
      drift: chartPoints(f.atlasDriftM, 0, driftHi, 1000, 40),
      inl: chartPoints(inl, inlLo, 100, 1000, 40, { keep: "min" }),
      it: chartPoints(f.iterations, 0, itHi, 1000, 40),
    };
  }, [f]);
  const x = n > 1 ? (frame / (n - 1)) * 1000 : 0;
  const onChart = (e: React.MouseEvent<SVGSVGElement>) => {
    const r = e.currentTarget.getBoundingClientRect();
    go(frameAtX(e.clientX - r.left, r.width, n));
  };
  const inlierPct = f && f.sourcePoints[frame] && frame > 0 ? (100 * f.pairs[frame]) / f.sourcePoints[frame] : null;

  const chart = (label: string, range: string, h: number, lines: { points: string; cls: string }[], aria: string) => (
    <>
      <span className="rp-chart-label"><span>{label}</span> <span>{range}</span></span>
      <svg className="rp-chart" viewBox={`0 0 1000 ${h}`} preserveAspectRatio="none" style={{ height: h }} role="img"
        aria-label={aria} onClick={onChart}>
        {lines.map((l) => <polyline key={l.cls} points={l.points} className={l.cls} />)}
        <line x1={x} x2={x} y1={0} y2={h} className="rp-playhead" />
      </svg>
    </>
  );

  return (
    <div className="rp">
      <header className="rp-head wrap">
        <div className="rp-brand">
          <a href={import.meta.env.BASE_URL} className="rp-logo">Atlas</a>
          <span className="rp-title">Replay explorer</span>
          <span className="rp-sub">KITTI sequence {seq} · {DRIVES[seq]}{f ? ` · ${fmt(f.frames, 0)} scans` : ""}</span>
        </div>
        <div className="rp-head-right">
          <div className="seg rp-seg" role="group" aria-label="Drive">
            {SEQUENCES.map((s) => (
              <button key={s} type="button" aria-pressed={seq === s} onClick={() => { wantFrame.current = 0; setSeq(s); }}>
                Sequence {s}
              </button>
            ))}
          </div>
          <span className="rp-replay" data-testid="replay-label">
            <svg width="10" height="10" viewBox="0 0 10 10" aria-hidden="true"><path d="M2 1.5 L8.5 5 L2 8.5 Z" /></svg>
            REPLAY · recorded on an Apple M4, not computed in this page
          </span>
        </div>
      </header>

      <main className="rp-main wrap">
        <section className="rp-view" aria-label="3D view of the drive">
          <div ref={host} className="rp-canvas" />
          {viewError && <p className="rp-state">{viewError}</p>}
          {!viewError && !drive && !error && (
            <div className="rp-state" role="status">
              <p>Loading sequence {seq}: Atlas's map and {n > 1 ? fmt(n, 0) : "every"} scan…</p>
              <div className="rp-progress"><div style={{ width: `${Math.round(progress * 100)}%` }} /></div>
            </div>
          )}
          {error && <p className="rp-state" role="alert">Could not load the recording: {error}. Reload the page to try again.</p>}
          {drive && (
            <>
              <div className="rp-overlay rp-top">
                <span className="pill mono" data-testid="frame-stamp">Frame {frame} / {n - 1} · {fmt(f!.time[frame], 1)} s</span>
                <div className="seg" role="group" aria-label="Camera">
                  {(["chase", "top", "orbit"] as const).map((m) => (
                    <button key={m} type="button" aria-pressed={mode === m} onClick={() => setMode(m)}>
                      {m === "chase" ? "Chase" : m === "top" ? "Top" : "Free orbit"}
                    </button>
                  ))}
                </div>
              </div>
              <div className="rp-overlay rp-bottom">
                {([["map", "Atlas's map", "rp-sw-map"], ["scan", "This scan", "rp-sw-scan"], ["truth", "True path", "rp-sw-truth"],
                  ["kiss", "KISS-ICP path", "rp-sw-kiss"]] as const).map(([k, label, sw]) => (
                  <button key={k} type="button" className="rp-layer" aria-pressed={layers[k]}
                    onClick={() => setLayers((l) => ({ ...l, [k]: !l[k] }))}>
                    <span className={`rp-sw ${sw}`} aria-hidden="true" />{label}
                  </button>
                ))}
                <span className="rp-layer rp-static"><span className="rp-sw rp-sw-est" aria-hidden="true" />Atlas's estimated path</span>
              </div>
            </>
          )}
        </section>

        <aside className="rp-side" aria-label="Frame details">
          {f ? (
            <>
              <div className="rp-frame-head">
                <p className="eyebrow">Frame {frame}</p>
                <h1 data-testid="frame-title">{describeFrame(f, frame)}</h1>
              </div>
              <div className="rp-block">
                <p className="rp-label">Against the answer key</p>
                <dl className="rp-stats">
                  <div><dd className="rp-blue" data-testid="atlas-step">{fmt(f.atlasStepCm[frame], 1)} cm</dd><dt>Atlas step error</dt></div>
                  <div><dd>{fmt(f.kissStepCm[frame], 1)} cm</dd><dt>KISS-ICP step error</dt></div>
                  <div><dd>{fmt(f.atlasDriftM[frame], 2)} m</dd><dt>Atlas drift so far</dt></div>
                </dl>
              </div>
              <div className="rp-block rp-thin">
                <p className="rp-label">What the engine saw (no answer key)</p>
                <table className="rp-table">
                  <tbody>
                    <tr><th scope="row">Matched points (inliers)</th><td>{inlierPct === null ? "first scan" : `${fmt(f.pairs[frame], 0)} of ${fmt(f.sourcePoints[frame], 0)} · ${fmt(inlierPct, 1)} %`}</td></tr>
                    <tr><th scope="row">ICP iterations</th><td>{f.iterations[frame]}</td></tr>
                    <tr><th scope="row">Match distance limit (adaptive)</th><td>{fmt(f.sigmaM[frame], 2)} m</td></tr>
                    <tr><th scope="row">Map size</th><td>{fmt(f.engineMapPoints[frame], 0)} points</td></tr>
                    <tr><th scope="row">Time for this scan</th><td>{fmt(f.engineMs[frame], 0)} ms</td></tr>
                  </tbody>
                </table>
              </div>
              <p className="rp-note">
                Step error: how far this scan's estimated motion is from the true motion. Drift: how far Atlas's position
                is from the true one so far. The engine never sees the answer key; it is used here only to grade it.
              </p>
            </>
          ) : (
            <p className="rp-note">The frame details appear when the recording has loaded.</p>
          )}
        </aside>
      </main>

      <section className="rp-timeline wrap" aria-label="Timeline">
        <div className="rp-controls">
          <button type="button" className="rp-play" aria-label={playing ? "Pause" : "Play"} disabled={!drive}
            onClick={() => { if (frame >= n - 1) setFrame(0); setPlaying((p) => !p); }}>
            {playing
              ? <svg width="14" height="14" viewBox="0 0 14 14" aria-hidden="true"><rect x="2.5" y="2" width="3" height="10" /><rect x="8.5" y="2" width="3" height="10" /></svg>
              : <svg width="14" height="14" viewBox="0 0 14 14" aria-hidden="true"><path d="M3 1.5 L12 7 L3 12.5 Z" /></svg>}
          </button>
          <button type="button" className="rp-step" aria-label="Previous scan" disabled={!drive} onClick={() => go(frame - 1)}>‹</button>
          <button type="button" className="rp-step" aria-label="Next scan" disabled={!drive} onClick={() => go(frame + 1)}>›</button>
          <div className="seg rp-seg" role="group" aria-label="Speed">
            {([1, 4] as const).map((s) => <button key={s} type="button" aria-pressed={speed === s} onClick={() => setSpeed(s)}>{s}×</button>)}
          </div>
          <div className="rp-jumps">
            <span>Jump to:</span>
            {notable.map((j) => (
              <button key={j.frame} type="button" className="rp-jump" aria-pressed={frame === j.frame} onClick={() => go(j.frame)}>
                {j.frame} · {j.label}
              </button>
            ))}
          </div>
        </div>
        {charts && f && (
          <div className="rp-charts">
            {chart("Step error", `0 to ${charts.stepHi} cm`, 96, [
              { points: charts.kiss, cls: "rp-l-kiss" }, { points: charts.atlas, cls: "rp-l-atlas" }],
              "Step error per scan: Atlas in blue, KISS-ICP in gray. Click to jump to a frame.")}
            {chart("Drift so far", `0 to ${charts.driftHi} m`, 40, [{ points: charts.drift, cls: "rp-l-atlas" }],
              "Atlas's distance from the true position, per scan. Click to jump to a frame.")}
            {chart("Inliers", `${charts.inlLo} to 100 %`, 40, [{ points: charts.inl, cls: "rp-l-ink" }],
              "Share of points matched to the map, per scan. Click to jump to a frame.")}
            {chart("ICP iterations", `0 to ${charts.itHi}`, 40, [{ points: charts.it, cls: "rp-l-ink" }],
              "ICP iterations per scan. Click to jump to a frame.")}
            <span className="rp-chart-label"><span className="rp-key-atlas">Atlas</span> · <span className="rp-key-kiss">KISS-ICP</span></span>
            <div>
              <input type="range" className="rp-range" min={0} max={n - 1} value={frame} aria-label="Frame"
                onChange={(e) => go(Number(e.target.value))} />
              <div className="rp-axis"><span>scan 0</span><span>scan {fmt(n - 1, 0)} · {fmt(f.time[n - 1], 0)} s</span></div>
            </div>
          </div>
        )}
        <p className="rp-foot">
          Replay of recorded runs: Atlas (scan-to-map, default settings) and KISS-ICP 1.3.0 on the same raw scans.
          The map is Atlas's own, built from its estimated poses ({f ? `${fmt(f.mapPoints, 0)} points, one per ${f.mapVoxel} m cube` : "thinned"}),
          and each scan is drawn where Atlas believed the car was (thinned to {f ? f.scanPoints : "a few hundred"} points).
          Data: <a href="https://www.cvlibs.net/datasets/kitti/">KITTI</a> (Geiger et al.), CC BY-NC-SA 3.0; these derived
          files are shared under the same license. <a href="https://github.com/djism/atlas/blob/main/docs/replay.md">How this was made</a>.
        </p>
      </section>
    </div>
  );
}
