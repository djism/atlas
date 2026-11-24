// Pure helpers for the replay explorer (no DOM, no Three.js), unit-tested in
// tests/replay.test.ts. Data comes from web/public/replay/<seq>/ (see
// docs/replay.md): frames.json, map.bin and scans.bin.

export type Seq = "07" | "04";
export const SEQUENCES: Seq[] = ["07", "04"];

/** frames.json, written by tools/export_replay.py. Arrays have one entry per frame. */
export interface ReplayFrames {
  sequence: Seq;
  frames: number;
  scanPoints: number;
  mapPoints: number;
  mapVoxel: number;
  mapRange: number;
  time: number[];
  atlasPose: number[][]; // row-major 3x4 [R | t], Velodyne frame of scan 0
  truthPos: number[][];
  kissPos: number[][];
  atlasStepCm: number[];
  kissStepCm: number[];
  atlasDriftM: number[];
  speedMps: number[];
  turnDeg: number[]; // + = left
  iterations: number[];
  pairs: number[];
  sourcePoints: number[];
  sigmaM: number[];
  engineMs: number[];
  engineMapPoints: number[];
}

/**
 * Little-endian int16 (x, y, z) triples -> Float32Array in meters.
 * `unitsPerMeter`: 10 for map.bin (decimeters), 100 for scans.bin (centimeters).
 */
export function decodePoints(buffer: ArrayBuffer, unitsPerMeter: number, offsetPoints = 0, count?: number): Float32Array {
  const total = Math.floor(buffer.byteLength / 6);
  const n = count ?? total - offsetPoints;
  if (offsetPoints < 0 || n < 0 || offsetPoints + n > total) throw new RangeError("decodePoints: out of range");
  const view = new DataView(buffer, offsetPoints * 6, n * 6);
  const out = new Float32Array(n * 3);
  for (let i = 0; i < n * 3; i++) out[i] = view.getInt16(i * 2, true) / unitsPerMeter;
  return out;
}

/** Points of scan `frame` (sensor frame, meters) out of scans.bin. */
export function scanPoints(scans: ArrayBuffer, frame: number, perScan: number): Float32Array {
  return decodePoints(scans, 100, frame * perScan, perScan);
}

/** Checks a loaded drive for consistency; returns a problem description or null. */
export function checkReplay(f: ReplayFrames, mapBytes: number, scanBytes: number): string | null {
  const arrays: (keyof ReplayFrames)[] = ["time", "atlasPose", "truthPos", "kissPos", "atlasStepCm", "kissStepCm",
    "atlasDriftM", "speedMps", "turnDeg", "iterations", "pairs", "sourcePoints", "sigmaM", "engineMs", "engineMapPoints"];
  for (const k of arrays) if ((f[k] as unknown[]).length !== f.frames) return `${k} has ${(f[k] as unknown[]).length} entries, expected ${f.frames}`;
  if (mapBytes !== f.mapPoints * 6) return `map.bin is ${mapBytes} bytes, expected ${f.mapPoints * 6}`;
  if (scanBytes !== f.frames * f.scanPoints * 6) return `scans.bin is ${scanBytes} bytes, expected ${f.frames * f.scanPoints * 6}`;
  return null;
}

export const clampFrame = (frame: number, frames: number) => Math.min(frames - 1, Math.max(0, Math.round(frame)));

/** ?seq=07&frame=476 -> state; anything invalid falls back to the defaults. */
export function parseReplayQuery(search: string): { seq: Seq; frame: number | null } {
  const q = new URLSearchParams(search);
  const seq = SEQUENCES.includes(q.get("seq") as Seq) ? (q.get("seq") as Seq) : "07";
  const raw = q.get("frame");
  const frame = raw !== null && /^\d{1,5}$/.test(raw) ? Number(raw) : null;
  return { seq, frame };
}

export const replayQuery = (seq: Seq, frame: number) => `?seq=${seq}&frame=${frame}`;

/**
 * SVG polyline points for a per-frame series on a `width` x `height` chart,
 * values clipped to [lo, hi]. With more frames than `buckets`, each bucket
 * keeps its largest value (peaks are what you look for) or smallest (`keep: "min"`).
 */
export function chartPoints(values: number[], lo: number, hi: number, width: number, height: number,
                            opts: { buckets?: number; keep?: "max" | "min"; pad?: number } = {}): string {
  if (values.length < 2 || !(hi > lo)) throw new Error("chartPoints: need two values and hi > lo");
  const { buckets = 400, keep = "max", pad = 3 } = opts;
  const per = Math.max(1, Math.ceil(values.length / buckets));
  const out: string[] = [];
  for (let i = 0; i < values.length; i += per) {
    const slice = values.slice(i, i + per);
    const v = keep === "max" ? Math.max(...slice) : Math.min(...slice);
    const x = (i / (values.length - 1)) * width;
    const y = pad + (height - 2 * pad) * (1 - (Math.min(hi, Math.max(lo, v)) - lo) / (hi - lo));
    out.push(`${x.toFixed(1)},${y.toFixed(1)}`);
  }
  return out.join(" ");
}

/** Chart x (0..width) -> nearest frame. */
export const frameAtX = (x: number, width: number, frames: number) => clampFrame((x / width) * (frames - 1), frames);

export interface Notable {
  frame: number;
  label: string;
}

/**
 * Frames worth jumping to, found from the data: Atlas's worst steps (at least
 * `spacing` frames apart, labeled by what the car was doing), KISS-ICP's
 * worst step, and the largest drift. Sorted by frame; one entry per frame.
 */
export function notableFrames(f: ReplayFrames, worst = 3, spacing = 50): Notable[] {
  const doing = (i: number) =>
    Math.abs(f.turnDeg[i]) >= 1.5 ? "turn" : f.speedMps[i] < 2 ? "slow" : "straight";
  const order = f.atlasStepCm.map((v, i) => [v, i] as const).filter(([, i]) => i > 0).sort((a, b) => b[0] - a[0]);
  const picked: number[] = [];
  for (const [, i] of order) {
    if (picked.length === worst) break;
    if (picked.every((p) => Math.abs(p - i) >= spacing)) picked.push(i);
  }
  const argmax = (v: number[]) => v.reduce((best, x, i) => (x > v[best] ? i : best), 0);
  const out = new Map<number, string>();
  for (const i of picked) out.set(i, `Atlas worst · ${doing(i)}`);
  const k = argmax(f.kissStepCm);
  out.set(k, out.has(k) ? `${out.get(k)} · KISS worst` : "KISS-ICP worst");
  const d = argmax(f.atlasDriftM);
  out.set(d, out.has(d) ? `${out.get(d)} · most drift` : "Most drift");
  return [...out].map(([frame, label]) => ({ frame, label })).sort((a, b) => a.frame - b.frame);
}

/** A short headline for a frame, from the answer key's motion: "Left turn, 4.5 m/s". */
export function describeFrame(f: ReplayFrames, i: number): string {
  const v = f.speedMps[i];
  if (i === 0) return "First scan: the map starts here";
  if (v < 0.3) return "Standing still";
  const turn = f.turnDeg[i];
  const what = Math.abs(turn) >= 1.5 ? (turn > 0 ? "Left turn" : "Right turn") : Math.abs(turn) >= 0.5 ? "Gentle curve" : "Straight";
  return `${what}, ${v.toFixed(1)} m/s`;
}
