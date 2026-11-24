// Geometry for the lab's small charts (pure; tested in tests/labCharts.test.ts).
// Errors in the lab span millimetres to metres, so both charts use a log scale.

/** y on a log scale: lo at the bottom (height - pad), hi at the top (pad); values are clamped. */
export function logY(v: number, lo: number, hi: number, height: number, pad = 4): number {
  if (!(lo > 0) || !(hi > lo)) throw new Error("logY: need 0 < lo < hi");
  const c = Math.min(hi, Math.max(lo, v));
  const t = (Math.log10(c) - Math.log10(lo)) / (Math.log10(hi) - Math.log10(lo));
  return pad + (height - 2 * pad) * (1 - t);
}

const LENGTHS: [number, string][] = [[0.001, "1 mm"], [0.01, "1 cm"], [0.1, "10 cm"], [1, "1 m"], [10, "10 m"]];
/** Decade ticks inside [lo, hi], labeled as lengths. */
export const lengthTicks = (lo: number, hi: number) => LENGTHS.filter(([v]) => v >= lo * 0.999 && v <= hi * 1.001);

/** One reading of the "Get it lost" experiment. null = no value (not started, or lost this scan). */
export interface LostSample {
  withMap: number | null; // meters
  noMap: number | null; // meters
  inliers: number | null; // 0..1; null while the map localizer is lost
}

/** Keeps the last `capacity` samples (oldest dropped). Returns a new array. */
export function pushSample<T>(history: T[], sample: T, capacity: number): T[] {
  const next = history.length >= capacity ? history.slice(history.length - capacity + 1) : history.slice();
  next.push(sample);
  return next;
}

/**
 * Polyline point strings for one series, newest sample at the right edge and
 * `capacity` samples across the width. A missing value breaks the line, so a
 * gap is drawn as a gap, never as a straight jump.
 */
export function historySegments(values: (number | null)[], capacity: number, width: number, height: number,
                                 lo: number, hi: number): string[] {
  const dx = width / Math.max(1, capacity - 1);
  const offset = width - (values.length - 1) * dx;
  const segments: string[] = [];
  let current: string[] = [];
  values.forEach((v, i) => {
    if (v === null || !Number.isFinite(v)) {
      if (current.length) segments.push(current.join(" "));
      current = [];
      return;
    }
    current.push(`${(offset + i * dx).toFixed(1)},${logY(v, lo, hi, height).toFixed(1)}`);
  });
  if (current.length) segments.push(current.join(" "));
  return segments;
}

/** Bars for the map-match strip: share of the scan matching the map, from `floor` to 100 %; lost scans are full-height marks. */
export function matchBars(samples: LostSample[], capacity: number, width: number, height: number, floor = 0.6) {
  const dx = width / Math.max(1, capacity - 1);
  const offset = width - (samples.length - 1) * dx;
  const w = Math.max(1, dx * 0.7);
  return samples.flatMap((s, i) => {
    if (s.withMap === null) return []; // not localizing yet
    const x = offset + i * dx - w / 2;
    if (s.inliers === null) return [{ x, w, y: 0, h: height, lost: true }];
    const h = Math.max(1, ((Math.min(1, Math.max(floor, s.inliers)) - floor) / (1 - floor)) * (height - 2));
    return [{ x, w, y: height - h, h, lost: false }];
  });
}

/**
 * The pass-by-pass chart of one ICP run: a bar per pass for the pair RMSE
 * (what the engine sees) and a dot per pose for the error against the answer
 * key (start = pass 0). Only passes up to `shown` are drawn, so the chart
 * fills in as the replay plays.
 */
export function passChart(rmse: number[], error: number[], shown: number, width: number, height: number,
                          lo = 0.002, hi = 0.4) {
  const n = error.length; // poses: start + one per pass
  if (rmse.length !== n - 1) throw new Error("passChart: need one RMSE per pass and one error per pose");
  const slot = width / Math.max(1, n);
  const cx = (i: number) => slot * (i + 0.5);
  const barW = Math.max(2, Math.min(26, slot * 0.6));
  const k = Math.max(0, Math.min(n - 1, shown));
  const bars = rmse.slice(0, k).map((v, j) => {
    const top = logY(v, lo, hi, height);
    return { x: cx(j + 1) - barW / 2, y: top, w: barW, h: height - 4 - top, current: j === k - 1 };
  });
  const dots = error.slice(0, k + 1).map((v, i) => ({ x: cx(i), y: logY(v, lo, hi, height) }));
  return { bars, dots, line: dots.map((d) => `${d.x.toFixed(1)},${d.y.toFixed(1)}`).join(" "), slot };
}
