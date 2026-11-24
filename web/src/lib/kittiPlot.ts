// Layout for the recorded KITTI paths in the "Real data" section. Pure
// functions (no DOM), so they are unit-tested in tests/kittiPlot.test.ts.

/** Top-down point in meters: u = -y (right on screen), v = x (up), scan 0's Velodyne frame. */
export type PlotPoint = [u: number, v: number];

export type PathName = "truth" | "atlas" | "kiss" | "noMap";

export interface KittiRecording {
  scans: number;
  scansPerStretch: number;
  paths: Record<PathName, PlotPoint[]>;
  stretchErrorCm: { atlas: number[]; kiss: number[] };
}

export interface FittedPlot {
  /** SVG viewBox, "0 0 width height". */
  viewBox: string;
  width: number;
  height: number;
  /** SVG polyline `points` for each path. */
  points: Record<PathName, string>;
  start: { x: number; y: number };
  /** Length in SVG units of a `barMeters` scale bar. */
  barMeters: number;
  barLength: number;
}

/**
 * Fits every path into a `size` x `size` box (minus `pad`) on EQUAL axes and
 * centers it, so a straight road stays straight and a loop keeps its shape.
 * Screen y grows downward, so v (forward) is flipped. Throws on empty input.
 */
export function fitPaths(paths: Record<PathName, PlotPoint[]>, size = 640, pad = 28): FittedPlot {
  const all = Object.values(paths).flat();
  if (all.length === 0) throw new Error("fitPaths: no points");
  let umin = Infinity, umax = -Infinity, vmin = Infinity, vmax = -Infinity;
  for (const [u, v] of all) {
    if (!Number.isFinite(u) || !Number.isFinite(v)) throw new Error("fitPaths: nonfinite point");
    umin = Math.min(umin, u); umax = Math.max(umax, u);
    vmin = Math.min(vmin, v); vmax = Math.max(vmax, v);
  }
  const du = Math.max(umax - umin, 1);
  const dv = Math.max(vmax - vmin, 1);
  const scale = (size - 2 * pad) / Math.max(du, dv); // SVG units per meter, both axes
  const width = size;
  const height = Math.ceil(dv * scale + 2 * pad);
  const ox = (width - du * scale) / 2; // center horizontally
  const x = (u: number) => ox + (u - umin) * scale;
  const y = (v: number) => pad + (vmax - v) * scale;
  const fmt = (p: PlotPoint) => `${x(p[0]).toFixed(1)},${y(p[1]).toFixed(1)}`;
  const points = Object.fromEntries(
    (Object.keys(paths) as PathName[]).map((k) => [k, paths[k].map(fmt).join(" ")]),
  ) as Record<PathName, string>;
  const first = paths.truth[0] ?? all[0];
  const barMeters = niceLength(Math.max(du, dv) / 4);
  return { viewBox: `0 0 ${width} ${height}`, width, height, points, start: { x: x(first[0]), y: y(first[1]) }, barMeters, barLength: barMeters * scale };
}

/** Largest 1, 2 or 5 x 10^k not above `limit` (scale-bar lengths: 50, 100, 200 m...). */
export function niceLength(limit: number): number {
  if (!(limit > 0)) throw new Error("niceLength: limit must be positive");
  const power = 10 ** Math.floor(Math.log10(limit));
  for (const m of [5, 2, 1]) if (m * power <= limit) return m * power;
  return power;
}

/** Bar heights in percent of the tallest value across both series (paired stretches). */
export function pairedBars(a: number[], b: number[]): { a: number; b: number }[] {
  if (a.length !== b.length) throw new Error("pairedBars: series differ in length");
  const max = Math.max(...a, ...b);
  if (!(max > 0)) return a.map(() => ({ a: 0, b: 0 }));
  return a.map((v, i) => ({ a: (100 * v) / max, b: (100 * b[i]) / max }));
}
