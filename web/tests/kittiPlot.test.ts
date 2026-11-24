// Unit tests for the "Real data" plot layout. Run: npm test.
import assert from "node:assert/strict";
import { test } from "node:test";
import { fitPaths, niceLength, pairedBars, type PlotPoint } from "../src/lib/kittiPlot.ts";

// u = -y (right), v = x (forward, up). A path 100 m forward, then 50 m right.
const truth: PlotPoint[] = [[0, 0], [0, 100], [50, 100]];
const paths = { truth, atlas: truth, kiss: truth, noMap: [[0, 0], [-10, 90]] as PlotPoint[] };
const xy = (points: string) => points.split(" ").map((p) => p.split(",").map(Number));

test("forward is up, right is right, equal scale on both axes", () => {
  const fit = fitPaths(paths, 640, 28);
  const [a, b, c] = xy(fit.points.truth);
  assert.ok(b[1] < a[1], "forward goes up the screen");
  assert.ok(Math.abs(b[0] - a[0]) < 0.11, "straight forward stays vertical");
  assert.ok(c[0] > b[0] && Math.abs(c[1] - b[1]) < 0.11, "u (right) goes right");
  assert.ok(Math.abs((a[1] - b[1]) / 100 - (c[0] - b[0]) / 50) < 0.01, "same pixels per meter on both axes");
});

test("everything fits inside the box, centered horizontally", () => {
  const fit = fitPaths(paths, 640, 28);
  const all = Object.values(fit.points).flatMap(xy);
  for (const [x, y] of all) assert.ok(x >= 27.9 && x <= 612.1 && y >= 27.9 && y <= fit.height - 27.9, `${x},${y} inside`);
  const xs = all.map((p) => p[0]);
  assert.ok(Math.abs(Math.min(...xs) + Math.max(...xs) - 640) < 0.2, "centered");
  assert.equal(fit.viewBox, `0 0 640 ${fit.height}`);
});

test("start marker sits on the first true pose; scale bar matches the scale", () => {
  const fit = fitPaths(paths, 640, 28);
  const [first] = xy(fit.points.truth);
  assert.ok(Math.abs(fit.start.x - first[0]) < 0.06 && Math.abs(fit.start.y - first[1]) < 0.06);
  const [a, b] = xy(fit.points.truth);
  assert.equal(fit.barMeters, 20); // span 100 m -> at most 25 m -> 20 m
  assert.ok(Math.abs(fit.barLength - (a[1] - b[1]) * 0.2) < 0.2);
});

test("bad input is rejected", () => {
  assert.throws(() => fitPaths({ truth: [], atlas: [], kiss: [], noMap: [] }));
  assert.throws(() => fitPaths({ ...paths, atlas: [[0, NaN]] }));
  assert.throws(() => pairedBars([1, 2], [1]));
  assert.throws(() => niceLength(0));
});

test("scale-bar lengths and bar heights", () => {
  assert.deepEqual([niceLength(173), niceLength(100), niceLength(49), niceLength(0.3)], [100, 100, 20, 0.2]);
  assert.deepEqual(pairedBars([1, 4], [2, 2]), [{ a: 25, b: 50 }, { a: 100, b: 50 }]);
  assert.deepEqual(pairedBars([0], [0]), [{ a: 0, b: 0 }]);
});
