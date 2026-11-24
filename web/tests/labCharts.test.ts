// Unit tests for the lab chart geometry. Run: npm test.
import assert from "node:assert/strict";
import { test } from "node:test";
import { historySegments, lengthTicks, logY, matchBars, passChart, pushSample } from "../src/lib/labCharts.ts";

const xy = (s: string) => s.split(" ").map((p) => p.split(",").map(Number));

test("log scale: each decade is the same height, larger is higher, values are clamped", () => {
  const a = logY(0.001, 0.001, 1, 100, 0), b = logY(0.01, 0.001, 1, 100, 0), c = logY(0.1, 0.001, 1, 100, 0);
  assert.equal(a, 100);
  assert.ok(Math.abs((a - b) - (b - c)) < 1e-9, "equal spacing per decade");
  assert.equal(logY(1, 0.001, 1, 100, 0), 0);
  assert.equal(logY(50, 0.001, 1, 100, 0), 0, "above hi clamps to the top");
  assert.equal(logY(0, 0.001, 1, 100, 0), 100, "zero clamps to the bottom instead of -Infinity");
  assert.throws(() => logY(1, 0, 1, 100));
  assert.deepEqual(lengthTicks(0.001, 3).map(([, l]) => l), ["1 mm", "1 cm", "10 cm", "1 m"]);
});

test("history keeps the newest samples, newest at the right edge", () => {
  let h: number[] = [];
  for (let i = 0; i < 75; i++) h = pushSample(h, i, 70);
  assert.equal(h.length, 70);
  assert.equal(h[0], 5);
  assert.equal(h[69], 74);
  const [seg] = historySegments([0.01, 0.1], 70, 300, 100, 0.001, 1);
  const pts = xy(seg);
  assert.equal(pts[1][0], 300, "newest sample at the right edge");
  assert.ok(pts[1][0] - pts[0][0] > 4 && pts[1][0] - pts[0][0] < 5, "spaced for 70 samples across 300 px");
  assert.ok(pts[1][1] < pts[0][1], "larger error is higher");
});

test("a missing reading breaks the line instead of bridging it", () => {
  const segs = historySegments([0.01, 0.02, null, 0.03, 0.04], 10, 90, 50, 0.001, 1);
  assert.equal(segs.length, 2);
  assert.equal(xy(segs[0]).length, 2);
  assert.equal(xy(segs[1]).length, 2);
});

test("map-match bars: taller for more matching, full-height red marks when lost, none before localizing", () => {
  const bars = matchBars([
    { withMap: null, noMap: null, inliers: null },
    { withMap: 0.002, noMap: 0.01, inliers: 1 },
    { withMap: 0.002, noMap: 0.01, inliers: 0.8 },
    { withMap: 0.02, noMap: 0.3, inliers: null },
  ], 4, 30, 30);
  assert.equal(bars.length, 3);
  assert.ok(bars[0].h > bars[1].h && !bars[0].lost);
  assert.equal(bars[0].h, 28);
  assert.deepEqual([bars[2].lost, bars[2].h], [true, 30]);
});

test("pass chart fills in pass by pass and marks the current pass", () => {
  const rmse = [0.264, 0.057, 0.012, 0.011, 0.011];
  const error = [0.098, 0.016, 0.0036, 0.0037, 0.0037, 0.0037];
  const start = passChart(rmse, error, 0, 300, 110);
  assert.deepEqual([start.bars.length, start.dots.length], [0, 1], "before pass 1: only the starting error");
  const mid = passChart(rmse, error, 2, 300, 110);
  assert.deepEqual([mid.bars.length, mid.dots.length], [2, 3]);
  assert.ok(mid.bars[1].current && !mid.bars[0].current);
  assert.ok(mid.bars[0].y < mid.bars[1].y, "26 cm bar is taller than 5.7 cm");
  const done = passChart(rmse, error, 99, 300, 110);
  assert.deepEqual([done.bars.length, done.dots.length], [5, 6], "shown is clamped to the run");
  assert.ok(done.dots[5].y > done.bars[4].y, "true error (3.7 mm) is drawn below the pair RMSE (1.1 cm)");
  assert.throws(() => passChart([0.1], [0.1], 0, 300, 110));
});
