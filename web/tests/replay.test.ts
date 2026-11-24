// Unit tests for the replay explorer's pure helpers. Run: npm test.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { test } from "node:test";
import {
  chartPoints, checkReplay, clampFrame, decodePoints, describeFrame, frameAtX, notableFrames, parseReplayQuery,
  replayQuery, scanPoints, type ReplayFrames,
} from "../src/lib/replay.ts";

function int16Buffer(values: number[]): ArrayBuffer {
  const b = new ArrayBuffer(values.length * 2);
  const v = new DataView(b);
  values.forEach((x, i) => v.setInt16(i * 2, x, true));
  return b;
}

test("points decode little-endian int16 with the right unit", () => {
  const map = decodePoints(int16Buffer([15, -20, 3, 32767, -32768, 0]), 10);
  assert.deepEqual([...map].map((x) => Math.round(x * 10) / 10), [1.5, -2, 0.3, 3276.7, -3276.8, 0]);
  // Big-endian reading would turn 15 (0x000f) into 3840: make sure the bytes are read low first.
  const raw = new Uint8Array(int16Buffer([15, 0, 0]));
  assert.deepEqual([raw[0], raw[1]], [15, 0]);
});

test("scan k is the k-th block of scans.bin", () => {
  const perScan = 2;
  const scans = int16Buffer([100, 0, 0, 200, 0, 0, /* frame 1: */ 300, -100, 50, 400, 0, 0]);
  assert.deepEqual([...scanPoints(scans, 1, perScan)], [3, -1, 0.5, 4, 0, 0]);
  assert.throws(() => scanPoints(scans, 2, perScan), RangeError);
});

const tiny = (n: number): ReplayFrames => {
  const z = (v = 0) => Array.from({ length: n }, () => v);
  return {
    sequence: "07", frames: n, scanPoints: 2, mapPoints: 3, mapVoxel: 0.5, mapRange: 50,
    time: z().map((_, i) => i / 10), atlasPose: z().map(() => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]),
    truthPos: z().map(() => [0, 0, 0]), kissPos: z().map(() => [0, 0, 0]), atlasStepCm: z(1), kissStepCm: z(1),
    atlasDriftM: z(0), speedMps: z(5), turnDeg: z(0), iterations: z(10), pairs: z(100), sourcePoints: z(100),
    sigmaM: z(0.4), engineMs: z(30), engineMapPoints: z(1000),
  };
};

test("a drive is checked for consistent lengths and file sizes", () => {
  const f = tiny(4);
  assert.equal(checkReplay(f, 3 * 6, 4 * 2 * 6), null);
  assert.match(checkReplay(f, 3 * 6, 4 * 2 * 6 - 6) ?? "", /scans.bin/);
  assert.match(checkReplay(f, 2 * 6, 4 * 2 * 6) ?? "", /map.bin/);
  assert.match(checkReplay({ ...f, pairs: [1, 2] }, 3 * 6, 4 * 2 * 6) ?? "", /pairs/);
});

test("links: frame and drive round trip; bad values fall back", () => {
  assert.deepEqual(parseReplayQuery(replayQuery("04", 120)), { seq: "04", frame: 120 });
  assert.deepEqual(parseReplayQuery("?seq=99&frame=-3"), { seq: "07", frame: null });
  assert.deepEqual(parseReplayQuery("?frame=12abc"), { seq: "07", frame: null });
  assert.equal(clampFrame(5000, 1101), 1100);
  assert.equal(clampFrame(-2, 1101), 0);
});

test("charts: values map to height (up is larger), peaks survive bucketing, x maps back to frames", () => {
  const pts = chartPoints([0, 10, 5], 0, 10, 100, 20, { pad: 0 }).split(" ").map((p) => p.split(",").map(Number));
  assert.deepEqual(pts, [[0, 20], [50, 0], [100, 10]]);
  const values = Array.from({ length: 1000 }, (_, i) => (i === 777 ? 9 : 1));
  const ys = chartPoints(values, 0, 10, 100, 20, { buckets: 100, pad: 0 }).split(" ").map((p) => Number(p.split(",")[1]));
  assert.equal(Math.min(...ys), 2, "a one-frame spike is still drawn");
  assert.equal(chartPoints([5, 50], 0, 10, 10, 10, { pad: 0 }).split(" ")[1], "10.0,0.0", "values above hi are clipped");
  assert.equal(frameAtX(50, 100, 1101), 550);
  assert.equal(frameAtX(1000, 100, 1101), 1100);
});

test("notable frames: worst steps spaced apart, labeled; KISS worst and most drift added", () => {
  const f = tiny(300);
  f.atlasStepCm[100] = 9; f.atlasStepCm[105] = 8.5; f.atlasStepCm[200] = 7; f.atlasStepCm[30] = 6;
  f.turnDeg[100] = 2.7; f.speedMps[30] = 1;
  f.kissStepCm[200] = 20;
  f.atlasDriftM[250] = 2.1;
  const n = notableFrames(f);
  assert.deepEqual(n.map((x) => x.frame), [30, 100, 200, 250], "105 is too close to 100");
  assert.equal(n.find((x) => x.frame === 100)?.label, "Atlas worst · turn");
  assert.equal(n.find((x) => x.frame === 30)?.label, "Atlas worst · slow");
  assert.equal(n.find((x) => x.frame === 200)?.label, "Atlas worst · straight · KISS worst");
  assert.equal(n.find((x) => x.frame === 250)?.label, "Most drift");
});

test("frame headlines follow the answer key's motion", () => {
  const f = tiny(5);
  f.turnDeg[1] = 2.7; f.speedMps[1] = 4.5;
  f.turnDeg[2] = -2; f.speedMps[3] = 0.1; f.turnDeg[4] = 0.7;
  assert.equal(describeFrame(f, 0), "First scan: the map starts here");
  assert.equal(describeFrame(f, 1), "Left turn, 4.5 m/s");
  assert.equal(describeFrame(f, 2), "Right turn, 5.0 m/s");
  assert.equal(describeFrame(f, 3), "Standing still");
  assert.equal(describeFrame(f, 4), "Gentle curve, 5.0 m/s");
});

test("the committed replay data is consistent", () => {
  for (const seq of ["07", "04"]) {
    const dir = new URL(`../public/replay/${seq}/`, import.meta.url);
    const f = JSON.parse(readFileSync(new URL("frames.json", dir), "utf8")) as ReplayFrames;
    const map = readFileSync(new URL("map.bin", dir)).byteLength;
    const scans = readFileSync(new URL("scans.bin", dir)).byteLength;
    assert.equal(checkReplay(f, map, scans), null, seq);
    assert.equal(f.sequence, seq);
    const steps = notableFrames(f);
    assert.ok(steps.length >= 3, `${seq}: some notable frames`);
  }
});
