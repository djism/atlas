// Unit tests for the shareable-link codec. Run: npm test (Node's built-in
// runner; Node strips the TypeScript types itself, no extra dependencies).
import assert from "node:assert/strict";
import { test } from "node:test";
import { decodeLab, decodeLayout, encodeLab, encodeLayout, layoutOf, type LabLimits, type LabState } from "../src/lib/share.ts";
import type { Furniture } from "../src/types.ts";

const limits: LabLimits = {
  range: { min: 1, max: 8, step: 0.5, default: 8 },
  noiseCm: { min: 0, max: 5, step: 0.5, default: 1 },
  maxBoxes: 10,
  width: 12,
  height: 8,
};
const ids = ["sofa", "table", "desk"];
const sofa: Furniture = { id: "sofa", label: "Sofa", x: 0.5, y: 4, w: 0.8, h: 2.2, angle: 0, height: 0.42, kind: "furniture" };
const box = (x: number, y: number): Furniture => ({ id: `box-${x}`, label: "Box", x, y, w: 0.5, h: 0.5, angle: 0, height: 0.4, kind: "box" });

test("layout lists only moved furniture and boxes", () => {
  const unmoved = layoutOf([sofa], [sofa]);
  assert.deepEqual(unmoved, { moved: {}, boxes: [] });
  const moved = layoutOf([{ ...sofa, x: 1.234 }, box(3.2, 4.1)], [sofa]);
  assert.deepEqual(moved.moved, { sofa: { x: 1.234, y: 4 } });
  assert.deepEqual(moved.boxes, [{ x: 3.2, y: 4.1 }]);
});

test("layout text: centimeters, readable, round trip", () => {
  const text = encodeLayout({ moved: { table: { x: 3, y: 4.256 }, sofa: { x: 1.2, y: 3.5 } }, boxes: [{ x: 3.2, y: 4.1 }] });
  assert.equal(text, "sofa:1.20,3.50;table:3.00,4.26;box:3.20,4.10");
  assert.deepEqual(decodeLayout(text, ids, limits), {
    moved: { sofa: { x: 1.2, y: 3.5 }, table: { x: 3, y: 4.26 } },
    boxes: [{ x: 3.2, y: 4.1 }],
  });
});

test("hand-edited layouts: bad parts are skipped, not fatal", () => {
  const layout = decodeLayout("sofa:1,2;piano:3,3;box:20,3;box:abc;table:2,2,2;desk:-1,2;;box:1,1", ids, limits);
  assert.deepEqual(layout, { moved: { sofa: { x: 1, y: 2 } }, boxes: [{ x: 1, y: 1 }] });
  const many = Array.from({ length: 15 }, (_, k) => `box:${k + 1},1`).join(";");
  assert.equal(decodeLayout(many, ids, limits).boxes.length, 10);
});

test("no lab parameter: no lab state", () => {
  assert.equal(decodeLab("", ids, limits), null);
  assert.equal(decodeLab("?lab=dance", ids, limits), null);
});

test("defaults are omitted from the link", () => {
  const state: LabState = { mode: "lost", range: 8, noiseCm: 1, wheels: true, room: { moved: {}, boxes: [] }, mapped: { moved: {}, boxes: [] } };
  assert.equal(encodeLab(state, limits), "?lab=lost");
  assert.equal(encodeLab({ ...state, mode: "match", range: 2 }, limits), "?lab=match");
});

test("lost-tab link round trip, with a map from a different layout", () => {
  const state: LabState = {
    mode: "lost", range: 1.5, noiseCm: 2.5, wheels: false,
    room: { moved: { sofa: { x: 1.2, y: 3.5 } }, boxes: [{ x: 3.2, y: 4.1 }] },
    mapped: { moved: {}, boxes: [] },
  };
  const link = encodeLab(state, limits);
  assert.equal(link, "?lab=lost&range=1.5&noise=2.5&guess=last&room=sofa:1.20,3.50;box:3.20,4.10&mapped=");
  assert.deepEqual(decodeLab(link, ids, limits), state);
});

test("map layout defaults to the room layout when not given", () => {
  const s = decodeLab("?lab=plan&room=box:2,2", ids, limits)!;
  assert.deepEqual(s.mapped, s.room);
  assert.equal(encodeLab(s, limits), "?lab=plan&room=box:2.00,2.00");
});

test("sensor values snap to the sliders and stay in range", () => {
  const s = decodeLab("?lab=lost&range=1.7&noise=99", ids, limits)!;
  assert.equal(s.range, 1.5);
  assert.equal(s.noiseCm, 5);
  assert.equal(decodeLab("?lab=lost&range=&noise=x", ids, limits)!.range, 8);
  assert.equal(decodeLab("?lab=lost&range=0", ids, limits)!.range, 1);
});

test("percent-encoded links (e.g. from a chat app) still decode", () => {
  const s = decodeLab("?lab=plan&room=sofa%3A1.20%2C3.50%3Bbox%3A2.00%2C2.00", ids, limits)!;
  assert.deepEqual(s.room, { moved: { sofa: { x: 1.2, y: 3.5 } }, boxes: [{ x: 2, y: 2 }] });
});
