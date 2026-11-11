import type { Furniture, Point } from "../types";

// Shareable lab state in the page URL, e.g.
//   ?lab=lost&range=1.5&noise=2&guess=last&room=sofa:1.20,3.50;box:3.20,4.10#lab
// A link reproduces the SETUP (tab, sensor, motion guess, furniture), not the
// exact run: the simulation's timing still depends on the machine.
// Kept free of runtime imports so Node can unit-test it directly.

export type LabMode = "match" | "lost" | "plan";

// Furniture moved away from the scenario's default spot, plus dropped boxes.
// Positions: world frame, meters, centers.
export interface Layout {
  moved: Record<string, Point>;
  boxes: Point[];
}

export interface LabState {
  mode: LabMode;
  range: number; // meters
  noiseCm: number;
  wheels: boolean; // map localizer's motion guess: wheel odometry (else last motion)
  room: Layout; // furniture now
  mapped: Layout; // furniture when the map was surveyed
}

export interface LabLimits {
  range: { min: number; max: number; step: number; default: number };
  noiseCm: { min: number; max: number; step: number; default: number };
  maxBoxes: number;
  width: number; // room bounds, meters
  height: number;
}

const MODES: LabMode[] = ["match", "lost", "plan"];
const MOVED_TOLERANCE = 0.005; // meters: closer than this to the default counts as not moved

export const emptyLayout = (): Layout => ({ moved: {}, boxes: [] });

// Layout from the furniture list, relative to the scenario's defaults.
export function layoutOf(furniture: Furniture[], defaults: Furniture[]): Layout {
  const layout = emptyLayout();
  for (const f of furniture) {
    if (f.kind === "box") layout.boxes.push({ x: f.x, y: f.y });
    else {
      const d = defaults.find((g) => g.id === f.id);
      if (!d || Math.hypot(f.x - d.x, f.y - d.y) > MOVED_TOLERANCE) layout.moved[f.id] = { x: f.x, y: f.y };
    }
  }
  return layout;
}

const cm = (v: number) => (Math.round(v * 100) / 100).toFixed(2);

export function encodeLayout(layout: Layout): string {
  const parts = Object.keys(layout.moved)
    .sort()
    .map((id) => `${id}:${cm(layout.moved[id].x)},${cm(layout.moved[id].y)}`);
  for (const b of layout.boxes) parts.push(`box:${cm(b.x)},${cm(b.y)}`);
  return parts.join(";");
}

// Unknown ids, malformed numbers, and points outside the room are skipped, not
// fatal: a hand-edited link should still open. Extra boxes beyond maxBoxes too.
export function decodeLayout(text: string, knownIds: string[], limits: LabLimits): Layout {
  const layout = emptyLayout();
  for (const part of text.split(";")) {
    const m = /^([a-z][a-z0-9-]*):(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)$/.exec(part.trim());
    if (!m) continue;
    const p = { x: Number(m[2]), y: Number(m[3]) };
    if (!(p.x > 0 && p.x < limits.width && p.y > 0 && p.y < limits.height)) continue;
    if (m[1] === "box") {
      if (layout.boxes.length < limits.maxBoxes) layout.boxes.push(p);
    } else if (knownIds.includes(m[1])) layout.moved[m[1]] = p;
  }
  return layout;
}

export const sameLayout = (a: Layout, b: Layout) => encodeLayout(a) === encodeLayout(b);

// Snap to the slider's steps and clamp to its range; NaN gives the default.
function onSlider(text: string | null, s: LabLimits["range"]): number {
  const v = Number(text);
  if (text === null || text.trim() === "" || !Number.isFinite(v)) return s.default;
  const snapped = s.min + Math.round((v - s.min) / s.step) * s.step;
  return Math.min(s.max, Math.max(s.min, Math.round(snapped * 1000) / 1000));
}

export function decodeLab(search: string, knownIds: string[], limits: LabLimits): LabState | null {
  const q = new URLSearchParams(search);
  const mode = q.get("lab");
  if (!MODES.includes(mode as LabMode)) return null;
  const room = decodeLayout(q.get("room") ?? "", knownIds, limits);
  return {
    mode: mode as LabMode,
    range: onSlider(q.get("range"), limits.range),
    noiseCm: onSlider(q.get("noise"), limits.noiseCm),
    wheels: q.get("guess") !== "last",
    room,
    mapped: q.has("mapped") ? decodeLayout(q.get("mapped") ?? "", knownIds, limits) : room,
  };
}

// Only what differs from the defaults is written, so links stay short. The
// sensor and guess matter only in "lost"; furniture only outside "match".
export function encodeLab(state: LabState, limits: LabLimits): string {
  const q = new URLSearchParams({ lab: state.mode });
  if (state.mode === "lost") {
    if (state.range !== limits.range.default) q.set("range", String(state.range));
    if (state.noiseCm !== limits.noiseCm.default) q.set("noise", String(state.noiseCm));
    if (!state.wheels) q.set("guess", "last");
  }
  if (state.mode !== "match") {
    const room = encodeLayout(state.room);
    if (room) q.set("room", room);
    if (!sameLayout(state.room, state.mapped)) q.set("mapped", encodeLayout(state.mapped));
  }
  // Keep ":", ",", ";" readable; URLSearchParams would percent-encode them.
  return `?${q.toString().replace(/%3A/g, ":").replace(/%2C/g, ",").replace(/%3B/g, ";")}`;
}
