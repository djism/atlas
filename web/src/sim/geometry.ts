import type { Furniture, Point, Segment } from "../types";

export function furnitureCorners(f: Furniture): Point[] {
  const c = Math.cos(f.angle);
  const s = Math.sin(f.angle);
  const hw = f.w / 2;
  const hh = f.h / 2;
  return [
    [-hw, -hh],
    [hw, -hh],
    [hw, hh],
    [-hw, hh],
  ].map(([lx, ly]) => ({ x: f.x + c * lx - s * ly, y: f.y + s * lx + c * ly }));
}

export function furnitureSegments(f: Furniture): Segment[] {
  const k = furnitureCorners(f);
  return k.map((a, i) => ({ a, b: k[(i + 1) % k.length] }));
}

// Flat [x0, y0, x1, y1, ...] for the engine.
export function flattenSegments(segments: Segment[]): Float64Array {
  const out = new Float64Array(segments.length * 4);
  segments.forEach((s, i) => out.set([s.a.x, s.a.y, s.b.x, s.b.y], i * 4));
  return out;
}

// Shortest distance from p to the segment a-b (world physics for the simulated robot).
export function distanceToSegment(p: Point, a: Point, b: Point): number {
  const ex = b.x - a.x;
  const ey = b.y - a.y;
  const len2 = ex * ex + ey * ey;
  const u = len2 > 0 ? Math.max(0, Math.min(1, ((p.x - a.x) * ex + (p.y - a.y) * ey) / len2)) : 0;
  return Math.hypot(p.x - a.x - u * ex, p.y - a.y - u * ey);
}

// Is p inside f, expanded by margin on every side?
export function insideFurniture(p: Point, f: Furniture, margin = 0): boolean {
  const c = Math.cos(f.angle);
  const s = Math.sin(f.angle);
  const dx = p.x - f.x;
  const dy = p.y - f.y;
  const lx = c * dx + s * dy;
  const ly = -s * dx + c * dy;
  return Math.abs(lx) <= f.w / 2 + margin && Math.abs(ly) <= f.h / 2 + margin;
}
