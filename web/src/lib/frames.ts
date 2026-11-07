import type { Point, Pose } from "../types";

// Frame bookkeeping for drawing (the estimation itself happens in the C++ engine).
// A Pose expresses a SOURCE frame in a DESTINATION frame, as in the C++ core:
// destination_point = R * source_point + t.

export const wrapAngle = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

export function transformPoint(sourceInDest: Pose, p: Point): Point {
  const c = Math.cos(sourceInDest.theta);
  const s = Math.sin(sourceInDest.theta);
  return { x: c * p.x - s * p.y + sourceInDest.x, y: s * p.x + c * p.y + sourceInDest.y };
}

// b_in_a composed with c_in_b gives c_in_a.
export function composePoses(bInA: Pose, cInB: Pose): Pose {
  const p = transformPoint(bInA, cInB);
  return { x: p.x, y: p.y, theta: wrapAngle(bInA.theta + cInB.theta) };
}

// Rotate a direction (not a point) from world axes into a frame's axes.
export function rotateIntoFrame(frame: Pose, v: Point): Point {
  const c = Math.cos(frame.theta);
  const s = Math.sin(frame.theta);
  return { x: c * v.x + s * v.y, y: -s * v.x + c * v.y };
}

export const formatLength = (m: number) =>
  m < 0.01 ? `${(m * 1000).toFixed(1)} mm` : m < 1 ? `${(m * 100).toFixed(1)} cm` : `${m.toFixed(2)} m`;
