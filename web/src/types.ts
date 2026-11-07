// Shared geometric types. All lengths in meters, angles in radians (CCW).
// World frame: x right, y up. Matches the C++ core's conventions.
export interface Point {
  x: number;
  y: number;
}

export interface Pose {
  x: number;
  y: number;
  theta: number;
}

export interface Segment {
  a: Point;
  b: Point;
}

// Movable rectangular object, centered at (x, y), rotated by angle.
export interface Furniture {
  id: string;
  label: string;
  x: number;
  y: number;
  w: number;
  h: number;
  angle: number;
  height: number; // meters, for 3D display only; the 2D sensor treats every object as solid
  kind: "furniture" | "box";
}
