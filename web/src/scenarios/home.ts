import type { Scenario } from "./types";

// Two-room apartment, 12 x 8 m, joined by a 1.5 m doorway in a wall at x = 6.
export const homeScenario: Scenario = {
  id: "home",
  title: "Home robot",
  tagline: "A robot vacuum maps nothing and remembers only its last scan.",
  bounds: { width: 12, height: 8 },
  walls: [
    { a: { x: 0, y: 0 }, b: { x: 12, y: 0 } },
    { a: { x: 12, y: 0 }, b: { x: 12, y: 8 } },
    { a: { x: 12, y: 8 }, b: { x: 0, y: 8 } },
    { a: { x: 0, y: 8 }, b: { x: 0, y: 0 } },
    { a: { x: 6, y: 0 }, b: { x: 6, y: 3 } },
    { a: { x: 6, y: 4.5 }, b: { x: 6, y: 8 } },
  ],
  furniture: [
    { id: "sofa", label: "Sofa", x: 0.5, y: 4, w: 0.8, h: 2.2, angle: 0, height: 0.42, kind: "furniture" },
    { id: "table", label: "Table", x: 3, y: 4, w: 1.2, h: 0.7, angle: 0, height: 0.45, kind: "furniture" },
    { id: "shelf", label: "Shelf", x: 3, y: 7.75, w: 2, h: 0.4, angle: 0, height: 0.9, kind: "furniture" },
    { id: "bed", label: "Bed", x: 11.1, y: 4.25, w: 1.6, h: 2.1, angle: 0, height: 0.5, kind: "furniture" },
    { id: "wardrobe", label: "Wardrobe", x: 8, y: 7.6, w: 1.6, h: 0.6, angle: 0, height: 1.1, kind: "furniture" },
    { id: "desk", label: "Desk", x: 8.5, y: 0.45, w: 1.4, h: 0.6, angle: 0, height: 0.45, kind: "furniture" },
  ],
  patrol: [
    { x: 1.5, y: 1.5 },
    { x: 4.5, y: 1.5 },
    { x: 6, y: 3.75 },
    { x: 8, y: 2 },
    { x: 10, y: 2 },
    { x: 10, y: 6.5 },
    { x: 8, y: 6.5 },
    { x: 6, y: 3.75 },
    { x: 4.5, y: 6.5 },
    { x: 1.5, y: 6.5 },
  ],
  robot: { radius: 0.18, label: "Robot vacuum" },
  lidar: { beams: 360, maxRange: 8, noise: 0.01 },
  stepLength: 0.05,
  tips: [
    "Drag the table into the robot's way: the planner finds a path around it.",
    "Block the doorway with a box: the robot cannot reach the other room and says so.",
    "Drop a box onto a patrol point: the robot skips that point.",
    "Set range to 2 m: near the doorway, too few walls are visible to track.",
    "Raise noise to 5 cm and watch drift grow.",
  ],
};
