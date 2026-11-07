import type { Furniture, Point, Segment } from "../types";

// A scenario is data: a world, a robot, a sensor, and patrol points.
// New worlds (street car, warehouse robot) are new Scenario objects.
export interface Scenario {
  id: string;
  title: string;
  tagline: string;
  bounds: { width: number; height: number }; // meters, world frame from (0, 0)
  walls: Segment[]; // fixed structure
  furniture: Furniture[]; // initial movable objects
  patrol: Point[]; // points the robot visits in order; the C++ planner finds paths between them
  robot: { radius: number; label: string };
  lidar: { beams: number; maxRange: number; noise: number }; // defaults
  stepLength: number; // meters traveled between scans
  tips: string[]; // suggested experiments shown to visitors
}
