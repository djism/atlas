import type { Point, Pose } from "../types";

// Messages between the page (main thread) and the engine worker. Every request
// carries an id; the worker answers in order, one message at a time.
export type ToEngine =
  | {
      // (Re)start both estimators at a known pose. The map is surveyed from
      // `walls` as they are now, at the answer-key `survey` poses.
      type: "reset";
      owner: number; // which view's estimators (each 3D view has its own robot)
      pose: Pose;
      walls: Float64Array; // flat segments, world frame
      survey: Pose[];
    }
  | {
      type: "step";
      id: number;
      walls: Float64Array; // flat segments, world frame
      truePose: Pose; // used ONLY to simulate the sensor, never given to the tracker
      beams: number;
      maxRange: number;
      noise: number;
      seed: number;
      track: boolean; // also run the tracker and the map localizer on this scan
      owner: number; // whose estimators (see reset)
      // True motion since this owner's previous tracked scan (sensor frame now,
      // in the frame then). Used ONLY to simulate the wheel encoders.
      trueMotion: Pose;
      wheels: boolean; // map localizer predicts from wheel odometry (else constant velocity)
    }
  | {
      type: "plan";
      id: number;
      obstacles: Float64Array; // flat segments, world frame
      width: number;
      height: number;
      resolution: number;
      robotRadius: number;
      start: Point;
      goal: Point;
    }
  | {
      type: "align";
      id: number;
      first: Float64Array; // flat x, y, first sensor axes
      second: Float64Array; // flat x, y, second sensor axes
      guess: Pose; // initial guess: second sensor pose in the first frame
      pointToLine: boolean;
    };

export type IcpStatus = "converged" | "iteration-limit" | "too-few-pairs" | "degenerate";
export type PlanStatus = "found" | "start-blocked" | "goal-blocked" | "no-path";

export interface StepResult {
  type: "step";
  id: number;
  ranges: Float64Array; // per beam, Infinity = no return
  points: Float64Array; // flat x, y in sensor axes (noisy measurements)
  estimate: Pose | null; // tracker's world pose estimate (null when not tracking)
  lost: boolean;
  status: IcpStatus | null;
  map: MapStep | null; // map localizer's result (null when not tracking)
  engineMs: number; // simulate (+ track) time inside the worker
}

export interface MapStep {
  estimate: Pose; // sensor pose in the map (= world) frame
  lost: boolean; // registration failed or a jump was rejected; pose predicted
  weak: boolean; // walls left one direction unconstrained; prediction used along it
  inliers: number; // share of scan points that matched the map, 0..1
  odometry: Pose; // what the simulated wheels reported for this interval
}

export interface ResetDone {
  type: "reset-done";
  mapPoints: Float64Array; // flat x, y, world frame
  mapMs: number;
}

export interface PlanResult {
  type: "plan";
  id: number;
  status: PlanStatus;
  path: Point[]; // world frame, start to goal
  length: number;
  expanded: number;
  planMs: number;
}

export interface AlignResult {
  type: "align";
  id: number;
  status: IcpStatus;
  iterations: number;
  poses: Pose[]; // [guess, after pass 1, after pass 2, ...]
  rmse: number[]; // per completed pass
  pairs: Uint32Array[]; // per pose: nearest first-scan index for every second-scan point
  engineMs: number;
}

export type Reply = StepResult | PlanResult | AlignResult;

export type FromEngine =
  | { type: "ready" }
  | ResetDone
  | Reply
  | { type: "error"; id?: number; message: string };
