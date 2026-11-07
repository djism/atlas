import type { Point, Pose } from "../types";

// Messages between the page (main thread) and the engine worker. Every request
// carries an id; the worker answers in order, one message at a time.
export type ToEngine =
  | { type: "reset"; pose: Pose } // (re)start the tracker at a known pose
  | {
      type: "step";
      id: number;
      walls: Float64Array; // flat segments, world frame
      truePose: Pose; // used ONLY to simulate the sensor, never given to the tracker
      beams: number;
      maxRange: number;
      noise: number;
      seed: number;
      track: boolean; // also run the tracker on this scan
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
  engineMs: number; // simulate (+ track) time inside the worker
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
  | { type: "reset-done" }
  | Reply
  | { type: "error"; id?: number; message: string };
