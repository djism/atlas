import { engine } from "../engine/client";
import type { AlignResult, IcpStatus } from "../engine/protocol";
import { composePoses, formatLength, rotateIntoFrame, transformPoint, wrapAngle } from "../lib/frames";
import { homeScenario } from "../scenarios/home";
import { flattenSegments, furnitureSegments } from "../sim/geometry";
import type { Point, Pose } from "../types";

export type IcpMethod = "p2l" | "p2p";

// Two scans of one room corner. The first is taken at FIRST_POSE (world frame);
// the robot then moves by MOTION (second sensor pose in the first frame). The
// engine gets only the two scans and a guess; MOTION is the answer key.
export const FIRST_POSE: Pose = { x: 2.4, y: 2.3, theta: 0.3 };
export const MOTION: Pose = { x: 0.45, y: 0.2, theta: 0.25 };
const IDENTITY: Pose = { x: 0, y: 0, theta: 0 };
const PASS_MS = 450;
const HOLD_MS = 2600;

export interface IcpFrame {
  pose: Pose; // displayed second-sensor pose in the first frame
  pass: number; // completed passes shown
  passes: number; // total passes the engine ran
  pairs: Uint32Array | null; // nearest-neighbor pairs at this step
  done: boolean;
}

const lerpPose = (a: Pose, b: Pose, t: number): Pose => ({
  x: a.x + (b.x - a.x) * t,
  y: a.y + (b.y - a.y) * t,
  theta: a.theta + wrapAngle(b.theta - a.theta) * t,
});

// Replays one real engine ICP run (alignScans) pass by pass, and lets a visitor
// drag the second scan to a new initial guess.
export class IcpDemo {
  first = new Float64Array(0); // sensor axes
  second = new Float64Array(0); // sensor axes
  firstWorld: Point[] = [];
  method: IcpMethod = "p2l";
  guess: Pose = { ...IDENTITY };
  run: AlignResult | null = null;
  dragging = false;
  auto = true; // replay on a loop until the visitor takes over
  error: string | null = null;
  private playStart = 0;
  private request = 0;

  async load(): Promise<void> {
    const e = engine();
    await e.ready;
    const walls = flattenSegments([...homeScenario.walls, ...homeScenario.furniture.flatMap(furnitureSegments)]);
    const second = composePoses(FIRST_POSE, MOTION);
    const scan = (pose: Pose, seed: number) =>
      e.step({ walls, truePose: pose, beams: 180, maxRange: 6.5, noise: 0.008, seed, track: false, owner: 0,
        trueMotion: { x: 0, y: 0, theta: 0 }, wheels: false });
    const [a, b] = [await scan(FIRST_POSE, 101), await scan(second, 102)];
    this.first = a.points;
    this.second = b.points;
    this.firstWorld = this.points(this.first).map((p) => transformPoint(FIRST_POSE, p));
    await this.align();
  }

  get loaded(): boolean {
    return this.first.length > 0;
  }

  async align(): Promise<void> {
    const id = ++this.request;
    try {
      const run = await engine().align({
        first: this.first, second: this.second, guess: this.guess, pointToLine: this.method === "p2l",
      });
      if (id !== this.request) return; // a newer request superseded this one
      this.run = run;
      this.error = null;
      this.playStart = performance.now();
    } catch (e) {
      this.error = e instanceof Error ? e.message : String(e);
      this.run = null;
    }
  }

  frame(now: number): IcpFrame {
    const run = this.run;
    if (this.dragging || !run) return { pose: this.guess, pass: 0, passes: 0, pairs: null, done: false };
    const passes = run.poses.length - 1;
    // `now` is a frame timestamp (start of the frame); playStart is set when the
    // engine's answer arrives, which on a slow frame is AFTER that start. Never
    // let elapsed time go negative (pass -1 would index past the start).
    const t = Math.max(0, now - this.playStart);
    const k = Math.floor(t / PASS_MS);
    if (k >= passes) {
      if (this.auto && t > passes * PASS_MS + HOLD_MS) this.playStart = now;
      return { pose: run.poses[passes], pass: passes, passes, pairs: run.pairs[passes], done: true };
    }
    const f = Math.min(1, (t - k * PASS_MS) / (PASS_MS * 0.6));
    const eased = 1 - Math.pow(1 - f, 3);
    return { pose: lerpPose(run.poses[k], run.poses[k + 1], eased), pass: k, passes, pairs: run.pairs[k], done: false };
  }

  secondInWorld(pose: Pose): Point[] {
    const inWorld = composePoses(FIRST_POSE, pose);
    return this.points(this.second).map((p) => transformPoint(inWorld, p));
  }

  errors(pose: Pose): { position: number; heading: number } {
    return { position: Math.hypot(pose.x - MOTION.x, pose.y - MOTION.y), heading: Math.abs(wrapAngle(pose.theta - MOTION.theta)) };
  }

  rmseAt(pass: number): number | null {
    return this.run && pass > 0 ? this.run.rmse[pass - 1] ?? null : null;
  }

  startDrag(now: number): void {
    this.guess = { ...this.frame(now).pose };
    this.dragging = true;
    this.auto = false;
  }

  // Drag delta in world meters, applied to the guess in first-sensor axes.
  dragBy(world: Point): void {
    const d = rotateIntoFrame(FIRST_POSE, world);
    this.guess = { ...this.guess, x: this.guess.x + d.x, y: this.guess.y + d.y };
  }

  endDrag(): void {
    this.dragging = false;
    void this.align();
  }

  replay(): void {
    this.guess = { ...IDENTITY };
    this.auto = true;
    void this.align();
  }

  setMethod(method: IcpMethod): void {
    this.method = method;
    this.replay();
  }

  // Status chip: the engine's verdict, checked against the answer key.
  verdict(frame: IcpFrame): { tone: "run" | "ok" | "warn" | "bad"; text: string } {
    if (this.dragging) return { tone: "run", text: "Release to align" };
    if (!this.run) return { tone: "run", text: this.error ? "Engine error" : "Loading engine" };
    if (!frame.done) return { tone: "run", text: "Aligning" };
    const status: IcpStatus = this.run.status;
    const err = this.errors(frame.pose);
    const off = err.position > 0.02 || err.heading > 0.02;
    if (status === "converged") return off ? { tone: "warn", text: `Converged, but ${formatLength(err.position)} off` } : { tone: "ok", text: "Converged" };
    if (status === "iteration-limit") return { tone: "warn", text: "Hit the pass limit" };
    if (status === "degenerate") return { tone: "bad", text: "Degenerate: walls don't pin it down" };
    return { tone: "bad", text: "Too few pairs" };
  }

  private points(flat: Float64Array): Point[] {
    const out: Point[] = [];
    for (let i = 0; i + 1 < flat.length; i += 2) out.push({ x: flat[i], y: flat[i + 1] });
    return out;
  }
}
