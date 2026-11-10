/// <reference lib="webworker" />
// Runs the Atlas C++ engine (WebAssembly) off the main thread.
import createAtlasEngine, { type Localizer, type MainModule, type Tracker } from "./generated/atlas_engine.js";
import type { AlignResult, FromEngine, IcpStatus, MapStep, PlanResult, PlanStatus, StepResult, ToEngine } from "./protocol";
import type { Point, Pose } from "../types";

declare const self: DedicatedWorkerGlobalScope;

let engine: MainModule | null = null;
let tracker: Tracker | null = null;
let localizer: Localizer | null = null;
const MAP_RESOLUTION = 0.05; // meters
const SURVEY_RANGE = 20; // meters: the survey sees the whole house
const post = (message: FromEngine, transfer: Transferable[] = []) => self.postMessage(message, transfer);

const ready = createAtlasEngine()
  .then((module) => {
    engine = module;
    post({ type: "ready" });
  })
  .catch((e: unknown) => post({ type: "error", message: `engine failed to load: ${String(e)}` }));

function handle(e: MainModule, msg: ToEngine): void {
  if (msg.type === "reset") {
    const t0 = performance.now();
    tracker?.delete();
    localizer?.delete();
    tracker = localizer = null;
    const survey = Float64Array.from(msg.survey.flatMap((p) => [p.x, p.y, p.theta]));
    const map = e.buildMap(msg.walls, survey, MAP_RESOLUTION, SURVEY_RANGE);
    if (map.error) return post({ type: "error", message: `map: ${map.error}` });
    tracker = new e.Tracker(msg.pose.x, msg.pose.y, msg.pose.theta);
    try {
      localizer = new e.Localizer(map.points, msg.pose.x, msg.pose.y, msg.pose.theta);
    } catch (err) {
      return post({ type: "error", message: `localizer: ${String(err)}` });
    }
    const mapPoints: Float64Array = map.points;
    post({ type: "reset-done", mapPoints, mapMs: performance.now() - t0 }, [mapPoints.buffer]);
    return;
  }
  const start = performance.now();
  if (msg.type === "plan") {
    const plan = e.planRoute(msg.obstacles, msg.width, msg.height, msg.resolution, msg.robotRadius,
      msg.start.x, msg.start.y, msg.goal.x, msg.goal.y);
    if (plan.error) return post({ type: "error", id: msg.id, message: plan.error });
    const flat: Float64Array = plan.path;
    const path: Point[] = [];
    for (let i = 0; i + 1 < flat.length; i += 2) path.push({ x: flat[i], y: flat[i + 1] });
    const result: PlanResult = {
      type: "plan", id: msg.id, status: plan.status as PlanStatus, path,
      length: plan.length, expanded: plan.expanded, planMs: performance.now() - start,
    };
    return post(result);
  }
  if (msg.type === "align") {
    const r = e.alignScans(msg.first, msg.second, msg.guess.x, msg.guess.y, msg.guess.theta, msg.pointToLine);
    if (r.error) return post({ type: "error", id: msg.id, message: r.error });
    const flat: Float64Array = r.poses;
    const poses: Pose[] = [];
    for (let i = 0; i + 2 < flat.length; i += 3) poses.push({ x: flat[i], y: flat[i + 1], theta: flat[i + 2] });
    const n = msg.second.length / 2;
    const allPairs: Float64Array = r.pairs;
    const pairs = poses.map((_, k) => Uint32Array.from(allPairs.subarray(k * n, (k + 1) * n)));
    const result: AlignResult = {
      type: "align", id: msg.id, status: r.status as IcpStatus, iterations: r.iterations, poses,
      rmse: Array.from(r.rmse as Float64Array), pairs, engineMs: performance.now() - start,
    };
    return post(result);
  }
  const p = msg.truePose;
  const scan = e.simulateScan(msg.walls, p.x, p.y, p.theta, msg.beams, msg.maxRange, msg.noise, msg.seed);
  if (scan.error) return post({ type: "error", id: msg.id, message: scan.error });
  let estimate: Pose | null = null;
  let lost = false;
  let status: IcpStatus | null = null;
  let mapStep: MapStep | null = null;
  if (msg.track) {
    if (!tracker || !localizer) return post({ type: "error", id: msg.id, message: "tracking requested before reset" });
    const est = tracker.update(scan.points);
    if (est.error) return post({ type: "error", id: msg.id, message: est.error });
    estimate = { x: est.x, y: est.y, theta: est.theta };
    lost = est.lost;
    status = est.status as IcpStatus;
    const m = localizer.update(scan.points);
    if (m.error) return post({ type: "error", id: msg.id, message: m.error });
    mapStep = { estimate: { x: m.x, y: m.y, theta: m.theta }, lost: m.lost, weak: m.weak, inliers: m.inliers };
  }
  const result: StepResult = {
    type: "step", id: msg.id, ranges: scan.ranges, points: scan.points, estimate, lost, status,
    map: mapStep, engineMs: performance.now() - start,
  };
  post(result, [result.ranges.buffer, result.points.buffer]);
}

self.onmessage = async (event: MessageEvent<ToEngine>) => {
  await ready;
  if (engine) handle(engine, event.data);
};
