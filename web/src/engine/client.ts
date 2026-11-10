import type { Pose } from "../types";
import type { AlignResult, FromEngine, PlanResult, Reply, ResetDone, StepResult, ToEngine } from "./protocol";

type Request<T extends ToEngine["type"]> = Omit<Extract<ToEngine, { type: T }>, "type" | "id">;

// Promise wrapper around the one engine worker the whole page shares. Several
// callers (hero field, lab field, figures) may each have a request in flight;
// the worker answers them in order. Callers keep their own request rate bounded
// by waiting for their previous reply.
export class EngineClient {
  private readonly worker: Worker;
  private nextId = 1;
  private readonly pending = new Map<number, { resolve: (r: Reply) => void; reject: (e: Error) => void }>();
  private readonly resetWaiters: ((done: ResetDone) => void)[] = [];
  private readonly errorListeners = new Set<(message: string) => void>();
  readonly ready: Promise<void>;
  failed: string | null = null;

  constructor() {
    this.worker = new Worker(new URL("./engine.worker.ts", import.meta.url), { type: "module" });
    let markReady: () => void = () => {};
    this.ready = new Promise((resolve) => (markReady = resolve));
    this.worker.onmessage = (event: MessageEvent<FromEngine>) => {
      const msg = event.data;
      if (msg.type === "ready") markReady();
      else if (msg.type === "reset-done") this.resetWaiters.shift()?.(msg);
      else if (msg.type === "error") {
        if (msg.id !== undefined && this.pending.has(msg.id)) {
          this.pending.get(msg.id)!.reject(new Error(msg.message));
          this.pending.delete(msg.id);
        } else this.fail(msg.message);
      } else {
        this.pending.get(msg.id)?.resolve(msg);
        this.pending.delete(msg.id);
      }
    };
    this.worker.onerror = (e) => this.fail(e.message || "engine worker crashed");
  }

  onError(listener: (message: string) => void): () => void {
    this.errorListeners.add(listener);
    if (this.failed) listener(this.failed);
    return () => this.errorListeners.delete(listener);
  }

  // Surveys the map from `walls` and restarts the tracker and the map localizer at `pose`.
  reset(pose: Pose, walls: Float64Array, survey: Pose[]): Promise<ResetDone> {
    const done = new Promise<ResetDone>((resolve) => this.resetWaiters.push(resolve));
    this.worker.postMessage({ type: "reset", pose, walls, survey } satisfies ToEngine);
    return done;
  }

  step(request: Request<"step">): Promise<StepResult> {
    return this.send({ type: "step", id: 0, ...request }) as Promise<StepResult>;
  }

  plan(request: Request<"plan">): Promise<PlanResult> {
    return this.send({ type: "plan", id: 0, ...request }) as Promise<PlanResult>;
  }

  align(request: Request<"align">): Promise<AlignResult> {
    return this.send({ type: "align", id: 0, ...request }) as Promise<AlignResult>;
  }

  private fail(message: string): void {
    this.failed = message;
    this.errorListeners.forEach((l) => l(message));
  }

  private send(message: Exclude<ToEngine, { type: "reset" }>): Promise<Reply> {
    const id = this.nextId++;
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.worker.postMessage({ ...message, id });
    });
  }
}

let shared: EngineClient | null = null;

// The page's single engine. Created on first use.
export function engine(): EngineClient {
  shared ??= new EngineClient();
  return shared;
}
