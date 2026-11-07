import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { engine } from "../engine/client";
import { FIRST_POSE, type IcpDemo } from "../icp/IcpDemo";
import { wrapAngle } from "../lib/frames";
import type { Scenario } from "../scenarios/types";
import { flattenSegments, furnitureSegments, insideFurniture } from "../sim/geometry";
import type { Furniture, Point, Pose } from "../types";
import { DotCloud } from "./DotCloud";
import { color } from "./palette";

export type FieldMode = "hero" | "match" | "lost" | "plan";
export type DriveState = "planning" | "driving" | "stuck" | "no-path";

export interface FieldStats {
  engine: "loading" | "ready" | "error";
  drive: DriveState;
  goal: number;
  skipped: number[];
  plans: number;
  lastPlanMs: number;
  expanded: number;
  pose: Pose; // true pose (answer key)
  lost: boolean;
  lostScans: number;
  error: number | null; // estimate vs answer key, meters (lost mode)
  driven: number; // meters since the tracker started
  scanMs: number; // engine time for the last scan (+ tracking)
}

export interface FieldOptions {
  scenario: Scenario;
  mode: FieldMode;
  icp?: IcpDemo; // required for "match"
  insetLeft?: () => number; // pixels covered by a floating panel on the left
  onStats?: (stats: FieldStats) => void;
}

const BEAMS = 360;
const GLOW = 24; // scans kept as fading afterglow
const SCAN_MS = 100; // 10 Hz, like a real 2D LiDAR
const SPEED = 0.6; // m/s
const TURN_RATE = 3.2; // rad/s
const BOX = 0.5;
const GRID_RESOLUTION = 0.05;
const MAX_LINE = 900;
const reduced = typeof matchMedia !== "undefined" && matchMedia("(prefers-reduced-motion: reduce)").matches;

// The 3D view and the simulated world behind it. The world (true robot motion,
// furniture) is simulated here; scans, planning, tracking and ICP all come from
// the C++ engine in the worker.
export class Field {
  private readonly scenario: Scenario;
  private readonly opts: FieldOptions;
  private readonly W: number;
  private readonly H: number;
  mode: FieldMode;

  // ---- world (answer key) ----
  furniture: Furniture[];
  private robot: Pose;
  private path: Point[] = [];
  private pi = 0;
  private goal = 1;
  private drive: DriveState = "planning";
  private skipped: number[] = [];
  private dirty = true;
  private planning = false;
  private lastPlanAt = 0;
  private plans = 0;
  private lastPlanMs = 0;
  private expanded = 0;
  private obstacleCache: Float64Array | null = null;
  private boxCount = 0;

  // ---- sensing / tracking ----
  range = 8;
  noise = 0.01;
  private engineState: FieldStats["engine"] = "loading";
  private scanInFlight = false;
  private scanAt = 0;
  private seed = 1;
  private generation = 0;
  private needsTrackerReset = false;
  private trackerRunning = false;
  private estimate: Pose | null = null;
  private shownEstimate: Pose | null = null;
  private lost = false;
  private lostScans = 0;
  private error: number | null = null;
  private driven = 0;
  private scanMs = 0;
  private glowSlot = 0;

  // ---- three.js ----
  private readonly host: HTMLElement;
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(30, 1, 0.1, 120);
  private readonly controls: OrbitControls;
  private readonly furnMeshes = new Map<string, THREE.Group>();
  private readonly pickables: THREE.Mesh[] = [];
  private readonly robotMesh = new THREE.Group();
  private readonly ghost = new THREE.Group();
  private readonly fanGeo = new THREE.BufferGeometry();
  private readonly fan: THREE.Mesh;
  private readonly glowAge = new Float32Array(GLOW).fill(GLOW);
  private readonly glow = DotCloud.round(BEAMS * GLOW, 0.028, color.signal);
  private readonly trueTrail: THREE.Line;
  private readonly estTrail: THREE.Line;
  private readonly pathLine: THREE.Line;
  private readonly trailPts: Point[] = [];
  private estPts: Point[] = [];
  private readonly goalRing: THREE.Mesh;
  private readonly patrolDots: THREE.Mesh[] = [];
  private readonly icpGroup = new THREE.Group();
  private icpB: DotCloud | null = null;
  private icpPairs: THREE.LineSegments | null = null;
  private readonly tip: HTMLDivElement;
  private view: "3d" | "top" = "3d";
  private camAnim: { from: THREE.Vector3; to: THREE.Vector3; fromT: THREE.Vector3; toT: THREE.Vector3; t0: number; dur: number } | null = null;
  private inset = 0;

  // ---- interaction & lifecycle ----
  private readonly raycaster = new THREE.Raycaster();
  private readonly floorPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  private readonly ndc = new THREE.Vector2();
  private down: { x: number; y: number; moved: boolean } | null = null;
  private drag: { group: THREE.Group; obj: Furniture; dx: number; dy: number } | null = null;
  private icpDrag: Point | null = null;
  private hovered: THREE.Group | null = null;
  private visible = true;
  private raf = 0;
  private last = performance.now();
  private lastStats = 0;
  private pathDrawnAt = 0;
  private readonly cleanups: (() => void)[] = [];

  constructor(host: HTMLElement, opts: FieldOptions) {
    this.host = host;
    this.opts = opts;
    this.scenario = opts.scenario;
    this.mode = opts.mode;
    this.W = opts.scenario.bounds.width;
    this.H = opts.scenario.bounds.height;
    this.furniture = opts.scenario.furniture.map((f) => ({ ...f }));
    const [a, b] = opts.scenario.patrol;
    this.robot = { x: a.x, y: a.y, theta: Math.atan2(b.y - a.y, b.x - a.x) };

    const R = (this.renderer = new THREE.WebGLRenderer({ antialias: true }));
    R.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    R.shadowMap.enabled = true;
    R.shadowMap.type = THREE.PCFSoftShadowMap;
    R.setClearColor(color.mist);
    host.appendChild(R.domElement);

    const controls = (this.controls = new OrbitControls(this.camera, R.domElement));
    Object.assign(controls, {
      enableDamping: true, dampingFactor: 0.08, enablePan: false, enableZoom: false,
      minPolarAngle: 0, maxPolarAngle: 1.18, rotateSpeed: 0.6,
    });

    this.buildStatic();
    this.syncFurniture();

    // robot: a low disc with a lidar turret and a blue nose
    const mat = (c: number, roughness = 0.5) => new THREE.MeshStandardMaterial({ color: c, roughness });
    const body = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.18, 0.085, 48), mat(color.ink));
    body.position.y = 0.05;
    body.castShadow = true;
    const top = new THREE.Mesh(new THREE.CylinderGeometry(0.165, 0.165, 0.004, 48), mat(0x26292f, 0.4));
    top.position.y = 0.095;
    const turret = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.04, 32), mat(0xf4f5f7, 0.3));
    turret.position.y = 0.115;
    turret.castShadow = true;
    const nose = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.02, 0.035), new THREE.MeshBasicMaterial({ color: color.signal }));
    nose.position.set(0.155, 0.075, 0);
    this.robotMesh.add(body, top, turret, nose);
    this.scene.add(this.robotMesh);

    // estimate ghost: where the tracker believes the robot is
    const ring = new THREE.Mesh(new THREE.RingGeometry(0.17, 0.205, 48), new THREE.MeshBasicMaterial({ color: color.signal, side: THREE.DoubleSide }));
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.012;
    const heading = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.004, 0.035), new THREE.MeshBasicMaterial({ color: color.signal }));
    heading.position.set(0.11, 0.012, 0);
    this.ghost.add(ring, heading);
    this.scene.add(this.ghost);

    // lidar: visible-area fan plus returns with a fading afterglow
    this.fanGeo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(BEAMS * 9), 3));
    this.fan = new THREE.Mesh(this.fanGeo, new THREE.MeshBasicMaterial({ color: color.signal, transparent: true, opacity: 0.1, depthWrite: false, side: THREE.DoubleSide }));
    this.fan.position.y = 0.006;
    this.fan.frustumCulled = false;
    this.scene.add(this.fan);
    this.scene.add(this.glow.mesh);

    this.trueTrail = this.makeLine(color.ink, false, 0.35);
    this.estTrail = this.makeLine(color.signal, true, 0.95);
    this.pathLine = this.makeLine(color.ink, true, 0.8);
    this.goalRing = new THREE.Mesh(new THREE.RingGeometry(0.13, 0.17, 40), new THREE.MeshBasicMaterial({ color: color.signal, side: THREE.DoubleSide }));
    this.goalRing.rotation.x = -Math.PI / 2;
    this.scene.add(this.goalRing);
    const dotGeo = new THREE.CircleGeometry(0.045, 20);
    const dotMat = new THREE.MeshBasicMaterial({ color: 0xa3a9b2 });
    for (const p of this.scenario.patrol) {
      const d = new THREE.Mesh(dotGeo, dotMat);
      d.rotation.x = -Math.PI / 2;
      d.position.copy(this.w2t(p.x, p.y, 0.004));
      this.patrolDots.push(d);
      this.scene.add(d);
    }
    this.scene.add(this.icpGroup);

    this.tip = document.createElement("div");
    this.tip.className = "field-tip";
    host.appendChild(this.tip);

    const el = R.domElement;
    const on = <K extends keyof HTMLElementEventMap>(t: K, f: (e: HTMLElementEventMap[K]) => void) => {
      el.addEventListener(t, f);
      this.cleanups.push(() => el.removeEventListener(t, f));
    };
    on("pointerdown", (e) => this.onDown(e));
    on("pointermove", (e) => this.onMove(e));
    on("pointerup", (e) => this.onUp(e));
    on("pointercancel", (e) => this.onUp(e));
    on("pointerleave", () => {
      this.tip.classList.remove("on");
      this.hover(null);
    });
    on("dblclick", () => {
      if (this.mode !== "match") this.setView(this.view === "top" ? "3d" : "top");
    });

    const ro = new ResizeObserver(() => this.resize());
    ro.observe(host);
    const io = new IntersectionObserver((es) => (this.visible = es[0].isIntersecting), { rootMargin: "120px" });
    io.observe(host);
    this.cleanups.push(() => ro.disconnect(), () => io.disconnect());

    const e = engine();
    e.ready.then(() => {
      if (this.engineState !== "error") this.engineState = "ready";
    });
    this.cleanups.push(e.onError(() => (this.engineState = "error")));

    this.resize();
    this.applyMode(true);
    this.raf = requestAnimationFrame(this.loop);
  }

  dispose(): void {
    cancelAnimationFrame(this.raf);
    this.cleanups.forEach((c) => c());
    this.controls.dispose();
    this.renderer.dispose();
    this.renderer.domElement.remove();
    this.tip.remove();
  }

  // ===================== public controls =====================
  setMode(mode: FieldMode): void {
    if (mode === this.mode) return;
    this.mode = mode;
    this.applyMode(false);
  }

  setView(v: "3d" | "top", instant = false): void {
    this.view = v;
    const to = this.viewPosition(v);
    const f = this.focus();
    const target = this.w2t(f.cx, f.cy);
    if (instant || reduced) {
      this.camera.position.copy(to);
      this.controls.target.copy(target);
      this.controls.update();
      this.camAnim = null;
      return;
    }
    this.camAnim = { from: this.camera.position.clone(), to, fromT: this.controls.target.clone(), toT: target, t0: performance.now(), dur: 900 };
  }

  get currentView(): "3d" | "top" {
    return this.view;
  }

  setSensor(range: number, noise: number): void {
    this.range = range;
    this.noise = noise;
  }

  restartTracker(): void {
    this.generation++;
    this.needsTrackerReset = true;
    this.trackerRunning = false;
    this.estimate = this.shownEstimate = null;
    this.estPts = [];
    this.setLine(this.estTrail, this.estPts);
    this.lostScans = 0;
    this.driven = 0;
    this.error = null;
    this.lost = false;
  }

  dropRandomBox(): void {
    for (let k = 0; k < 60; k++) {
      const p = { x: 0.6 + Math.random() * (this.W - 1.2), y: 0.6 + Math.random() * (this.H - 1.2) };
      if (Math.abs(p.x - 6) < 0.5) continue;
      if (this.furniture.some((f) => insideFurniture(p, f, 0.45))) continue;
      if (this.addBox(p)) return;
    }
  }

  clearBoxes(): void {
    this.furniture = this.furniture.filter((f) => f.kind !== "box");
    this.worldChanged();
  }

  resetRoom(): void {
    this.furniture = this.scenario.furniture.map((f) => ({ ...f }));
    this.worldChanged();
  }

  // ===================== setup =====================
  private w2t(x: number, y: number, h = 0): THREE.Vector3 {
    return new THREE.Vector3(x - this.W / 2, h, -(y - this.H / 2));
  }

  private buildStatic(): void {
    const S = this.scene;
    S.add(new THREE.HemisphereLight(0xffffff, 0xc9ced6, 0.68 * Math.PI));
    const sun = new THREE.DirectionalLight(0xffffff, 0.95 * Math.PI);
    sun.position.set(-5, 13, 7);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    Object.assign(sun.shadow.camera, { left: -9, right: 9, top: 7, bottom: -7, near: 1, far: 40 });
    sun.shadow.bias = -0.0004;
    S.add(sun);

    const ground = new THREE.Mesh(new THREE.PlaneGeometry(80, 80), new THREE.ShadowMaterial({ opacity: 0.06 }));
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.02;
    ground.receiveShadow = true;
    S.add(ground);
    const floor = new THREE.Mesh(new THREE.BoxGeometry(this.W + 0.1, 0.06, this.H + 0.1), new THREE.MeshStandardMaterial({ color: color.paper, roughness: 1 }));
    floor.position.y = -0.03;
    floor.receiveShadow = true;
    S.add(floor);

    const gp: number[] = [];
    for (let x = 0.5; x < this.W; x += 0.5) gp.push(...this.w2t(x, 0, 0.002).toArray(), ...this.w2t(x, this.H, 0.002).toArray());
    for (let y = 0.5; y < this.H; y += 0.5) gp.push(...this.w2t(0, y, 0.002).toArray(), ...this.w2t(this.W, y, 0.002).toArray());
    const gg = new THREE.BufferGeometry();
    gg.setAttribute("position", new THREE.Float32BufferAttribute(gp, 3));
    S.add(new THREE.LineSegments(gg, new THREE.LineBasicMaterial({ color: 0xedeff2 })));

    // walls: matte white with a crisp black cap, like an architectural cutaway
    const wallMat = new THREE.MeshStandardMaterial({ color: 0xfbfbfc, roughness: 0.95 });
    const capMat = new THREE.MeshBasicMaterial({ color: color.ink });
    for (const w of this.scenario.walls) {
      const len = Math.hypot(w.b.x - w.a.x, w.b.y - w.a.y) + 0.1;
      const mid = this.w2t((w.a.x + w.b.x) / 2, (w.a.y + w.b.y) / 2);
      const ang = Math.atan2(w.b.y - w.a.y, w.b.x - w.a.x);
      const m = new THREE.Mesh(new THREE.BoxGeometry(len, 0.55, 0.1), wallMat);
      m.position.set(mid.x, 0.275, mid.z);
      m.rotation.y = ang;
      m.castShadow = m.receiveShadow = true;
      const cap = new THREE.Mesh(new THREE.BoxGeometry(len, 0.012, 0.1), capMat);
      cap.position.set(mid.x, 0.556, mid.z);
      cap.rotation.y = ang;
      S.add(m, cap);
    }
  }

  private makeLine(c: number, dashed: boolean, opacity: number): THREE.Line {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(3 * MAX_LINE), 3));
    g.setDrawRange(0, 0);
    const m = dashed
      ? new THREE.LineDashedMaterial({ color: c, dashSize: 0.12, gapSize: 0.08, transparent: true, opacity })
      : new THREE.LineBasicMaterial({ color: c, transparent: true, opacity });
    const line = new THREE.Line(g, m);
    line.frustumCulled = false;
    this.scene.add(line);
    return line;
  }

  private setLine(line: THREE.Line, pts: Point[], h = 0.006): void {
    const a = line.geometry.attributes.position as THREE.BufferAttribute;
    const n = Math.min(MAX_LINE, pts.length);
    for (let i = 0; i < n; i++) {
      const v = this.w2t(pts[i].x, pts[i].y, h);
      a.setXYZ(i, v.x, v.y, v.z);
    }
    a.needsUpdate = true;
    line.geometry.setDrawRange(0, n);
    if ((line.material as THREE.Material & { isLineDashedMaterial?: boolean }).isLineDashedMaterial) line.computeLineDistances();
  }

  private syncFurniture(): void {
    const ids = new Set(this.furniture.map((f) => f.id));
    for (const [id, g] of this.furnMeshes) {
      if (ids.has(id)) continue;
      this.scene.remove(g);
      const i = this.pickables.indexOf(g.userData.mesh);
      if (i >= 0) this.pickables.splice(i, 1);
      this.furnMeshes.delete(id);
    }
    for (const f of this.furniture) {
      let g = this.furnMeshes.get(f.id);
      if (!g) {
        g = new THREE.Group();
        const box = f.kind === "box";
        const m = new THREE.Mesh(
          new THREE.BoxGeometry(f.w, f.height, f.h),
          new THREE.MeshStandardMaterial({ color: box ? color.ink : color.furniture, roughness: box ? 0.6 : 0.9 }),
        );
        m.position.y = f.height / 2;
        m.castShadow = m.receiveShadow = true;
        g.add(m);
        if (!box) {
          const edges = new THREE.LineSegments(new THREE.EdgesGeometry(m.geometry), new THREE.LineBasicMaterial({ color: color.ink, transparent: true, opacity: 0.55 }));
          edges.position.y = f.height / 2;
          g.add(edges);
        }
        g.userData = { id: f.id, kind: f.kind, mesh: m };
        m.userData.group = g;
        this.pickables.push(m);
        this.scene.add(g);
        this.furnMeshes.set(f.id, g);
        if (box && !reduced) g.userData.dropAt = performance.now();
      }
      const p = this.w2t(f.x, f.y);
      g.position.x = p.x;
      g.position.z = p.z;
      g.rotation.y = f.angle;
    }
  }

  // ===================== modes & camera =====================
  private applyMode(instant: boolean): void {
    const m = this.mode;
    const match = m === "match";
    const planning = m === "plan" || m === "hero";
    for (const o of [this.robotMesh, this.fan, this.glow.mesh, this.trueTrail]) o.visible = !match;
    this.icpGroup.visible = match;
    this.ghost.visible = this.estTrail.visible = m === "lost";
    this.pathLine.visible = this.goalRing.visible = planning;
    this.patrolDots.forEach((d) => (d.visible = planning));
    this.furnMeshes.forEach((g) => (g.visible = !match || g.userData.kind === "furniture")); // boxes hide while matching
    this.controls.enabled = !match;
    if (m === "lost") this.restartTracker();
    else this.generation++; // ignore scans in flight from the previous mode
    if (match) this.buildIcp();
    this.tip.classList.remove("on");
    this.setView(match ? "top" : "3d", instant);
  }

  private focus() {
    return this.mode === "match"
      ? { cx: 3.2, cy: 4, w: 6.6, h: 8.2 }
      : { cx: this.W / 2, cy: this.H / 2, w: this.W, h: this.H };
  }

  private fitDistance(v: "3d" | "top"): number {
    const f = this.focus();
    const r = this.host.getBoundingClientRect();
    const w = Math.max(200, r.width - this.inset);
    const h = Math.max(200, r.height);
    const t = Math.tan(THREE.MathUtils.degToRad(this.camera.fov / 2));
    // The tilted view needs a little more room on narrow (phone) screens.
    const tilt = w / h < 1.4 ? 1.1 : 0.96;
    return Math.max((f.h + 1.6) / 2 / t, (f.w + 1.2) / 2 / (t * (w / h))) * (v === "top" ? 1 : tilt);
  }

  private viewPosition(v: "3d" | "top"): THREE.Vector3 {
    const f = this.focus();
    const d = this.fitDistance(v);
    const polar = v === "top" ? 0.0001 : 0.72;
    const az = v === "top" ? 0 : -0.32;
    return this.w2t(f.cx, f.cy).add(new THREE.Vector3(d * Math.sin(polar) * Math.sin(az), d * Math.cos(polar), d * Math.sin(polar) * Math.cos(az)));
  }

  private resize(): void {
    const r = this.host.getBoundingClientRect();
    if (r.width < 2 || r.height < 2) return;
    this.inset = this.opts.insetLeft?.() ?? 0;
    this.renderer.setSize(r.width, r.height, false);
    const fullWidth = r.width + this.inset;
    this.camera.aspect = fullWidth / r.height;
    // Shift the picture right so the floating panel does not cover the map.
    if (this.inset > 0) this.camera.setViewOffset(fullWidth, r.height, 0, 0, r.width, r.height);
    else this.camera.clearViewOffset();
    this.camera.updateProjectionMatrix();
    if (this.camAnim) return;
    const want = this.fitDistance(this.view);
    if (this.camera.position.lengthSq() < 1e-6) this.setView(this.view, true);
    else {
      const off = this.camera.position.clone().sub(this.controls.target);
      this.camera.position.copy(this.controls.target).add(off.multiplyScalar(want / off.length()));
    }
  }

  // ===================== world edits =====================
  private worldChanged(): void {
    this.obstacleCache = null;
    this.dirty = true;
    this.syncFurniture();
  }

  private obstacles(): Float64Array {
    this.obstacleCache ??= flattenSegments([...this.scenario.walls, ...this.furniture.flatMap(furnitureSegments)]);
    return this.obstacleCache;
  }

  private addBox(p: Point): boolean {
    const boxes = this.furniture.filter((f) => f.kind === "box").length;
    if (boxes >= 10) return false;
    if (Math.hypot(p.x - this.robot.x, p.y - this.robot.y) < 0.5) return false;
    if (p.x < 0.3 || p.y < 0.3 || p.x > this.W - 0.3 || p.y > this.H - 0.3) return false;
    this.boxCount++;
    this.furniture.push({ id: `box-${this.boxCount}`, label: `Box ${this.boxCount}`, x: p.x, y: p.y, w: BOX, h: BOX, angle: 0, height: BOX * 0.8, kind: "box" });
    this.worldChanged();
    return true;
  }

  // ===================== pointer =====================
  private floorAt(e: PointerEvent): Point | null {
    const r = this.renderer.domElement.getBoundingClientRect();
    this.ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    this.raycaster.setFromCamera(this.ndc, this.camera);
    const p = new THREE.Vector3();
    if (!this.raycaster.ray.intersectPlane(this.floorPlane, p)) return null;
    return { x: p.x + this.W / 2, y: -p.z + this.H / 2 };
  }

  private objectAt(e: PointerEvent): THREE.Group | null {
    if (this.mode === "match" || this.mode === "lost") return null;
    this.floorAt(e);
    const hit = this.raycaster.intersectObjects(this.pickables, false)[0];
    return hit ? (hit.object.userData.group as THREE.Group) : null;
  }

  private hover(g: THREE.Group | null): void {
    if (this.hovered === g) return;
    const paint = (grp: THREE.Group | null, c: number) => {
      const f = grp && this.furniture.find((x) => x.id === grp.userData.id);
      if (f && f.kind === "furniture") (grp!.userData.mesh.material as THREE.MeshStandardMaterial).color.setHex(c);
    };
    paint(this.hovered, color.furniture);
    this.hovered = g;
    paint(g, color.furnitureHover);
  }

  private onDown(e: PointerEvent): void {
    this.down = { x: e.clientX, y: e.clientY, moved: false };
    if (this.mode === "match") {
      const p = this.floorAt(e);
      if (!p || !this.opts.icp?.loaded) return;
      this.icpDrag = p;
      this.opts.icp.startDrag(performance.now());
      this.renderer.domElement.setPointerCapture(e.pointerId);
      this.host.dataset.cursor = "grabbing";
      return;
    }
    const g = this.objectAt(e);
    const p = g && this.floorAt(e);
    if (!g || !p) return;
    const obj = this.furniture.find((f) => f.id === g.userData.id);
    if (!obj) return;
    this.drag = { group: g, obj, dx: obj.x - p.x, dy: obj.y - p.y };
    this.controls.enabled = false;
    this.renderer.domElement.setPointerCapture(e.pointerId);
    this.host.dataset.cursor = "grabbing";
  }

  private onMove(e: PointerEvent): void {
    if (this.down && Math.hypot(e.clientX - this.down.x, e.clientY - this.down.y) > 5) this.down.moved = true;
    const p = this.floorAt(e);
    const r = this.host.getBoundingClientRect();
    this.tip.style.left = `${e.clientX - r.left}px`;
    this.tip.style.top = `${e.clientY - r.top}px`;

    if (this.mode === "match") {
      if (this.icpDrag && p) {
        this.opts.icp?.dragBy({ x: p.x - this.icpDrag.x, y: p.y - this.icpDrag.y });
        this.icpDrag = p;
      }
      this.showTip(this.icpDrag ? "Release to align" : "Drag to move the blue scan", this.icpDrag ? "grabbing" : "grab");
      return;
    }
    if (this.drag && p) {
      const o = this.drag.obj;
      const hw = o.w / 2;
      const hh = o.h / 2;
      o.x = Math.min(this.W - hw - 0.05, Math.max(hw + 0.05, p.x + this.drag.dx));
      o.y = Math.min(this.H - hh - 0.05, Math.max(hh + 0.05, p.y + this.drag.dy));
      this.drag.group.position.y = 0.04;
      this.worldChanged();
    }
    const g = this.drag ? this.drag.group : this.objectAt(e);
    this.hover(g);
    const obj = g && this.furniture.find((f) => f.id === g.userData.id);
    if (obj) {
      this.showTip(this.drag ? `${obj.label} · x ${obj.x.toFixed(2)} m, y ${obj.y.toFixed(2)} m` : `${obj.label} · drag to move`, this.drag ? "grabbing" : "grab");
    } else if (p && p.x >= 0 && p.y >= 0 && p.x <= this.W && p.y <= this.H) {
      const canDrop = this.mode === "plan" || this.mode === "hero";
      this.showTip(`x ${p.x.toFixed(2)} m · y ${p.y.toFixed(2)} m${canDrop ? " · click to drop a box" : ""}`, canDrop ? "drop" : "");
    } else {
      this.tip.classList.remove("on");
      this.host.dataset.cursor = "";
    }
  }

  private showTip(text: string, cursor: string): void {
    this.tip.textContent = text;
    this.tip.classList.add("on");
    this.host.dataset.cursor = cursor;
  }

  private onUp(e: PointerEvent): void {
    if (this.icpDrag) {
      this.icpDrag = null;
      this.opts.icp?.endDrag();
      this.host.dataset.cursor = "grab";
    } else if (this.drag) {
      this.drag.group.position.y = 0;
      this.drag = null;
      this.controls.enabled = true;
      this.host.dataset.cursor = "grab";
    } else if (this.down && !this.down.moved && (this.mode === "plan" || this.mode === "hero")) {
      const p = this.floorAt(e);
      if (p && !this.furniture.some((f) => insideFurniture(p, f, 0.05))) this.addBox(p);
    }
    this.down = null;
  }

  // ===================== engine: planning =====================
  private maybePlan(now: number): void {
    if (!this.dirty || this.planning || now - this.lastPlanAt < 90 || this.engineState !== "ready") return;
    this.planning = true;
    this.dirty = false;
    this.plan()
      .catch(() => {})
      .finally(() => {
        this.planning = false;
        this.lastPlanAt = performance.now();
      });
  }

  // Plans from the robot's TRUE position with the KNOWN floor plan (simplified),
  // trying patrol points in order until one is reachable.
  private async plan(): Promise<void> {
    const patrol = this.scenario.patrol;
    const obstacles = this.obstacles();
    const skipped: number[] = [];
    for (let k = 0; k < patrol.length; k++) {
      const res = await engine().plan({
        obstacles, width: this.W, height: this.H, resolution: GRID_RESOLUTION, robotRadius: this.scenario.robot.radius,
        start: { x: this.robot.x, y: this.robot.y }, goal: patrol[this.goal],
      });
      this.plans++;
      this.lastPlanMs = res.planMs;
      this.expanded = res.expanded;
      if (this.dirty) return; // the world changed meanwhile; plan again
      if (res.status === "found") {
        this.path = res.path;
        this.pi = 1;
        this.drive = "driving";
        this.skipped = skipped;
        return this.drawPath();
      }
      if (res.status === "start-blocked") {
        this.drive = "stuck";
        this.path = [];
        this.skipped = skipped;
        return this.drawPath();
      }
      skipped.push(this.goal);
      this.goal = (this.goal + 1) % patrol.length;
    }
    this.drive = "no-path";
    this.path = [];
    this.skipped = skipped;
    this.drawPath();
  }

  private drawPath(): void {
    const pts = this.path.length ? [{ x: this.robot.x, y: this.robot.y }, ...this.path.slice(this.pi)] : [];
    this.setLine(this.pathLine, pts, 0.008);
  }

  // Moves the simulated robot along the planned path (true motion).
  private driveStep(dt: number): number {
    if (this.drive !== "driving" || this.pi >= this.path.length) return 0;
    const r = this.robot;
    const target = this.path[this.pi];
    const dx = target.x - r.x;
    const dy = target.y - r.y;
    const dist = Math.hypot(dx, dy);
    if (dist < 0.03) {
      this.pi++;
      if (this.pi >= this.path.length) {
        this.goal = (this.goal + 1) % this.scenario.patrol.length;
        this.dirty = true;
        this.lastPlanAt = 0;
      }
      return 0;
    }
    const err = wrapAngle(Math.atan2(dy, dx) - r.theta);
    r.theta = wrapAngle(r.theta + Math.max(-TURN_RATE * dt, Math.min(TURN_RATE * dt, err)));
    const d = Math.min(dist, SPEED * dt * Math.max(0, Math.cos(err)) ** 2);
    r.x += d * Math.cos(r.theta);
    r.y += d * Math.sin(r.theta);
    return d;
  }

  // ===================== engine: scanning & tracking =====================
  private maybeScan(now: number): void {
    if (this.mode === "match" || this.scanInFlight || now - this.scanAt < SCAN_MS || this.engineState !== "ready") return;
    this.scanAt = now;
    this.scanInFlight = true;
    const pose = { ...this.robot };
    const gen = this.generation;
    const lostMode = this.mode === "lost";
    const maxRange = lostMode ? this.range : 8;
    const run = async () => {
      const e = engine();
      if (lostMode && this.needsTrackerReset) {
        // The tracker's only pose input: where it starts. Its first scan is taken there.
        this.needsTrackerReset = false;
        await e.reset(pose);
        if (gen !== this.generation) return;
        this.trackerRunning = true;
      }
      const res = await e.step({
        walls: this.obstacles(), truePose: pose, beams: BEAMS, maxRange,
        noise: lostMode ? this.noise : 0.005, seed: this.seed++, track: lostMode && this.trackerRunning,
      });
      if (gen !== this.generation) return;
      this.scanMs = res.engineMs;
      this.applyScan(pose, res.ranges, maxRange);
      if (res.estimate) {
        this.estimate = res.estimate;
        this.lost = res.lost;
        if (res.lost) this.lostScans++;
        this.error = Math.hypot(res.estimate.x - pose.x, res.estimate.y - pose.y);
        const last = this.estPts[this.estPts.length - 1];
        if (!last || Math.hypot(last.x - res.estimate.x, last.y - res.estimate.y) > 0.03) {
          this.estPts.push({ x: res.estimate.x, y: res.estimate.y });
          if (this.estPts.length > MAX_LINE) this.estPts.shift();
          this.setLine(this.estTrail, this.estPts);
        }
      }
    };
    run()
      .catch(() => {})
      .finally(() => (this.scanInFlight = false));
  }

  private applyScan(pose: Pose, ranges: Float64Array, maxRange: number): void {
    const n = ranges.length;
    const fp = this.fanGeo.attributes.position.array as Float32Array;
    const o = this.w2t(pose.x, pose.y);
    const slot = this.glowSlot;
    const end = (i: number) => {
      const a = pose.theta + (2 * Math.PI * (i % n)) / n;
      const r = Number.isFinite(ranges[i % n]) ? ranges[i % n] : maxRange;
      return this.w2t(pose.x + r * Math.cos(a), pose.y + r * Math.sin(a));
    };
    for (let i = 0; i < BEAMS; i++) {
      if (i < n) {
        const p0 = end(i);
        const p1 = end(i + 1);
        fp.set([o.x, 0, o.z, p0.x, 0, p0.z, p1.x, 0, p1.z], i * 9);
      }
      const gi = slot * BEAMS + i;
      if (i < n && Number.isFinite(ranges[i])) {
        const p = end(i);
        this.glow.set(gi, p.x, 0.11, p.z);
      } else this.glow.hide(gi);
    }
    this.fanGeo.attributes.position.needsUpdate = true;
    // afterglow: newest scan in signal blue, older scans fade toward the floor
    for (let s = 0; s < GLOW; s++) this.glowAge[s]++;
    this.glowAge[slot] = 0;
    for (let s = 0; s < GLOW; s++) {
      const k = Math.min(1, this.glowAge[s] / GLOW) ** 2;
      const [r, g, b] = [0.137 + (0.86 - 0.137) * k, 0.275 + (0.88 - 0.275) * k, 1 + (0.93 - 1) * k];
      for (let i = 0; i < BEAMS; i++) this.glow.color(s * BEAMS + i, r, g, b);
    }
    this.glow.commit(true);
    this.glowSlot = (slot + 1) % GLOW;
  }

  // ===================== ICP (match mode) =====================
  private buildIcp(): void {
    const icp = this.opts.icp;
    if (!icp?.loaded || this.icpB) return;
    const pts = (n: number) => {
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(n * 3), 3));
      return g;
    };
    const a = DotCloud.flat(icp.firstWorld.length, 0.034, color.ink);
    icp.firstWorld.forEach((p, i) => {
      const v = this.w2t(p.x, p.y, 0.03);
      a.set(i, v.x, v.y, v.z);
    });
    a.commit();
    const n = icp.second.length / 2;
    this.icpB = DotCloud.flat(n, 0.038, color.signal);
    this.icpPairs = new THREE.LineSegments(pts(n * 2), new THREE.LineBasicMaterial({ color: color.signal, transparent: true, opacity: 0.3 }));
    // a marker for where the first scan was taken
    const origin = new THREE.Mesh(new THREE.RingGeometry(0.1, 0.13, 32), new THREE.MeshBasicMaterial({ color: color.ink, side: THREE.DoubleSide }));
    origin.rotation.x = -Math.PI / 2;
    origin.position.copy(this.w2t(FIRST_POSE.x, FIRST_POSE.y, 0.02));
    this.icpPairs.frustumCulled = false;
    // Scan points lie on wall faces; draw this layer over the geometry so walls never hide it.
    [a.mesh, this.icpB.mesh, this.icpPairs].forEach((o, i) => {
      (o.material as THREE.Material).depthTest = false;
      o.renderOrder = 10 + i;
    });
    this.icpGroup.add(a.mesh, this.icpB.mesh, this.icpPairs, origin);
  }

  private updateIcp(now: number): void {
    const icp = this.opts.icp;
    if (!icp?.loaded) return;
    if (!this.icpB) this.buildIcp();
    if (!this.icpB || !this.icpPairs) return;
    const frame = icp.frame(now);
    const world = icp.secondInWorld(frame.pose);
    const b = this.icpB;
    const pr = this.icpPairs.geometry.attributes.position as THREE.BufferAttribute;
    world.forEach((p, i) => {
      const v = this.w2t(p.x, p.y, 0.035);
      b.set(i, v.x, v.y, v.z);
      if (frame.pairs) {
        const a = icp.firstWorld[frame.pairs[i]];
        const va = this.w2t(a.x, a.y, 0.03);
        pr.setXYZ(2 * i, v.x, v.y, v.z);
        pr.setXYZ(2 * i + 1, va.x, va.y, va.z);
      }
    });
    b.commit();
    pr.needsUpdate = true;
    this.icpPairs.visible = !!frame.pairs && !this.icpDrag;
  }

  // ===================== frame loop =====================
  private readonly loop = (now: number): void => {
    this.raf = requestAnimationFrame(this.loop);
    const dt = Math.min(0.05, (now - this.last) / 1000);
    this.last = now;
    if (!this.visible) return;

    if (this.camAnim) {
      const a = this.camAnim;
      const f = Math.min(1, (now - a.t0) / a.dur);
      const e = f < 0.5 ? 4 * f * f * f : 1 - Math.pow(-2 * f + 2, 3) / 2;
      this.camera.position.lerpVectors(a.from, a.to, e);
      this.controls.target.lerpVectors(a.fromT, a.toT, e);
      this.camera.lookAt(this.controls.target);
      if (f >= 1) this.camAnim = null;
    } else this.controls.update();

    if (this.mode === "match") this.updateIcp(now);
    else {
      this.maybePlan(now);
      const d = this.driveStep(dt);
      if (this.trackerRunning) this.driven += d;
      if (d > 0 && now - this.pathDrawnAt > 120) {
        this.pathDrawnAt = now;
        this.drawPath();
      }
      this.maybeScan(now);
      const r = this.robot;
      this.robotMesh.position.copy(this.w2t(r.x, r.y));
      this.robotMesh.rotation.y = r.theta;
      const last = this.trailPts[this.trailPts.length - 1];
      if (!last || Math.hypot(last.x - r.x, last.y - r.y) > 0.04) {
        this.trailPts.push({ x: r.x, y: r.y });
        if (this.trailPts.length > 400) this.trailPts.shift();
        this.setLine(this.trueTrail, this.trailPts);
      }
      if (this.mode === "lost" && this.estimate) {
        // ease the drawn ghost toward the latest 10 Hz estimate
        const s = this.shownEstimate ?? { ...this.estimate };
        const k = Math.min(1, dt * 14);
        s.x += (this.estimate.x - s.x) * k;
        s.y += (this.estimate.y - s.y) * k;
        s.theta += wrapAngle(this.estimate.theta - s.theta) * k;
        this.shownEstimate = s;
        this.ghost.visible = true;
        this.ghost.position.copy(this.w2t(s.x, s.y));
        this.ghost.rotation.y = s.theta;
      } else if (this.mode === "lost") this.ghost.visible = false;
      if (this.goalRing.visible) {
        const g = this.scenario.patrol[this.goal];
        this.goalRing.position.copy(this.w2t(g.x, g.y, 0.01));
        const s = reduced ? 1 : 1 + 0.18 * Math.sin(now / 260);
        this.goalRing.scale.set(s, s, s);
      }
    }
    for (const g of this.furnMeshes.values()) {
      const at = g.userData.dropAt as number | undefined;
      if (at === undefined) continue;
      const f = Math.min(1, (now - at) / 560);
      g.position.y = f < 0.7 ? 1.2 * (1 - (f / 0.7) ** 2) : 0.07 * Math.sin(((f - 0.7) / 0.3) * Math.PI);
      if (f >= 1) {
        g.position.y = 0;
        delete g.userData.dropAt;
      }
    }
    this.renderer.render(this.scene, this.camera);

    if (now - this.lastStats > 100 && this.opts.onStats) {
      this.lastStats = now;
      this.opts.onStats({
        engine: this.engineState, drive: this.drive, goal: this.goal, skipped: this.skipped, plans: this.plans,
        lastPlanMs: this.lastPlanMs, expanded: this.expanded, pose: { ...this.robot }, lost: this.lost,
        lostScans: this.lostScans, error: this.error, driven: this.driven, scanMs: this.scanMs,
      });
    }
  };
}
