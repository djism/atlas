import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { color } from "../field/palette";
import { scanPoints, type ReplayFrames } from "../lib/replay";

export type CameraMode = "chase" | "top" | "orbit";
export interface Layers { map: boolean; scan: boolean; truth: boolean; kiss: boolean }

/**
 * Three.js view of one recorded drive. Everything is placed in KITTI's
 * Velodyne frame of scan 0 (x forward, y left, z up) inside a root group
 * rotated so z points up on screen. The map is Atlas's own (built from its
 * estimated poses), the scan is drawn where ATLAS believed the car was, and
 * the true and KISS-ICP paths are drawn for comparison.
 */
export class ReplayView {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(50, 1, 0.5, 4000);
  private readonly controls: OrbitControls;
  private readonly root = new THREE.Group(); // data frame: z up
  private readonly car = new THREE.Group();
  private readonly trueDot: THREE.Mesh;
  private map: THREE.Points | null = null;
  private scan: THREE.Points | null = null;
  private lines: { truth: THREE.Line; atlas: THREE.Line; kiss: THREE.Line } | null = null;
  private frames: ReplayFrames | null = null;
  private scans: ArrayBuffer | null = null;
  private frame = 0;
  private mode: CameraMode = "chase";
  private layers: Layers = { map: true, scan: true, truth: true, kiss: false };
  private snap = true; // jump the camera (no easing) on the next frame
  private raf = 0;
  private readonly observer: ResizeObserver;

  constructor(private readonly host: HTMLElement) {
    const R = (this.renderer = new THREE.WebGLRenderer({ antialias: true }));
    R.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    R.setClearColor(color.mist);
    host.appendChild(R.domElement);
    this.root.rotation.x = -Math.PI / 2;
    this.scene.add(this.root);

    const ring = new THREE.Mesh(new THREE.RingGeometry(1.1, 1.5, 48), new THREE.MeshBasicMaterial({ color: color.signal, side: THREE.DoubleSide }));
    const nose = new THREE.Mesh(new THREE.PlaneGeometry(2.4, 0.35), new THREE.MeshBasicMaterial({ color: color.signal, side: THREE.DoubleSide }));
    nose.position.x = 1.9;
    ring.position.z = nose.position.z = -1.7; // on the road: the sensor rides 1.73 m up
    this.car.add(ring, nose);
    this.car.matrixAutoUpdate = false;
    this.root.add(this.car);
    this.trueDot = new THREE.Mesh(new THREE.CircleGeometry(0.6, 32), new THREE.MeshBasicMaterial({ color: color.ink, side: THREE.DoubleSide }));
    this.root.add(this.trueDot);

    this.controls = new OrbitControls(this.camera, R.domElement);
    Object.assign(this.controls, { enableDamping: true, dampingFactor: 0.1, enabled: false, maxDistance: 600 });

    this.observer = new ResizeObserver(() => this.resize());
    this.observer.observe(host);
    this.resize();
    const loop = () => {
      this.raf = requestAnimationFrame(loop);
      this.render();
    };
    loop();
  }

  setDrive(frames: ReplayFrames, map: Float32Array, scans: ArrayBuffer): void {
    this.clearDrive();
    this.frames = frames;
    this.scans = scans;

    // Atlas's map: a quiet blue-gray, darker with height so buildings read.
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.BufferAttribute(map, 3));
    const colors = new Float32Array(map.length);
    const low = new THREE.Color(0xd9deee), high = new THREE.Color(0x8792b4), c = new THREE.Color(); // quiet, so the live scan stands out
    for (let i = 0; i < map.length; i += 3) {
      c.copy(low).lerp(high, Math.min(1, Math.max(0, (map[i + 2] + 2) / 5)));
      colors[i] = c.r; colors[i + 1] = c.g; colors[i + 2] = c.b;
    }
    geo.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    this.map = new THREE.Points(geo, new THREE.PointsMaterial({ size: 1.6, sizeAttenuation: false, vertexColors: true }));
    this.root.add(this.map);

    const scanGeo = new THREE.BufferGeometry();
    scanGeo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(frames.scanPoints * 3), 3));
    this.scan = new THREE.Points(scanGeo, new THREE.PointsMaterial({ color: color.signal, size: 4, sizeAttenuation: false }));
    this.scan.matrixAutoUpdate = false;
    this.scan.frustumCulled = false;
    this.root.add(this.scan);

    const line = (pts: number[][], c: number, opacity: number) => {
      const g = new THREE.BufferGeometry().setFromPoints(pts.map(([x, y, z]) => new THREE.Vector3(x, y, z - 1.73)));
      const l = new THREE.Line(g, new THREE.LineBasicMaterial({ color: c, transparent: opacity < 1, opacity }));
      l.frustumCulled = false;
      this.root.add(l);
      return l;
    };
    this.lines = {
      truth: line(frames.truthPos, color.ink, 0.8),
      kiss: line(frames.kissPos, color.baseline, 1),
      atlas: line(frames.atlasPose.map((p) => [p[3], p[7], p[11]]), color.signal, 1),
    };
    this.snap = true;
    this.applyLayers();
    this.setFrame(0);
  }

  setFrame(i: number): void {
    const f = this.frames;
    if (!f || !this.scans || !this.scan || !this.lines) return;
    this.frame = Math.min(f.frames - 1, Math.max(0, i));
    const pose = this.poseMatrix(this.frame);
    const attr = this.scan.geometry.getAttribute("position") as THREE.BufferAttribute;
    (attr.array as Float32Array).set(scanPoints(this.scans, this.frame, f.scanPoints));
    attr.needsUpdate = true;
    this.scan.matrix.copy(pose); // drawn where Atlas believed the sensor was
    this.car.matrix.copy(pose);
    const [tx, ty, tz] = f.truthPos[this.frame];
    this.trueDot.position.set(tx, ty, tz - 1.72);
    // Estimated paths grow as the replay plays; the answer key is drawn in full.
    this.lines.atlas.geometry.setDrawRange(0, this.frame + 1);
    this.lines.kiss.geometry.setDrawRange(0, this.frame + 1);
  }

  setMode(mode: CameraMode): void {
    this.mode = mode;
    this.controls.enabled = mode === "orbit";
    this.snap = true;
  }

  setLayers(layers: Layers): void {
    this.layers = layers;
    this.applyLayers();
  }

  dispose(): void {
    cancelAnimationFrame(this.raf);
    this.observer.disconnect();
    this.controls.dispose();
    this.clearDrive();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  private applyLayers(): void {
    if (this.map) this.map.visible = this.layers.map;
    if (this.scan) this.scan.visible = this.layers.scan;
    if (this.lines) {
      this.lines.truth.visible = this.layers.truth;
      this.lines.kiss.visible = this.layers.kiss;
    }
    this.trueDot.visible = this.layers.truth;
  }

  private clearDrive(): void {
    for (const o of [this.map, this.scan, ...(this.lines ? Object.values(this.lines) : [])]) {
      if (!o) continue;
      this.root.remove(o);
      o.geometry.dispose();
      (o.material as THREE.Material).dispose();
    }
    this.map = this.scan = null;
    this.lines = null;
  }

  /** Atlas's estimated pose at frame i as a 4x4 matrix (data frame). */
  private poseMatrix(i: number): THREE.Matrix4 {
    const p = this.frames!.atlasPose[i];
    return new THREE.Matrix4().set(p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], 0, 0, 0, 1);
  }

  private resize(): void {
    const w = Math.max(1, this.host.clientWidth), h = Math.max(1, this.host.clientHeight);
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = "100%";
    this.renderer.domElement.style.height = "100%";
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  private render(): void {
    if (this.frames && this.mode !== "orbit") {
      const pose = this.poseMatrix(this.frame);
      const toWorld = (x: number, y: number, z: number) => this.root.localToWorld(new THREE.Vector3(x, y, z).applyMatrix4(pose));
      let eye: THREE.Vector3, target: THREE.Vector3;
      if (this.mode === "chase") {
        eye = toWorld(-24, 0, 15);
        target = toWorld(14, 0, -2);
        this.camera.up.set(0, 1, 0);
      } else {
        // Top: straight down, the car's heading pointing up the screen.
        eye = toWorld(10, 0, 140);
        target = toWorld(10, 0, 0);
        this.camera.up.copy(toWorld(1, 0, 0).sub(toWorld(0, 0, 0)).normalize());
      }
      if (this.snap) {
        this.camera.position.copy(eye);
        this.controls.target.copy(target);
        this.snap = false;
      } else {
        this.camera.position.lerp(eye, 0.15);
        this.controls.target.lerp(target, 0.15);
      }
      this.camera.lookAt(this.controls.target);
    } else if (this.frames) {
      this.camera.up.set(0, 1, 0);
      if (this.snap) {
        this.controls.target.copy(this.car.getWorldPosition(new THREE.Vector3()));
        this.snap = false;
      }
      this.controls.update();
    }
    this.renderer.render(this.scene, this.camera);
  }
}
