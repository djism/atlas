import * as THREE from "three";

// Many small dots as one instanced mesh. Used instead of THREE.Points because
// point sprites render inconsistently across GPUs (size limits, missing sprites).
export class DotCloud {
  readonly mesh: THREE.InstancedMesh;
  private readonly m = new THREE.Matrix4();
  private readonly hidden = new THREE.Matrix4().makeScale(0, 0, 0);
  private readonly c = new THREE.Color();

  constructor(capacity: number, geometry: THREE.BufferGeometry, color: number) {
    this.mesh = new THREE.InstancedMesh(geometry, new THREE.MeshBasicMaterial({ color: 0xffffff }), capacity);
    this.mesh.frustumCulled = false;
    for (let i = 0; i < capacity; i++) {
      this.mesh.setMatrixAt(i, this.hidden);
      this.mesh.setColorAt(i, this.c.setHex(color));
    }
  }

  static flat(capacity: number, radius: number, color: number): DotCloud {
    return new DotCloud(capacity, new THREE.CircleGeometry(radius, 12).rotateX(-Math.PI / 2), color);
  }

  static round(capacity: number, radius: number, color: number): DotCloud {
    return new DotCloud(capacity, new THREE.IcosahedronGeometry(radius, 0), color);
  }

  set(i: number, x: number, y: number, z: number): void {
    this.mesh.setMatrixAt(i, this.m.makeTranslation(x, y, z));
  }

  hide(i: number): void {
    this.mesh.setMatrixAt(i, this.hidden);
  }

  color(i: number, r: number, g: number, b: number): void {
    this.mesh.setColorAt(i, this.c.setRGB(r, g, b, THREE.SRGBColorSpace));
  }

  commit(colors = false): void {
    this.mesh.instanceMatrix.needsUpdate = true;
    if (colors && this.mesh.instanceColor) this.mesh.instanceColor.needsUpdate = true;
  }
}
