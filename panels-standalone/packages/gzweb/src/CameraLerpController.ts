import * as THREE from "three";
import type { OrbitControls } from "../include/OrbitControls";

export type FocusInput = THREE.Object3D | THREE.Object3D[];

export interface ViewSubRect {
  fullWidth: number;
  fullHeight: number;
  offsetX: number;
  offsetY: number;
  width: number;
  height: number;
}

interface Transition {
  active: boolean;

  startPos: THREE.Vector3;
  startQuat: THREE.Quaternion;
  startTarget: THREE.Vector3;

  endPos: THREE.Vector3;
  endQuat: THREE.Quaternion;
  endTarget: THREE.Vector3;

  duration: number;
  elapsed: number;
}

export class CameraLerpController {
  private camera: THREE.PerspectiveCamera;
  private controls: OrbitControls;

  private transition: Transition = {
    active: false,
    startPos: new THREE.Vector3(),
    startQuat: new THREE.Quaternion(),
    startTarget: new THREE.Vector3(),
    endPos: new THREE.Vector3(),
    endQuat: new THREE.Quaternion(),
    endTarget: new THREE.Vector3(),
    duration: 0,
    elapsed: 0,
  };

  private viewSubRect: ViewSubRect | null = null;

  private tmpBox = new THREE.Box3();
  private tmpBox2 = new THREE.Box3();
  private tmpSize = new THREE.Vector3();
  private tmpCenter = new THREE.Vector3();
  private tmpMat = new THREE.Matrix4();
  private tmpDir = new THREE.Vector3();

  constructor(camera: THREE.PerspectiveCamera, controls: OrbitControls) {
    this.camera = camera;
    this.controls = controls;
  }

  /**
   * @param view Sub-rectangle of the full view; pass null/undefined to clear.
   */
  setViewSubRect(view?: ViewSubRect | null) {
    if (view) {
      this.viewSubRect = {
        fullWidth: view.fullWidth,
        fullHeight: view.fullHeight,
        offsetX: view.offsetX,
        offsetY: view.offsetY,
        width: view.width,
        height: view.height,
      };
    } else {
      this.viewSubRect = null;
    }

    this.applyViewSubRect();
  }

  private applyViewSubRect() {
    const v = this.viewSubRect;

    if (v) {
      this.camera.setViewOffset(
        v.fullWidth,
        v.fullHeight,
        v.offsetX,
        v.offsetY,
        v.width,
        v.height
      );
    } else {
      this.camera.clearViewOffset();
    }

    this.camera.updateProjectionMatrix();
  }

  cancel() {
    this.transition.active = false;
  }

  moveTo(position: THREE.Vector3, lookAt: THREE.Vector3, duration = 600) {
    this.camera.getWorldPosition(this.transition.startPos);
    this.camera.getWorldQuaternion(this.transition.startQuat);
    this.transition.startTarget.copy(this.controls.target);

    this.transition.endPos.copy(position);
    this.transition.endTarget.copy(lookAt);

    this.tmpMat.lookAt(position, lookAt, this.camera.up);
    this.transition.endQuat.setFromRotationMatrix(this.tmpMat);

    this.transition.duration = Math.max(1, duration);
    this.transition.elapsed = 0;
    this.transition.active = true;
  }

  focus(objects: FocusInput, duration = 650, padding = 1.2) {
    const roots = Array.isArray(objects) ? objects : [objects];
    const box = this.computeRecursiveBounds(roots);

    if (box.isEmpty()) return;

    box.getCenter(this.tmpCenter);
    box.getSize(this.tmpSize);

    const radius = this.tmpSize.length() * 0.5 * padding;

    const fov = THREE.MathUtils.degToRad(this.camera.fov);

    let halfV = fov / 2;
    let halfH = Math.atan(Math.tan(halfV) * this.camera.aspect);

    if (this.viewSubRect) {
      const v = this.viewSubRect;

      const near = 1;
      const topFull = Math.tan(halfV) * near;
      const heightFull = 2 * topFull;
      const widthFull = heightFull * this.camera.aspect;

      let left = -0.5 * widthFull;
      let top = topFull;
      let width = widthFull;
      let height = heightFull;

      left += (v.offsetX / v.fullWidth) * widthFull;
      top -= (v.offsetY / v.fullHeight) * heightFull;
      width *= v.width / v.fullWidth;
      height *= v.height / v.fullHeight;

      const right = left + width;
      const bottom = top - height;

      const hx = Math.min(
        Math.atan(Math.max(1e-6, right) / near),
        Math.atan(Math.max(1e-6, -left) / near)
      );
      const hy = Math.min(
        Math.atan(Math.max(1e-6, top) / near),
        Math.atan(Math.max(1e-6, -bottom) / near)
      );

      halfH = Math.max(1e-6, hx);
      halfV = Math.max(1e-6, hy);
    }

    const distV = radius / Math.sin(halfV);
    const distH = radius / Math.sin(halfH);

    const distance = Math.max(distV, distH) * 2;

    this.tmpDir
      .copy(this.camera.position)
      .sub(this.controls.target)
      .normalize();

    if (this.tmpDir.lengthSq() === 0) this.tmpDir.set(1, 0, 0);

    const newPos = this.tmpCenter.clone().add(this.tmpDir.multiplyScalar(distance));

    this.camera.updateProjectionMatrix();

    this.moveTo(newPos, this.tmpCenter, duration);
  }

  update(deltaMs: number) {
    if (this.viewSubRect) this.applyViewSubRect();
    if (!this.transition.active) return;

    this.transition.elapsed += deltaMs;
    const t = Math.min(1, this.transition.elapsed / this.transition.duration);

    const ease = t < 0.5
      ? 4 * t * t * t
      : 1 - Math.pow(-2 * t + 2, 3) / 2;

    this.camera.position.lerpVectors(
      this.transition.startPos,
      this.transition.endPos,
      ease
    );

    THREE.Quaternion.slerp(
      this.transition.startQuat,
      this.transition.endQuat,
      this.camera.quaternion,
      ease
    );

    this.controls.target.lerpVectors(
      this.transition.startTarget,
      this.transition.endTarget,
      ease
    );

    this.camera.updateMatrixWorld(true);
    this.controls.update();

    if (t === 1) this.transition.active = false;
  }

  /**
   * Start a move_to_entity animation (legacy method for Scene compatibility)
   * @param startPos Starting position
   * @param endPos Ending position
   * @param endRotMat Ending rotation matrix
   */
  public startMoveTo(startPos: THREE.Vector3, endPos: THREE.Vector3, endRotMat: THREE.Matrix4): void {
    this.transition.startPos.copy(startPos);
    this.transition.endPos.copy(endPos);
    this.transition.startQuat.copy(this.camera.quaternion);
    this.transition.endQuat.setFromRotationMatrix(endRotMat);
    this.transition.elapsed = 0;
    this.transition.active = true;
  }

  private computeRecursiveBounds(objects: THREE.Object3D[]): THREE.Box3 {
    const box = this.tmpBox;
    box.makeEmpty();

    objects.forEach(root => {
      root.updateMatrixWorld(true);

      root.traverse(obj => {
        const mesh = obj as THREE.Mesh;
        if (!mesh.geometry) return;

        if (!mesh.geometry.boundingBox)
          mesh.geometry.computeBoundingBox();

        this.tmpBox2.copy(mesh.geometry.boundingBox!);
        this.tmpBox2.applyMatrix4(obj.matrixWorld);

        box.union(this.tmpBox2);
      });
    });

    return box;
  }
}