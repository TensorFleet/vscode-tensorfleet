import * as THREE from "three";
import type { OrbitControls } from "../include/OrbitControls";

export type FocusInput = THREE.Object3D | THREE.Object3D[];

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
    const hFov = 2 * Math.atan(Math.tan(fov / 2) * this.camera.aspect);

    const distV = radius / Math.sin(fov / 2);
    const distH = radius / Math.sin(hFov / 2);

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