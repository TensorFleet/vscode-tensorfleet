import * as THREE from "three";

/**
 * Manages layer membership for THREE.js Object3D instances.
 * This class provides robust tracking of layer membership by patching
 * the THREE.js Object3D prototype to automatically track when objects
 * are added to or removed from the scene.
 */
export class LayerMembershipManager {
  private layerObjects: Map<number, Set<THREE.Object3D>> = new Map();

  constructor() {
    this.patchObject3DPrototype();
  }

  /**
   * Add an object to a specific layer
   * @param {THREE.Object3D} object - The object to add to the layer
   * @param {number} layer - The layer number (0-31)
   */
  public addToLayer(object: THREE.Object3D, layer: number): void {
    if (!this.layerObjects.has(layer)) {
      this.layerObjects.set(layer, new Set());
    }

    const layerSet = this.layerObjects.get(layer)!;
    if (layerSet.has(object)) {
      return;
    }

    layerSet.add(object);
    this.updateLayerMembership(object, layer, true);
  }

  /**
   * Remove an object from a specific layer
   * @param {THREE.Object3D} object - The object to remove from the layer
   * @param {number} layer - The layer number (0-31)
   */
  public removeFromLayer(object: THREE.Object3D, layer: number): void {
    const layerSet = this.layerObjects.get(layer);
    if (!layerSet || !layerSet.has(object)) {
      return;
    }

    layerSet.delete(object);
    this.updateLayerMembership(object, layer, false);
  }

  /**
   * Update layer membership for an object
   * @param {THREE.Object3D} object - The object to update
   * @param {number} layer - The layer number (0-31)
   * @param {boolean} enable - Whether to enable the layer
   */
  public updateLayerMembership(object: THREE.Object3D, layer: number, enable: boolean): void {
    if (enable) {
      // Add to layer
      object.traverse((child) => {
        child.layers.enable(layer);
      });
    } else {
      // Remove from layer
      object.traverse((child) => {
        child.layers.disable(layer);
      });
    }
  }

  /**
   * Check if an object is currently in a specific layer
   * @param {THREE.Object3D} object - The object to check
   * @param {number} layer - The layer number (0-31)
   * @returns {boolean} True if the object is in the layer
   */
  public isInLayer(object: THREE.Object3D, layer: number): boolean {
    const layerSet = this.layerObjects.get(layer);
    return layerSet ? layerSet.has(object) : false;
  }

  /**
   * Get all objects in a specific layer
   * @param {number} layer - The layer number (0-31)
   * @returns {Set<THREE.Object3D>} Set of objects in the layer
   */
  public getObjectsInLayer(layer: number): Set<THREE.Object3D> {
    return this.layerObjects.get(layer) || new Set();
  }

  /**
   * Clear all layer tracking
   */
  public clear(): void {
    this.layerObjects.clear();
  }

  /**
   * Patch THREE.js Object3D prototype to track scene membership changes
   * This ensures that when objects are added to or removed from the scene,
   * we can automatically update their layer membership.
   */
  private patchObject3DPrototype(): void {
    // Store original methods
    const originalAdd = THREE.Object3D.prototype.add;
    const originalRemove = THREE.Object3D.prototype.remove;
    const layerManager = this;

    // Patch add method
    THREE.Object3D.prototype.add = function (...objects: THREE.Object3D[]) {
      // Call original method
      const result = originalAdd.apply(this, objects);
      
      // Update layer membership for any objects that are in any layer
      objects.forEach(obj => {
        layerManager.layerObjects.forEach((layerSet, layer) => {
          if (layerSet.has(obj)) {
            layerManager.updateLayerMembership(obj, layer, true);
          }
        });
      });
      
      return result;
    };

    // Patch remove method
    THREE.Object3D.prototype.remove = function (...objects: THREE.Object3D[]) {
      // Update layer membership for any objects that are in any layer
      objects.forEach(obj => {
        layerManager.layerObjects.forEach((layerSet, layer) => {
          if (layerSet.has(obj)) {
            layerManager.updateLayerMembership(obj, layer, false);
          }
        });
      });
      
      // Call original method
      return originalRemove.apply(this, objects);
    };
  }
}