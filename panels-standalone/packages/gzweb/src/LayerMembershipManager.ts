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
   */
  public updateLayerMembership(object: THREE.Object3D, layer: number, enable: boolean): void {
    if (enable) {
      object.traverse((child) => child.layers.enable(layer));
    } else {
      object.traverse((child) => child.layers.disable(layer));
    }
  }

  /**
   * Called when an object is added anywhere in the graph.
   *
   * Handles:
   * ✔ ancestor tracked → new subtree inherits layers
   * ✔ subtree contains tracked nodes → propagate down
   */
  public handleObjectAdd(object: THREE.Object3D) {
    if (!object) return;

    // accumulate tracked layers from ancestors
    const inheritedLayers: number[] = [];

    let parent = object.parent;
    while (parent) {
      this.layerObjects.forEach((set, layer) => {
        if (set.has(parent)) inheritedLayers.push(layer);
      });
      parent = parent.parent;
    }

    const inheritedUnique = [...new Set(inheritedLayers)];

    // single DFS traversal
    const stack: THREE.Object3D[] = [object];

    while (stack.length) {
      const node = stack.pop()!;

      // apply inherited layers
      inheritedUnique.forEach(layer => node.layers.enable(layer));

      // if node itself is tracked, propagate that layer downward
      this.layerObjects.forEach((set, layer) => {
        if (set.has(node)) {
          node.traverse(child => child.layers.enable(layer));
        }
      });

      for (const child of node.children) {
        stack.push(child);
      }
    }
  }

  /**
   * Called when an object is removed from the graph.
   *
   * Handles:
   * ✔ removing tracked objects from sets
   * ✔ removing orphaned tracked children
   * ✔ disabling removed layer bits from subtree
   */
  public handleObjectRemove(object: THREE.Object3D) {
    if (!object) return;

    object.traverse((node) => {
      this.layerObjects.forEach((layerSet, layer) => {
        if (!layerSet.has(node)) return;

        // remove node from tracking set
        layerSet.delete(node);

        // disable layer on node + descendants
        node.traverse(child => child.layers.disable(layer));
      });
    });
  }

  /**
   * Check if an object is currently in a specific layer
   */
  public isInLayer(object: THREE.Object3D, layer: number): boolean {
    const layerSet = this.layerObjects.get(layer);
    return layerSet ? layerSet.has(object) : false;
  }

  /**
   * Get all objects in a specific layer
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
}