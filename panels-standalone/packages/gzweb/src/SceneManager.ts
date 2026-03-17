import * as THREE from "three";
import { AudioTopic } from "./AudioTopic";
import { Publisher } from "./Publisher";
import { Scene } from "./Scene";
import { SDFParser } from "./SDFParser";
import { Shaders } from "./Shaders";
import { map, Observable, Subscription } from "rxjs";
import { Topic } from "./Topic";
import { Transport } from "./Transport";

/**
 * Interface used to pass arguments to the SceneManager constructor.
 */
export interface SceneManagerConfig {
  /**
   * ElementId is the id of the HTML element that will hold the rendering
   * context. If not specified, the id gz-scene will be used.
   */
  elementId?: string;

  /**
   * A websocket url that points to a Gazebo server.
   */
  websocketUrl?: string;

  /**
   * An authentication key for the websocket server.
   */
  websocketKey?: string;

  /**
   * The name of a an audio control topic, used to play audio files.
   */
  audioTopic?: string;

  /**
   * Name of the topic to advertise.
   */
  topicName?: string;

  /**
   * Message type of the topic to advertise.
   */
  msgType?: string;

  /**
   * Message data of the topic to advertise.
   */
  msgData?: any;

  /**
   * Whether or not lights in models are visible.
   */
  enableLights?: boolean;
}

type ManipulationCommitPayload = {
  name?: string;
  position?: { x?: number; y?: number; z?: number };
  orientation?: { x?: number; y?: number; z?: number; w?: number };
};

type ManipulationServiceReply = {
  ok: boolean | null;
  detail?: string;
};

type PendingManipulationPose = {
  requestId: string;
  world: string;
  requestedName: string;
  aliases: Set<string>;
  targetPosition: { x: number; y: number; z: number };
  expiresAtMs: number;
};

/**
 * SceneManager handles the interface between a Gazebo server and the
 * rendering scene. A user of gzweb will typically create a SceneManager and
 * then connect the SceneManager to a Gazebo server's websocket.
 *
 * This example will connect to a Gazebo server's websocket at WS_URL, and
 * start the rendering process. Rendering output will be placed in the HTML
 * element with the id ELEMENT_ID
 *
 * ```
 * let sceneMgr = new SceneManager(ELEMENT_ID, WS_URL, WS_KEY);
 * ```
 */
export class SceneManager {
  private static readonly POSE_MESSAGE_TYPE_CANDIDATES = ["gz.msgs.Pose"] as const;
  private static readonly MANIPULATION_POSE_LOCK_TIMEOUT_MS = 1500;
  private static readonly MANIPULATION_POSE_LOCK_TOLERANCE_METERS = 0.03;

  /**
   * Particle emitter updates.
   */
  private particleEmittersSubscription: Subscription;

  /**
   * Subscription for status updates.
   */
  private statusSubscription: Subscription;

  /**
   * Connection status from the Websocket.
   */
  private connectionStatus: string = "disconnected";

  /**
   * Scene Information updates.
   */
  private sceneInfoSubscription: Subscription;

  /**
   * Scene information obtained from the Websocket.
   */
  private sceneInfo: object;

  /**
   * Gz3D Scene.
   */
  private scene: any;

  /**
   * List of 3d models.
   */
  private models: any[] = [];

  /**
   * Map of model name to 3D object for quick lookup.
   */
  private modelMap: Map<string, any> = new Map();

  /**
   * A sun directional light for global illumination
   */
  private sunLight: object;

  /**
   * A Transport interface used to connect to a Gazebo server.
   */
  private transport = new Transport();

  /**
   * ID of the Request Animation Frame method. Required to cancel the animation.
   */
  private cancelAnimation: number;

  /**
   *
   */
  private previousRenderTimestampMs: number = 0;

  /**
   * The container of the Scene.
   */
  private sceneElement: HTMLElement;

  /**
   * Gz3D SDF parser.
   */
  private sdfParser: any;

  /**
   * Name of the HTML element that will hold the rendering scene.
   */
  private elementId: string = "gz-scene";

  /**
   * Name of an audio topic, which can be used to playback audio files.
   */
  private audioTopic: string;

  /*
   * Name of the topic to advertise.
   */
  private topicName: string;

  /*
   * Message type of the topic to advertise.
   */
  private msgType: string;

  /*
   * Message data of the topic to advertise.
   */
  private msgData: any;

  /*
   * Publisher object to publish to a topic
   */
  private publisher: Publisher;

  /*
   * Whether or not lights in models are visible. Enabled by default.
   */
  private enableLights: boolean = true;
  private manipulationDispatchCounter: number = 0;
  private pendingManipulationPoses: PendingManipulationPose[] = [];

  private readonly handleManipulationCommit = (payload: ManipulationCommitPayload): void => {
    const scene = this.scene;
    if (!scene?.emitter?.emit) {
      return;
    }

    const name = typeof payload?.name === "string" ? payload.name.trim() : "";
    const position = payload?.position;
    const orientation = payload?.orientation;
    const world = this.transport.getWorld();

    if (
      !name ||
      !world ||
      !position ||
      !orientation ||
      typeof position.x !== "number" ||
      typeof position.y !== "number" ||
      typeof position.z !== "number" ||
      typeof orientation.x !== "number" ||
      typeof orientation.y !== "number" ||
      typeof orientation.z !== "number" ||
      typeof orientation.w !== "number"
    ) {
      scene.emitter.emit("manipulation_dispatch", {
        ok: false,
        name,
        world,
        error: "Invalid manipulation commit payload",
      });
      return;
    }

    const requestId = `drag-${Date.now().toString(36)}-${(++this.manipulationDispatchCounter).toString(36)}`;
    const msgType = this.getOrderedPoseMessageTypes()[0];
    const serviceName = `/world/${world}/set_pose`;
    if (!msgType) {
      scene.emitter.emit("manipulation_dispatch", {
        ok: false,
        requestId,
        name,
        world,
        serviceName,
        error: "No compatible pose message type available",
      });
      return;
    }

    try {
      const request = {
        name,
        position,
        orientation,
      };
      this.trackPendingManipulationPose({
        requestId,
        world,
        requestedName: name,
        aliases: this.buildPoseAliases(world, name),
        targetPosition: {
          x: position.x,
          y: position.y,
          z: position.z,
        },
        expiresAtMs: Date.now() + SceneManager.MANIPULATION_POSE_LOCK_TIMEOUT_MS,
      });
      scene.emitter.emit("manipulation_dispatch", {
        ok: true,
        requestId,
        name,
        world,
        serviceName,
        msgType,
        position,
        orientation,
      });
      void this.transport
        .requestServiceWithResponse(serviceName, msgType, request)
        .then(({ msgType: responseType, response }) => {
          const normalizedReply = this.normalizeManipulationServiceReply(
            responseType,
            response,
          );
          scene.emitter.emit("manipulation_service_reply", {
            requestId,
            name,
            world,
            serviceName,
            requestType: msgType,
            responseType,
            ok: normalizedReply.ok,
            detail: normalizedReply.detail,
          });
        })
        .catch((error) => {
          scene.emitter.emit("manipulation_service_reply", {
            requestId,
            name,
            world,
            serviceName,
            requestType: msgType,
            ok: false,
            detail: error instanceof Error ? error.message : String(error),
          });
        });
    } catch (error) {
      scene.emitter.emit("manipulation_dispatch", {
        ok: false,
        requestId,
        name,
        world,
        serviceName,
        msgType,
        position,
        orientation,
        error: error instanceof Error ? error.message : String(error),
      });
    }
  };

  /**
   * Constructor. If a url is specified, then then SceneManager will connect
   * to the specified websocket server. Otherwise, the `connect` function
   * should be called after construction.
   * @param params Optional. The scene manager configuration options
   *
   */
  constructor(config: SceneManagerConfig = {}) {
    this.elementId = config.elementId ?? "gz-scene";

    if (config.audioTopic) {
      this.audioTopic = config.audioTopic;
    }

    if (config.topicName && config.msgType && config.msgData) {
      this.topicName = config.topicName;
      this.msgType = config.msgType;
      this.msgData = config.msgData;
    }

    if (config.websocketUrl) {
      this.connect(config.websocketUrl, config.websocketKey);
    }

    if (config.enableLights !== undefined) {
      this.enableLights = config.enableLights;
    }
  }

  /**
   * Destrory the scene
   */
  public destroy(): void {
    this.disconnect();

    if (this.cancelAnimation) {
      cancelAnimationFrame(this.cancelAnimation);
    }

    this.previousRenderTimestampMs = 0;

    if (this.scene) {
      this.scene.cleanup();
    }
  }

  /**
   * Get the current connection status to a Gazebo server.
   */
  public getConnectionStatus(): string {
    return this.connectionStatus;
  }

  /**
   * Get the connection status as an observable.
   * Allows clients to subscribe to this stream, to let them know when the connection to Gazebo
   * is ready for communication.
   *
   * @returns An Observable of a boolean: Whether the connection status is ready or not.
   */
  public getConnectionStatusAsObservable(): Observable<boolean> {
    return this.transport
      .getConnectionStatus()
      .pipe(map((status) => status === "ready"));
  }

  /**
   * Change the width and height of the visualization upon a resize event.
   */
  public resize(): void {
    if (this.scene) {
      this.scene.setSize(
        this.sceneElement.clientWidth,
        this.sceneElement.clientHeight,
      );
    }
  }

  public snapshot(): void {
    if (this.scene) {
      this.scene.saveScreenshot(this.transport.getWorld());
    }
  }

  public resetView(): void {
    if (this.scene) {
      this.scene.resetView();
    }
  }

  public follow(entityName: string): void {
    if (this.scene) {
      this.scene.emitter.emit("follow_entity", entityName);
    }
  }

  public thirdPersonFollow(entityName: string): void {
    if (this.scene) {
      this.scene.emitter.emit("third_person_follow_entity", entityName);
    }
  }

  public firstPerson(entityName: string): void {
    if (this.scene) {
      this.scene.emitter.emit("first_person_entity", entityName);
    }
  }

  public moveTo(entityName: string): void {
    if (this.scene) {
      this.scene.emitter.emit("move_to_entity", entityName);
    }
  }

  public select(entityName: string): void {
    if (this.scene) {
      this.scene.emitter.emit("select_entity", entityName);
    }
  }

  public clearSelection(): void {
    if (this.scene) {
      this.scene.selectEntity(null);
    }
  }

  public getSelectedEntityName(): string | null {
    if (!this.scene || typeof this.scene.getSelectedEntityName !== "function") {
      return null;
    }
    return this.scene.getSelectedEntityName();
  }

  public setManipulationMode(mode: string): void {
    if (this.scene && typeof this.scene.setManipulationMode === "function") {
      this.scene.setManipulationMode(mode);
    }
  }

  public setManipulationTargetNames(entityNames: string[]): void {
    if (
      this.scene &&
      typeof this.scene.setManipulationTargetNames === "function"
    ) {
      this.scene.setManipulationTargetNames(entityNames);
    }
  }

  public getManipulationMode(): string {
    if (!this.scene || typeof this.scene.getManipulationMode !== "function") {
      return "view";
    }
    return this.scene.getManipulationMode();
  }

  public setControlsEnabled(enabled: boolean): void {
    if (this.scene && typeof this.scene.setControlsEnabled === "function") {
      this.scene.setControlsEnabled(enabled);
    }
  }

  public getControlsEnabled(): boolean {
    if (!this.scene || typeof this.scene.getControlsEnabled !== "function") {
      return true;
    }
    return this.scene.getControlsEnabled();
  }

  public getDomElement(): HTMLCanvasElement | null {
    if (!this.scene || typeof this.scene.getDomElement !== "function") {
      return null;
    }
    return this.scene.getDomElement();
  }

  public intersectPointerOnHorizontalPlane(
    clientX: number,
    clientY: number,
    planeZ: number,
  ): THREE.Vector3 | null {
    if (
      !this.scene ||
      typeof this.scene.intersectPointerOnHorizontalPlane !== "function"
    ) {
      return null;
    }
    return this.scene.intersectPointerOnHorizontalPlane(clientX, clientY, planeZ);
  }

  public intersectPointerOnSceneSurface(
    clientX: number,
    clientY: number,
    options?: {
      ignoreNames?: string[];
      ignoreNameSubstrings?: string[];
    },
  ): { point: THREE.Vector3; surfaceName: string } | null {
    if (
      !this.scene ||
      typeof this.scene.intersectPointerOnSceneSurface !== "function"
    ) {
      return null;
    }
    return this.scene.intersectPointerOnSceneSurface(clientX, clientY, options);
  }

  public previewPose(
    world: string,
    poseNames: string[],
    pose: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    },
  ): boolean {
    if (!this.scene || typeof this.scene.previewPose !== "function") {
      return false;
    }
    return this.scene.previewPose(world, poseNames, pose);
  }

  public onSceneEvent(eventName: string, listener: (...args: any[]) => void): void {
    if (this.scene && typeof this.scene.on === "function") {
      this.scene.on(eventName, listener);
    }
  }

  public offSceneEvent(eventName: string, listener: (...args: any[]) => void): void {
    if (this.scene && typeof this.scene.off === "function") {
      this.scene.off(eventName, listener);
    }
  }

  public isGizmoActive(): boolean {
    if (!this.scene || typeof this.scene.isGizmoActive !== "function") {
      return false;
    }
    return this.scene.isGizmoActive();
  }

  public isGizmoDragTarget(entityName: string): boolean {
    if (!this.scene || typeof this.scene.isGizmoDragTarget !== "function") {
      return false;
    }
    return this.scene.isGizmoDragTarget(entityName);
  }

  /**
   * Publishes a message to an advertised topic.
   */
  public publish(): void {
    if (this.scene && this.publisher) {
      let msg = this.publisher.createMessage(this.msgData);
      this.publisher.publish(msg);
    }
  }

  /**
   * Get the list of models in the scene
   * @return The list of available models.
   */
  public getModels(): any[] {
    return this.models;
  }

  /**
   * Get a 3D model object by its name
   * @param name The name of the model to retrieve
   * @return The 3D model object, or undefined if not found
   */
  public getModelByName(name: string): any {
    return this.modelMap.get(name);
  }

  /**
   * Get multiple 3D model objects by their names
   * @param names Array of model names to retrieve
   * @return Array of 3D model objects, with undefined for any names not found
   */
  public getModelsByNames(names: string[]): any[] {
    return names.map(name => this.modelMap.get(name));
  }

  /**
   * Focus camera on multiple models by their names
   * @param modelNames Array of model names to focus on
   * @param durationMs Duration of the camera movement in milliseconds (default: 800)
   * @param paddingFactor Additional padding factor for the camera view (default: 1.2)
   */
  public focusOnModels(modelNames: string[], durationMs: number = 800, paddingFactor: number = 1.2): void {
    if (!this.scene || !modelNames || modelNames.length === 0) {
      return;
    }

    // Get all model objects
    const models = this.getModelsByNames(modelNames);
    
    // Filter out undefined models and get their 3D objects
    const validModels = models.filter(model => model !== undefined);
    
    if (validModels.length === 0) {
      console.warn('No valid models found for focus:', modelNames);
      return;
    }

    // Use the scene's focus method which handles the camera lerp
    this.scene.cameraLerp.focus(validModels, durationMs, paddingFactor);
  }

  /**
   * Remove a model by name from both the models array and the modelMap
   * @param name The name of the model to remove
   * @return true if the model was found and removed, false otherwise
   */
  public removeModelByName(name: string): boolean {
    // Find the model in the models array
    const foundIndex = this.getModelIndex(name);
    
    if (foundIndex >= 0) {
      // Remove from the models array
      this.models.splice(foundIndex, 1);
      
      // Remove from the modelMap
      this.modelMap.delete(name);
      
      // Remove from the scene
      const modelObj = this.scene.getByName(name);
      if (modelObj) {
        this.scene.remove(modelObj);
      }
      
      return true;
    }
    
    return false;
  }

  /**
   * Clear all models from the scene, models array, and modelMap
   */
  public clearAllModels(): void {
    // Remove all models from the scene
    this.scene.removeAll();
    
    // Clear the models array
    this.models.length = 0;
    
    // Clear the modelMap
    this.modelMap.clear();
  }

  private getOrderedPoseMessageTypes(): string[] {
    const world = this.transport.getWorld();
    const topicName = `/world/${world}/dynamic_pose/info`;
    const availableTopics = this.transport.getAvailableTopics();
    const defaultOrder = [...SceneManager.POSE_MESSAGE_TYPE_CANDIDATES];

    if (!Array.isArray(availableTopics)) {
      return defaultOrder;
    }

    const topicMeta = availableTopics.find((topic) => topic["topic"] === topicName);
    const msgType = typeof topicMeta?.["msg_type"] === "string" ? topicMeta["msg_type"] : "";
    const preferredPrefix = msgType.startsWith("ignition.msgs.")
      ? "ignition.msgs."
      : msgType.startsWith("gazebo.msgs.")
        ? "gazebo.msgs."
        : msgType.startsWith("gz.msgs.")
          ? "gz.msgs."
          : "";

    if (!preferredPrefix) {
      return defaultOrder;
    }

    return defaultOrder.sort(
      (a, b) => Number(b.startsWith(preferredPrefix)) - Number(a.startsWith(preferredPrefix)),
    );
  }

  private normalizeManipulationServiceReply(
    responseType: string,
    response: any,
  ): ManipulationServiceReply {
    const boolField = response?.data;
    if (typeof boolField === "boolean") {
      return {
        ok: boolField,
        detail: responseType,
      };
    }

    if (typeof response?.success === "boolean") {
      return {
        ok: response.success,
        detail: responseType,
      };
    }

    if (typeof response?.result === "boolean") {
      return {
        ok: response.result,
        detail: responseType,
      };
    }

    if (typeof response?.data === "string") {
      return {
        ok: null,
        detail: `${responseType}: ${response.data}`,
      };
    }

    return {
      ok: null,
      detail: responseType,
    };
  }

  private getUnscopedEntityName(entityName: string): string {
    const trimmed = entityName.trim();
    if (!trimmed.includes("::")) {
      return trimmed;
    }
    const parts = trimmed.split("::");
    return parts[parts.length - 1] ?? trimmed;
  }

  private buildPoseAliases(world: string, entityName: string): Set<string> {
    const aliases = new Set<string>();
    const trimmed = entityName.trim();
    if (!trimmed) {
      return aliases;
    }
    const unscoped = this.getUnscopedEntityName(trimmed);
    aliases.add(trimmed);
    aliases.add(unscoped);
    aliases.add(`${world}::${unscoped}`);
    return aliases;
  }

  private trackPendingManipulationPose(pending: PendingManipulationPose): void {
    const now = Date.now();
    this.pendingManipulationPoses = this.pendingManipulationPoses.filter(
      (entry) => entry.expiresAtMs > now && !entry.aliases.has(pending.requestedName),
    );
    this.pendingManipulationPoses.push(pending);
  }

  private consumeMatchingPendingManipulationPose(
    entityName: string,
    pose: { position?: { x?: number; y?: number; z?: number } },
  ): PendingManipulationPose | null {
    const now = Date.now();
    let matched: PendingManipulationPose | null = null;
    this.pendingManipulationPoses = this.pendingManipulationPoses.filter((entry) => {
      if (entry.expiresAtMs <= now) {
        return false;
      }
      if (!entry.aliases.has(entityName)) {
        return true;
      }

      const dx = (pose.position?.x ?? 0) - entry.targetPosition.x;
      const dy = (pose.position?.y ?? 0) - entry.targetPosition.y;
      const dz = (pose.position?.z ?? 0) - entry.targetPosition.z;
      const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (distance <= SceneManager.MANIPULATION_POSE_LOCK_TOLERANCE_METERS) {
        matched = entry;
        return false;
      }

      matched = entry;
      return true;
    });
    return matched;
  }

  private shouldSuppressPoseUpdate(
    entityName: string,
    pose: { position?: { x?: number; y?: number; z?: number } },
  ): boolean {
    const pending = this.consumeMatchingPendingManipulationPose(entityName, pose);
    if (!pending) {
      return false;
    }

    const dx = (pose.position?.x ?? 0) - pending.targetPosition.x;
    const dy = (pose.position?.y ?? 0) - pending.targetPosition.y;
    const dz = (pose.position?.z ?? 0) - pending.targetPosition.z;
    const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
    return distance > SceneManager.MANIPULATION_POSE_LOCK_TOLERANCE_METERS;
  }

  /**
   * Disconnect from the Gazebo server
   */
  public disconnect(): void {
    if (this.scene && typeof this.scene.off === "function") {
      this.scene.off("manipulation_commit", this.handleManipulationCommit);
    }

    // Remove the canvas. Helpful to disconnect and connect several times.
    if (
      this.sceneElement?.childElementCount > 0 &&
      this.scene.scene.renderer?.domElement
    ) {
      this.sceneElement.removeChild(this.scene.scene.renderer.domElement);
    }

    this.transport.disconnect();
    this.sceneInfo = {};
    this.connectionStatus = "disconnected";
    this.pendingManipulationPoses = [];

    // Clear all models and mappings
    this.clearAllModels();

    // Unsubscribe from observables.
    if (this.sceneInfoSubscription) {
      this.sceneInfoSubscription.unsubscribe();
    }
    if (this.particleEmittersSubscription) {
      this.particleEmittersSubscription.unsubscribe();
    }

    if (this.statusSubscription) {
      this.statusSubscription.unsubscribe();
    }
  }

  /**
   * Connect to a Gazebo server
   * @param url A websocket url that points to a Gazebo server.
   * @param key An optional authentication key.
   */
  public connect(url: string, key?: string): void {
    this.transport.connect(url, key);

    this.statusSubscription = this.transport
      .getConnectionStatus()
      .subscribe((response) => {
        if (response === "error") {
          // TODO: Return an error so the caller can open a snackbar
          console.log("Connection failed. Please contact an administrator.");
          // this.snackBar.open('Connection failed. Please contact an administrator.', 'Got it');
        }

        this.connectionStatus = response;

        // We can start setting up the visualization after we are Connected.
        // We still don't have scene and world information at this step.
        if (response === "connected") {
          this.setupVisualization();
        }

        // Once the status is ready, we have the world and scene information
        // available.
        if (response === "ready") {
          this.subscribeToTopics();
          if (this.topicName) {
            this.publisher = this.advertise(this.topicName, this.msgType);
            console.log(`Advertised ${this.topicName} with msg type of
                      ${this.msgType}`);
          }
        }
      });

    // Scene information.
    this.sceneInfoSubscription = this.transport.sceneInfo$.subscribe(
      (sceneInfo) => {
        if (!sceneInfo) {
          return;
        }

        if ("sky" in sceneInfo && sceneInfo["sky"]) {
          const sky = sceneInfo["sky"];

          // Check to see if a cubemap has been specified in the header.
          if ("header" in sky && sky["header"] && sky["header"]["data"]) {
            const data = sky["header"]["data"];
            for (let i = 0; i < data.length; ++i) {
              if (
                data[i]["key"] === "cubemap_uri" &&
                data[i]["value"] !== undefined
              ) {
                this.scene.addSky(data[i]["value"][0]);
              }
            }
          } else {
            this.scene.addSky();
          }
        }
        this.sceneInfo = sceneInfo;
        this.startVisualization();

        sceneInfo["model"].forEach((model: any) => {
          const modelObj = this.sdfParser.spawnFromObj(
            { model },
            { enableLights: this.enableLights },
          );
          console.log("Gazebo visualization : adding ", modelObj.name);
          model["gz3dName"] = modelObj.name;
          this.models.push(model);
          this.modelMap.set(model.name, modelObj);
          this.scene.add(modelObj);
        });

        sceneInfo["light"].forEach((light: any) => {
          const lightObj = this.sdfParser.spawnLight(light);
          this.scene.add(lightObj);
        });

        // Set the ambient color, if present
        if (
          sceneInfo["ambient"] !== undefined &&
          sceneInfo["ambient"] !== null
        ) {
          this.scene.ambient.color = new THREE.Color(
            sceneInfo["ambient"]["r"],
            sceneInfo["ambient"]["g"],
            sceneInfo["ambient"]["b"],
          );
        }
      },
    );
  }

  /**
   * Advertise a topic.
   *
   * @param topic The topic to advertise.
   */
  public advertise(topic: string, msgTypeName: string): Publisher {
    return this.transport.advertise(topic, msgTypeName);
  }

  /**
   * Allows clients to subscribe to a custom topic.
   *
   * @param topic The topic to subscribe to.
   */
  public subscribeToTopic(topic: Topic): void {
    this.transport.subscribe(topic);
  }

  /**
   * Allows clients to unsubscribe from topics.
   *
   * @param name The name of the topic to unsubscribe from.
   */
  public unsubscribeFromTopic(name: string): void {
    this.transport.unsubscribe(name);
  }

  /**
   * Play the Simulation.
   */
  public play(): void {
    this.transport.requestService(
      `/world/${this.transport.getWorld()}/control`,
      "ignition.msgs.WorldControl",
      { pause: false },
    );
  }

  /**
   * Pause the Simulation.
   */
  public pause(): void {
    this.transport.requestService(
      `/world/${this.transport.getWorld()}/control`,
      "ignition.msgs.WorldControl",
      { pause: true },
    );
  }

  /**
   * Stop the Simulation.
   */
  public stop(): void {
    this.transport.requestService(
      "/server_control",
      "ignition.msgs.ServerControl",
      { stop: true },
    );
  }

  /**
   * Subscribe to Gazebo topics required to render a scene.
   *
   * This includes:
   * - /world/WORLD_NAME/dynamic_pose/info
   * - /world/WORLD_NAME/scene/info
   */
  private subscribeToTopics(): void {
    // Subscribe to the pose topic and modify the models' poses.
    const poseTopic = new Topic(
      `/world/${this.transport.getWorld()}/dynamic_pose/info`,
      (msg) => {
        msg["pose"].forEach((pose: any) => {
          let entityName = pose["name"];
          if (
            typeof this.scene.isDragTarget === "function" &&
            this.scene.isDragTarget(entityName)
          ) {
            return;
          }
          if (this.shouldSuppressPoseUpdate(entityName, pose)) {
            return;
          }
          // Objects created by Gz3D have an unique name, which is the
          // name plus the id.
          const entity = this.scene.getByName(entityName);

          if (entity) {
            this.scene.setPose(entity, pose.position, pose.orientation);
          } else {
            console.warn(
              "Unable to find entity with name ",
              entityName,
              entity,
            );
          }
        });
      },
    );
    this.transport.subscribe(poseTopic);

    // Subscribe to the audio control topic.
    if (this.audioTopic) {
      const audioTopic = new AudioTopic(this.audioTopic, this.transport);
    }

    // Subscribe to the 'scene/info' topic which sends scene changes.
    const sceneTopic = new Topic(
      `/world/${this.transport.getWorld()}/scene/info`,
      (sceneInfo) => {
        if (!sceneInfo) {
          return;
        }

        // Process each model in the scene.
        sceneInfo["model"].forEach((model: any) => {
          // Check to see if the model already exists in the scene. This
          // could happen when a simulation level is loaded multiple times.
          let foundIndex = this.getModelIndex(model["name"]);

          // If the model was not found, then add the new model. Otherwise
          // update the models ID.
          if (foundIndex < 0) {
            const modelObj = this.sdfParser.spawnFromObj(
              { model },
              { enableLights: this.enableLights },
            );
            this.models.push(model);
            this.modelMap.set(model.name, modelObj);
            this.scene.add(modelObj);
          } else {
            // Make sure to update the exisiting models so that future pose
            // messages can update the model.
            this.models[foundIndex]["id"] = model["id"];
          }
        });
      },
    );
    this.transport.subscribe(sceneTopic);
  }

  /**
   * Get the index into the model array of a model based on a name
   */
  private getModelIndex(name: string): number {
    let foundIndex = -1;
    for (let i = 0; i < this.models.length; ++i) {
      // Simulation enforces unique names between models. The ID
      // of a model may change. This occurs when levels are loaded,
      // unloaded, and then reloaded.
      if (this.models[i]["name"] === name) {
        foundIndex = i;
        break;
      }
    }
    return foundIndex;
  }

  /**
   * Setup the visualization scene.
   */
  private setupVisualization(): void {
    var that = this;

    // Create a find asset helper
    function findAsset(_uri: string, _cb: any) {
      that.transport.getAsset(_uri, _cb);
    }

    this.scene = new Scene({
      shaders: new Shaders(),
      findResourceCb: findAsset,
    });
    this.scene.on("manipulation_commit", this.handleManipulationCommit);
    this.sdfParser = new SDFParser(this.scene);
    this.sdfParser.usingFilesUrls = true;

    if (window.document.getElementById(this.elementId)) {
      this.sceneElement = window.document.getElementById(this.elementId)!;
    } else {
      console.error(
        "Unable to find HTML element with an id of",
        this.elementId,
      );
    }
    this.sceneElement.appendChild(this.scene.renderer.domElement);

    this.scene.setSize(
      this.sceneElement.clientWidth,
      this.sceneElement.clientHeight,
    );
  }

  /**
   * Animation loop.
   *
   * Renders the scene and updates any system and time-related variables.
   */
  private animate(): void {
    this.cancelAnimation = requestAnimationFrame((timestampMs) => {
      if (this.previousRenderTimestampMs === 0) {
        this.previousRenderTimestampMs = timestampMs;
      }

      this.animate();

      if (this.scene.getParticleSystem()) {
        this.scene.getParticleSystem().update();
      }

      this.scene.render(timestampMs - this.previousRenderTimestampMs);
      this.previousRenderTimestampMs = timestampMs;
    });
  }

  /**
   * Start the visualization rendering loop.
   */
  private startVisualization(): void {
    this.animate();
  }
}
