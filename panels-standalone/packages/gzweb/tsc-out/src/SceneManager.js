import * as THREE from "three";
import { AudioTopic } from "./AudioTopic";
import { Scene } from "./Scene";
import { SDFParser } from "./SDFParser";
import { Shaders } from "./Shaders";
import { map } from "rxjs";
import { Topic } from "./Topic";
import { Transport } from "./Transport";
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
    /**
     * Constructor. If a url is specified, then then SceneManager will connect
     * to the specified websocket server. Otherwise, the `connect` function
     * should be called after construction.
     * @param params Optional. The scene manager configuration options
     *
     */
    constructor(config = {}) {
        var _a;
        /**
         * Connection status from the Websocket.
         */
        this.connectionStatus = "disconnected";
        /**
         * List of 3d models.
         */
        this.models = [];
        /**
         * A Transport interface used to connect to a Gazebo server.
         */
        this.transport = new Transport();
        /**
         *
         */
        this.previousRenderTimestampMs = 0;
        /**
         * Name of the HTML element that will hold the rendering scene.
         */
        this.elementId = "gz-scene";
        /*
         * Whether or not lights in models are visible. Enabled by default.
         */
        this.enableLights = true;
        this.elementId = (_a = config.elementId) !== null && _a !== void 0 ? _a : "gz-scene";
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
    destroy() {
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
    getConnectionStatus() {
        return this.connectionStatus;
    }
    /**
     * Get the connection status as an observable.
     * Allows clients to subscribe to this stream, to let them know when the connection to Gazebo
     * is ready for communication.
     *
     * @returns An Observable of a boolean: Whether the connection status is ready or not.
     */
    getConnectionStatusAsObservable() {
        return this.transport
            .getConnectionStatus()
            .pipe(map((status) => status === "ready"));
    }
    /**
     * Change the width and height of the visualization upon a resize event.
     */
    resize() {
        if (this.scene) {
            this.scene.setSize(this.sceneElement.clientWidth, this.sceneElement.clientHeight);
        }
    }
    snapshot() {
        if (this.scene) {
            this.scene.saveScreenshot(this.transport.getWorld());
        }
    }
    resetView() {
        if (this.scene) {
            this.scene.resetView();
        }
    }
    follow(entityName) {
        if (this.scene) {
            this.scene.emitter.emit("follow_entity", entityName);
        }
    }
    thirdPersonFollow(entityName) {
        if (this.scene) {
            this.scene.emitter.emit("third_person_follow_entity", entityName);
        }
    }
    firstPerson(entityName) {
        if (this.scene) {
            this.scene.emitter.emit("first_person_entity", entityName);
        }
    }
    moveTo(entityName) {
        if (this.scene) {
            this.scene.emitter.emit("move_to_entity", entityName);
        }
    }
    select(entityName) {
        if (this.scene) {
            this.scene.emitter.emit("select_entity", entityName);
        }
    }
    /**
     * Publishes a message to an advertised topic.
     */
    publish() {
        if (this.scene && this.publisher) {
            let msg = this.publisher.createMessage(this.msgData);
            this.publisher.publish(msg);
        }
    }
    /**
     * Get the list of models in the scene
     * @return The list of available models.
     */
    getModels() {
        return this.models;
    }
    /**
     * Disconnect from the Gazebo server
     */
    disconnect() {
        var _a, _b;
        // Remove the canvas. Helpful to disconnect and connect several times.
        if (((_a = this.sceneElement) === null || _a === void 0 ? void 0 : _a.childElementCount) > 0 &&
            ((_b = this.scene.scene.renderer) === null || _b === void 0 ? void 0 : _b.domElement)) {
            this.sceneElement.removeChild(this.scene.scene.renderer.domElement);
        }
        this.transport.disconnect();
        this.sceneInfo = {};
        this.connectionStatus = "disconnected";
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
    connect(url, key) {
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
        this.sceneInfoSubscription = this.transport.sceneInfo$.subscribe((sceneInfo) => {
            if (!sceneInfo) {
                return;
            }
            if ("sky" in sceneInfo && sceneInfo["sky"]) {
                const sky = sceneInfo["sky"];
                // Check to see if a cubemap has been specified in the header.
                if ("header" in sky && sky["header"] && sky["header"]["data"]) {
                    const data = sky["header"]["data"];
                    for (let i = 0; i < data.length; ++i) {
                        if (data[i]["key"] === "cubemap_uri" &&
                            data[i]["value"] !== undefined) {
                            this.scene.addSky(data[i]["value"][0]);
                        }
                    }
                }
                else {
                    this.scene.addSky();
                }
            }
            this.sceneInfo = sceneInfo;
            this.startVisualization();
            sceneInfo["model"].forEach((model) => {
                const modelObj = this.sdfParser.spawnFromObj({ model }, { enableLights: this.enableLights });
                model["gz3dName"] = modelObj.name;
                this.models.push(model);
                this.scene.add(modelObj);
            });
            sceneInfo["light"].forEach((light) => {
                const lightObj = this.sdfParser.spawnLight(light);
                this.scene.add(lightObj);
            });
            // Set the ambient color, if present
            if (sceneInfo["ambient"] !== undefined &&
                sceneInfo["ambient"] !== null) {
                this.scene.ambient.color = new THREE.Color(sceneInfo["ambient"]["r"], sceneInfo["ambient"]["g"], sceneInfo["ambient"]["b"]);
            }
        });
    }
    /**
     * Advertise a topic.
     *
     * @param topic The topic to advertise.
     */
    advertise(topic, msgTypeName) {
        return this.transport.advertise(topic, msgTypeName);
    }
    /**
     * Allows clients to subscribe to a custom topic.
     *
     * @param topic The topic to subscribe to.
     */
    subscribeToTopic(topic) {
        this.transport.subscribe(topic);
    }
    /**
     * Allows clients to unsubscribe from topics.
     *
     * @param name The name of the topic to unsubscribe from.
     */
    unsubscribeFromTopic(name) {
        this.transport.unsubscribe(name);
    }
    /**
     * Play the Simulation.
     */
    play() {
        this.transport.requestService(`/world/${this.transport.getWorld()}/control`, "ignition.msgs.WorldControl", { pause: false });
    }
    /**
     * Pause the Simulation.
     */
    pause() {
        this.transport.requestService(`/world/${this.transport.getWorld()}/control`, "ignition.msgs.WorldControl", { pause: true });
    }
    /**
     * Stop the Simulation.
     */
    stop() {
        this.transport.requestService("/server_control", "ignition.msgs.ServerControl", { stop: true });
    }
    /**
     * Subscribe to Gazebo topics required to render a scene.
     *
     * This includes:
     * - /world/WORLD_NAME/dynamic_pose/info
     * - /world/WORLD_NAME/scene/info
     */
    subscribeToTopics() {
        // Subscribe to the pose topic and modify the models' poses.
        const poseTopic = new Topic(`/world/${this.transport.getWorld()}/dynamic_pose/info`, (msg) => {
            msg["pose"].forEach((pose) => {
                let entityName = pose["name"];
                // Objects created by Gz3D have an unique name, which is the
                // name plus the id.
                const entity = this.scene.getByName(entityName);
                if (entity) {
                    this.scene.setPose(entity, pose.position, pose.orientation);
                }
                else {
                    console.warn("Unable to find entity with name ", entityName, entity);
                }
            });
        });
        this.transport.subscribe(poseTopic);
        // Subscribe to the audio control topic.
        if (this.audioTopic) {
            const audioTopic = new AudioTopic(this.audioTopic, this.transport);
        }
        // Subscribe to the 'scene/info' topic which sends scene changes.
        const sceneTopic = new Topic(`/world/${this.transport.getWorld()}/scene/info`, (sceneInfo) => {
            if (!sceneInfo) {
                return;
            }
            // Process each model in the scene.
            sceneInfo["model"].forEach((model) => {
                // Check to see if the model already exists in the scene. This
                // could happen when a simulation level is loaded multiple times.
                let foundIndex = this.getModelIndex(model["name"]);
                // If the model was not found, then add the new model. Otherwise
                // update the models ID.
                if (foundIndex < 0) {
                    const modelObj = this.sdfParser.spawnFromObj({ model }, { enableLights: this.enableLights });
                    this.models.push(model);
                    this.scene.add(modelObj);
                }
                else {
                    // Make sure to update the exisiting models so that future pose
                    // messages can update the model.
                    this.models[foundIndex]["id"] = model["id"];
                }
            });
        });
        this.transport.subscribe(sceneTopic);
    }
    /**
     * Get the index into the model array of a model based on a name
     */
    getModelIndex(name) {
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
    setupVisualization() {
        var that = this;
        // Create a find asset helper
        function findAsset(_uri, _cb) {
            that.transport.getAsset(_uri, _cb);
        }
        this.scene = new Scene({
            shaders: new Shaders(),
            findResourceCb: findAsset,
        });
        this.sdfParser = new SDFParser(this.scene);
        this.sdfParser.usingFilesUrls = true;
        if (window.document.getElementById(this.elementId)) {
            this.sceneElement = window.document.getElementById(this.elementId);
        }
        else {
            console.error("Unable to find HTML element with an id of", this.elementId);
        }
        this.sceneElement.appendChild(this.scene.renderer.domElement);
        this.scene.setSize(this.sceneElement.clientWidth, this.sceneElement.clientHeight);
    }
    /**
     * Animation loop.
     *
     * Renders the scene and updates any system and time-related variables.
     */
    animate() {
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
    startVisualization() {
        this.animate();
    }
}
