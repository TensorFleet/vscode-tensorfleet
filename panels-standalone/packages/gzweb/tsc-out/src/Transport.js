import { BehaviorSubject } from "rxjs";
import { parse } from "protobufjs";
import { Publisher } from "./Publisher";
import { AssetError } from "./Asset";
/**
 * The Transport class is in charge of managing the websocket connection to a
 * Gazebo websocket server.
 */
export class Transport {
    constructor() {
        /**
         * Scene Information behavior subject.
         * Components can subscribe to it to get the scene information once it is obtained.
         */
        this.sceneInfo$ = new BehaviorSubject(null);
        /**
         * List of available topics.
         *
         * Array of objects containing {topic, msg_type}.
         */
        this.availableTopics = [];
        /**
         * Map of the subscribed topics.
         * - Key: The topic name.
         * - Value: The Topic object, which includes the callback.
         *
         * New subscriptions should be added to this map, in order to correctly derive the messages
         * received.
         */
        this.topicMap = new Map();
        /**
         * A map of asset uri to asset types. This allows a caller to request
         * an asset from the websocket server and receive a callback when the
         * aseset has been fetched.
         */
        this.assetMap = new Map();
        /**
         * The world that is being used in the Simulation.
         */
        this.world = "";
        /**
         * Status connection behavior subject.
         * Internally keeps track of the connection state.
         * Uses a Behavior Subject because it has an initial state and stores a value.
         */
        this.status$ = new BehaviorSubject("disconnected");
    }
    /**
     * Connects to a websocket.
     *
     * @param url The url to connect to.
     * @param key Optional. A key to authorize access to the websocket messages.
     */
    connect(url, key) {
        // First, disconnect from previous connections.
        // This way we make sure that we only support one websocket connection.
        this.disconnect();
        // Create the Websocket interface.
        this.ws = new WebSocket(url);
        // Set the handlers of the websocket events.
        this.ws.onopen = () => this.onOpen(key);
        this.ws.onclose = () => this.onClose();
        this.ws.onmessage = (msgEvent) => this.onMessage(msgEvent);
        this.ws.onerror = (errorEvent) => this.onError(errorEvent);
    }
    /**
     * Disconnects from a websocket.
     * Note: The cleanup should be done in the onclose event of the Websocket.
     */
    disconnect() {
        if (this.ws) {
            this.ws.close();
        }
    }
    /**
     * Advertise a topic.
     *
     * @param topic The topic to advertise.
     * @param msgTypeName The message type the topic will handle.
     * @returns The Publisher instance.
     */
    advertise(topic, msgTypeName) {
        this.sendMessage(["adv", topic, msgTypeName, ""]);
        const msgDef = this.root.lookupType(msgTypeName);
        return new Publisher(topic, msgTypeName, msgDef, (topic, msgTypeName, msg) => {
            this.publish(topic, msgTypeName, msg);
        });
    }
    /**
     * Publish to a topic.
     *
     * @param topic The topic to publish to.
     * @param msgTypeName The message type.
     * @param msg The message to publish.
     */
    publish(topic, msgTypeName, msg) {
        this.sendMessage(["pub_in", topic, msgTypeName, msg]);
    }
    /**
     * Request a service.
     *
     * @param topic The service to request to.
     * @param msgTypeName The message type.
     * @param msg The message to publish. This should be a JSON representation
     * of the protobuf message.
     */
    requestService(topic, msgTypeName, msgProperties) {
        if (!this.root) {
            console.error("Unable to request service - Message definitions are not ready");
            return;
        }
        const msgDef = this.root.lookupType(msgTypeName);
        if (!msgDef || msgDef === undefined) {
            console.error(`Unable to lookup message type: ${msgTypeName}`);
            return;
        }
        const msg = msgDef.create(msgProperties);
        if (!msg || msg === undefined) {
            console.error(`Unable to create ${msgTypeName}, from, ${msgProperties}`);
            return;
        }
        // Serialized the message
        const buffer = msgDef.encode(msg).finish();
        if (!buffer || buffer === undefined || buffer.length === 0) {
            console.error("Unable to serialize message.");
            return;
        }
        const strBuf = new TextDecoder().decode(buffer);
        this.sendMessage(["req", topic, msgTypeName, strBuf]);
    }
    /**
     * Subscribe to a topic.
     *
     * @param topic The topic to subscribe to.
     */
    subscribe(topic) {
        this.topicMap.set(topic.name, topic);
        const publisher = this.availableTopics.filter((pub) => pub["topic"] === topic.name)[0];
        if (publisher["msg_type"] === "ignition.msgs.Image" ||
            publisher["msg_type"] === "gazebo.msgs.Image") {
            this.sendMessage(["image", topic.name, "", ""]);
        }
        else {
            this.sendMessage(["sub", topic.name, "", ""]);
        }
    }
    /**
     * Unsubscribe from a topic.
     *
     * @param name The name of the topic to unsubcribe from.
     */
    unsubscribe(name) {
        if (this.topicMap.has(name)) {
            const topic = this.topicMap.get(name);
            if (topic !== undefined && topic.unsubscribe !== undefined) {
                topic.unsubscribe();
            }
            this.topicMap.delete(name);
            this.sendMessage(["unsub", name, "", ""]);
        }
    }
    /**
     * throttle the rate at which messages are published on a topic.
     *
     * @param topic The topic to throttle.
     * @param rate Publish rate.
     */
    throttle(topic, rate) {
        this.sendMessage(["throttle", topic.name, "na", rate.toString()]);
    }
    /**
     * Return the list of available topics.
     *
     * @returns The list of topics that can be subscribed to.
     */
    getAvailableTopics() {
        return this.availableTopics;
    }
    /**
     * Return the list of subscribed topics.
     *
     * @returns A map containing the name and message type of topics that we are currently
     *          subscribed to.
     */
    getSubscribedTopics() {
        return this.topicMap;
    }
    /**
     * Return the world.
     *
     * @returns The name of the world the websocket is connected to.
     */
    getWorld() {
        return this.world;
    }
    /**
     * Get an asset from Gazebo
     */
    getAsset(_uri, _cb) {
        let asset = {
            uri: _uri,
            cb: _cb,
        };
        console.log(`Getting asset via websocket - ${_uri}`);
        this.assetMap.set(_uri, asset);
        this.sendMessage(["asset", "", "", _uri]);
    }
    /**
     * Send a message through the websocket. It verifies if the message is correct and if the
     * connection status allows it to be sent.
     *
     * @param msg The message to send. It consists of four parts:
     *   1. Operation
     *   2. Topic name
     *   3. Message type
     *   4. Payload
     */
    sendMessage(msg) {
        // Verify the message has four parts.
        if (msg.length !== 4) {
            console.error("Message must have four parts", msg);
            return;
        }
        // Only send the message when the connection allows it.
        // Note: Some messages need to be sent during the connection process.
        const connectionStatus = this.status$.getValue();
        if (connectionStatus === "error") {
            console.error("Cannot send the message. Connection failed.", {
                status: connectionStatus,
                message: msg,
            });
            return;
        }
        // In order to properly establish a connection, we need to send certain messages, such as
        // authentication messages, world name, etc.
        const operation = msg[0];
        if (operation === "auth" ||
            operation === "protos" ||
            operation === "topics-types" ||
            operation === "worlds") {
            this.ws.send(this.buildMsg(msg));
            return;
        }
        // Other messages should be sent when the connection status is connected or ready.
        if (connectionStatus === "disconnected") {
            console.error("Trying to send a message but the websocket is disconnected.", msg);
            return;
        }
        this.ws.send(this.buildMsg(msg));
    }
    /**
     * Exposes the connection status as an Observable.
     */
    getConnectionStatus() {
        return this.status$.asObservable();
    }
    /**
     * Handler for the open event of a Websocket.
     *
     * @param key Optional. A key to authorize access to the websocket messages.
     */
    onOpen(key) {
        // An authorization key could be required to request the message definitions.
        if (key) {
            this.sendMessage(["auth", "", "", key]);
        }
        else {
            this.sendMessage(["protos", "", "", ""]);
        }
    }
    /**
     * Handler for the close event of a Websocket.
     *
     * Cleanup the connections.
     */
    onClose() {
        this.topicMap.clear();
        this.availableTopics = [];
        this.root = null;
        this.status$.next("disconnected");
        this.sceneInfo$.next(null);
    }
    /**
     * Handler for the message event of a Websocket.
     *
     * Parses message responses from Gazebo and sends to the corresponding topic.
     */
    onMessage(event) {
        // If there is no Root, then handle authentication and the message definitions.
        const fileReader = new FileReader();
        if (!this.root) {
            fileReader.onloadend = () => {
                const content = fileReader.result;
                // Handle the response.
                switch (content) {
                    case "authorized":
                        // Get the message definitions.
                        this.sendMessage(["protos", "", "", ""]);
                        break;
                    case "invalid":
                        // TODO(germanmas) Throw a proper Unauthorized error.
                        console.error("Invalid key");
                        break;
                    default:
                        // Parse the message definitions.
                        this.root = parse(fileReader.result, {
                            keepCase: true,
                        }).root;
                        // Request topics.
                        this.sendMessage(["topics-types", "", "", ""]);
                        // Request world information.
                        this.sendMessage(["worlds", "", "", ""]);
                        // Now we can update the connection status.
                        this.status$.next("connected");
                        break;
                }
            };
            fileReader.readAsText(event.data);
            return;
        }
        fileReader.onloadend = () => {
            var _a, _b;
            if (!this.root) {
                console.error("Protobuf root has not been created");
                return;
            }
            // Return if at any point, the websocket connection is lost.
            if (this.status$.getValue() === "disconnected") {
                return;
            }
            // Decode as UTF-8 to get the header.
            const str = new TextDecoder("utf-8").decode(fileReader.result);
            const frameParts = str.split(",", 4);
            const msgType = this.root.lookup(frameParts[2]);
            const buffer = new Uint8Array(fileReader.result);
            // Decode the Message. The "+3" in the slice accounts for the commas in the frame.
            let msg;
            // get the actual msg payload without the header
            const msgData = buffer.slice(frameParts[0].length + frameParts[1].length + frameParts[2].length + 3);
            // do not decode image msg as it is raw compressed png data and not a
            // protobuf msg
            if (frameParts[2] === "ignition.msgs.Image" ||
                frameParts[2] === "gazebo.msgs.Image") {
                msg = msgData;
            }
            else {
                msg = msgType.decode(msgData);
            }
            // For frame format information see the WebsocketServer documentation at:
            // https://github.com/gazebosim/gz-launch/blob/ign-launch5/plugins/websocket_server/WebsocketServer.hh
            if (frameParts[0] == "asset") {
                // Error to pass to the callback function, in order for the requester to handle it.
                let error;
                // Check for errors. We can check if the type is a string to avoid comapring with large assets.
                if (frameParts[2] === "ignition.msgs.StringMsg" ||
                    frameParts[2] === "gazebo.msgs.StringMsg") {
                    switch (msg["data"]) {
                        case AssetError.URI_MISSING:
                            console.error("Asset is missing an URI");
                            break;
                        case AssetError.NOT_FOUND:
                            console.error(`Asset not found via websocket - ${frameParts[1]}`);
                            // Set the error for the requester to handle.
                            error = AssetError.NOT_FOUND;
                            break;
                        default:
                            console.error(`Asset error:`, msg["data"]);
                            break;
                    }
                    // There is no error for the requester.
                    if (!error) {
                        return;
                    }
                }
                // Run the callback associated with the asset. This lets the requester
                // process the asset message.
                if (this.assetMap.has(frameParts[1])) {
                    this.assetMap.get(frameParts[1]).cb(msg["data"], error);
                }
                else {
                    console.error(`No resource callback for ${this.assetMap.get(frameParts[1]).uri}`);
                }
            }
            else if (frameParts[0] == "pub") {
                // Handle actions and messages.
                switch (frameParts[1]) {
                    case "topics-types":
                        for (const pub of msg["publisher"]) {
                            this.availableTopics.push(pub);
                        }
                        break;
                    case "topics":
                        this.availableTopics = msg["data"];
                        break;
                    case "worlds":
                        // The world name needs to be used to get the scene information.
                        this.world = msg["data"][0];
                        this.sendMessage(["scene", this.world, "", ""]);
                        break;
                    case "scene":
                        // Emit the scene information. Contains all the models used.
                        this.sceneInfo$.next(msg);
                        // Once we received the Scene Information, we can start working.
                        // We emit the Ready status to reflect this.
                        this.status$.next("ready");
                        break;
                    default:
                        // Message from a subscribed topic. Get the topic and execute its
                        // callback.
                        if (this.topicMap.has(frameParts[1])) {
                            (_b = (_a = this === null || this === void 0 ? void 0 : this.topicMap) === null || _a === void 0 ? void 0 : _a.get(frameParts[1])) === null || _b === void 0 ? void 0 : _b.cb(msg);
                        }
                        break;
                }
            }
            else if (frameParts[0] == "req") {
                // We are not handling response messages from service calls.
            }
            else {
                console.warn("Unhandled websocket message with frame operation", frameParts[0]);
            }
        };
        // Read the blob data as an array buffer.
        fileReader.readAsArrayBuffer(event.data);
        return;
    }
    /**
     * Handler for the error event of a Websocket.
     */
    onError(event) {
        this.status$.next("error");
        this.disconnect();
        console.error(event);
    }
    /**
     * Helper function to build a message.
     * The message is a comma-separated string consisting in four parts:
     * 1. Operation
     * 2. Topic name
     * 3. Message type
     * 4. Payload
     */
    buildMsg(parts) {
        return parts.join(",");
    }
}
