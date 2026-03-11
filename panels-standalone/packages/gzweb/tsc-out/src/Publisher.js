/**
 * A Publisher is used to allow clients to publish messages to a particular topic.
 */
export class Publisher {
    /**
     * This constructor should be called by Transport.
     *
     * @param topic The topic name to publish to.
     * @param msgTypeName The message type name to use.
     * @param def The protobuf message definition.
     * @param pub Function set by Transport in order to send the message through the websocket.
     */
    constructor(topic, msgTypeName, def, pub) {
        this.topic = topic;
        this.msgTypeName = msgTypeName;
        this.messageDef = def;
        this.pubFunc = pub;
    }
    /**
     * Creates a new message using the specified properties.
     *
     * @param properties The propoerties to be set in the message.
     * @returns The message instance.
     */
    createMessage(properties) {
        return this.messageDef.create(properties);
    }
    /**
     * Publish a message.
     *
     * @param msg The message to publish.
     */
    publish(msg) {
        // Serialized the message
        let buffer = this.messageDef.encode(msg).finish();
        let strBuf = new TextDecoder().decode(buffer);
        // Publish the message over the websocket
        this.pubFunc(this.topic, this.msgTypeName, strBuf);
    }
}
