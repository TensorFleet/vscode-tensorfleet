/**
 * Type that represents a topic to be subscribed. This allows communication between Components and
 * the Websocket service of a Simulation.
 */
export class Topic {
    constructor(name, cb) {
        this.name = name;
        this.cb = cb;
    }
}
