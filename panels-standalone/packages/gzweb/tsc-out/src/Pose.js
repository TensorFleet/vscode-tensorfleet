import { Vector3, Quaternion } from "three";
export class Pose {
    constructor(pos, rot) {
        this.position = new Vector3();
        this.orientation = new Quaternion();
        if (pos) {
            this.position = pos;
        }
        if (rot) {
            this.orientation = rot;
        }
    }
}
