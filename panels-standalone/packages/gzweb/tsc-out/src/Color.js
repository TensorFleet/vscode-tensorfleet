import * as THREE from "three";
// Color class with alpha. THREE.js has a Color class that lacks an alpha
// channel.
export class Color extends THREE.Color {
    constructor(r, g, b, a) {
        super(r, g, b);
        this.a = 1.0;
        if (a) {
            this.a = a;
        }
    }
}
