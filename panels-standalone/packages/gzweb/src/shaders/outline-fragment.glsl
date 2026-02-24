uniform sampler2D tDepth;
uniform vec2 px; // Pre-calculated pixel size (1.0 / resolution)
uniform vec3 outlineColor; // Interpolated outline color
uniform vec3 fillColor;
uniform float fillOpacity;
varying vec2 vUv;

float depthAt(vec2 uv){
    return texture2D(tDepth, uv).r;
}

void main(){
    float d = depthAt(vUv);

    if (d <= 0.9999) {
        discard;
    }
    // gl_FragColor = vec4(1.0,0.0,0.0, 1.0);
    // return;

    float edgeFactor = 0.17;
    float edgeAmount = 0.0;
    for(int i = -3; i < 3; ++i)
        for(int j = -3; j < 3; ++j) {
            if (i == 0 && j == 0) continue;
            float diff = abs(d - depthAt(vUv + vec2(px.x*float(i), px.y*float(j))));
            if (diff > 0.0001) {
                edgeAmount += edgeFactor;
            }
        }
    
    if (edgeAmount > 0.0) {
        gl_FragColor = vec4(outlineColor, min(1.0, edgeAmount));
        return;
    }
//   gl_FragColor = vec4(0.0,0.0,0.0, 1.0);
    discard;
}