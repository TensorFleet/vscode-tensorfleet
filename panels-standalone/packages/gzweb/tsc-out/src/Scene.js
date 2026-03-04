import * as THREE from "three";
import { getDescendants, binaryToImage } from "./Globals";
import { ColladaLoader } from "../include/ColladaLoader";
import { Color } from "./Color";
import { DDSLoader } from "../include/DDSLoader";
import { EventEmitter2 } from "eventemitter2";
import { GzObjLoader } from "./GzObjLoader";
import { ModelUserData } from "./ModelUserData";
import { OrbitControls } from "../include/OrbitControls";
import { createFuelUri } from "./FuelServer";
import { Pose } from "./Pose";
import { SpawnModel } from "./SpawnModel";
import { STLLoader } from "../include/STLLoader";
import { GLTFLoader } from "../include/GLTFLoader";
import { WsLoadingManager } from "./WsLoadingManager";
import * as JSZip from "jszip";
var JointTypes;
(function (JointTypes) {
    JointTypes[JointTypes["REVOLUTE"] = 1] = "REVOLUTE";
    JointTypes[JointTypes["REVOLUTE2"] = 2] = "REVOLUTE2";
    JointTypes[JointTypes["PRISMATIC"] = 3] = "PRISMATIC";
    JointTypes[JointTypes["UNIVERSAL"] = 4] = "UNIVERSAL";
    JointTypes[JointTypes["BALL"] = 5] = "BALL";
    JointTypes[JointTypes["SCREW"] = 6] = "SCREW";
    JointTypes[JointTypes["GEARBOX"] = 7] = "GEARBOX";
    JointTypes[JointTypes["FIXED"] = 8] = "FIXED";
})(JointTypes || (JointTypes = {}));
/**
 * The scene is where everything is placed, from objects, to lights and cameras.
 *
 * Supports radial menu on an orthographic scene when gzradialmenu.js has been
 * included (useful for mobile devices).
 *
 * @param shaders Shaders instance, if not provided, custom shaders will
 *                not be set.
 * @param defaultCameraPosition THREE.Vector3 Default, and starting, camera
 *                              position. A value of [0, -5, 5] will be used
 *                              if this is undefined.
 * @param defaultCameraLookAt THREE.Vector3 Default, and starting, camera
 *                            lookAt position. A value of [0, 0, 0] will
 *                            be used if this is undefined.
 * @param backgroundColor THREE.Color The background color. A value of
 *                        0xb2b2b2 will be used if undefined.
 *
 * @param {function(resource)} findResourceCb - A function callback that can be used to help
 * @constructor
 */
export class Scene {
    constructor(config) {
        this.meshes = new Map();
        this.showCollisions = false;
        this.COMVisual = new THREE.Object3D();
        this.textureCache = new Map();
        this.currentThirdPersonLookAt = new THREE.Vector3();
        this.defaultThirdPersonCameraOffset = new THREE.Vector3(-6, -2, 1.5);
        this.currentThirdPersonCameraOffset = new THREE.Vector3();
        this.mousePointerDown = false;
        this.currentFirstPersonLookAt = new THREE.Vector3();
        /**
         * Create plane
         * @param {THREE.Vector3} normal
         * @param {double} width
         * @param {double} height
         * @returns {THREE.Mesh}
         */
        this.createPlane = function (normal, width, height) {
            // Create plane where width is along the x-axis and
            // and height along y-axi
            let geometry = new THREE.PlaneGeometry(width, height, 1, 1);
            // Manually specify the up vector to be along the z-axis since
            // the plane is created on XY plane
            let up = new THREE.Vector3(0, 0, 1);
            let material = new THREE.MeshPhongMaterial();
            let mesh = new THREE.Mesh(geometry, material);
            // Make sure the normal is normalized.
            normal = normal.normalize();
            // Rotate the plane according to the normal.
            let axis = new THREE.Vector3();
            axis.crossVectors(up, normal);
            mesh.setRotationFromAxisAngle(axis, normal.angleTo(up));
            mesh.updateMatrix();
            mesh.name = "plane";
            mesh.receiveShadow = true;
            return mesh;
        };
        this.emitter = new EventEmitter2({ verboseMemoryLeak: true });
        this.shaders = config.shaders;
        if (config.findResourceCb) {
            this.findResourceCb = config.findResourceCb;
        }
        // This matches Gazebo's default camera position
        this.defaultCameraPosition = new THREE.Vector3(-6, 0, 6);
        if (config.defaultCameraPosition) {
            this.defaultCameraPosition.copy(config.defaultCameraPosition);
        }
        this.defaultCameraLookAt = new THREE.Vector3(0, 0, 0);
        if (config.defaultCameraLookAt) {
            this.defaultCameraLookAt.copy(config.defaultCameraLookAt);
        }
        this.backgroundColor = new THREE.Color(0xb2b2b2);
        if (config.backgroundColor) {
            this.backgroundColor.copy(config.backgroundColor);
        }
        this.init();
        /**
         * @member {string} selectEntity
         * The select entity event name.
         */
        this.selectEntityEvent = "select_entity";
        /**
         * @member {string} followEntity
         * The follow entity event name.
         */
        this.followEntityEvent = "follow_entity";
        /**
         * @member {string} moveToEntity
         * The move to entity event name.
         */
        this.moveToEntityEvent = "move_to_entity";
        /**
         * @member {string} thirdPersonFollowEntity
         * The third-person follow entity event name.
         */
        this.thirdPersonFollowEntityEvent = "third_person_follow_entity";
        /**
         * @member {string} firstPersonEntity
         * The first-person camera entity event name.
         */
        this.firstPersonEntityEvent = "first_person_entity";
        var that = this;
        /**
         * Handle entity selection signal ('select_entity').
         * @param {string} entityName The name of the entity to select.
         */
        this.emitter.on(this.selectEntityEvent, function (entityName) {
            var object = that.scene.getObjectByName(entityName);
            if (object !== undefined && object !== null) {
                that.selectEntity(object);
            }
        });
        /**
         * Handle the follow entity follow signal ('follow_entity').
         * @param {string} entityName Name of the entity. Pass in null or an empty
         * string to stop following.
         */
        this.emitter.on(this.followEntityEvent, function (entityName) {
            // Turn off following if `entity` is null.
            if (entityName === undefined || entityName === null) {
                that.cameraMode = "";
                return;
            }
            var object = that.scene.getObjectByName(entityName);
            if (object !== undefined && object !== null) {
                // Set the object to track.
                that.cameraTrackObject = object;
                // Set the camera mode.
                that.cameraMode = that.followEntityEvent;
            }
        });
        /**
         * Handle the third-person follow entity signal ('third_person_follow_entity').
         * @param {string} entityName Name of the entity. Pass in null or an empty
         * string to stop third-person following.
         */
        this.emitter.on(this.thirdPersonFollowEntityEvent, function (entityName) {
            // Turn off following if `entity` is null.
            if (entityName === undefined || entityName === null) {
                that.cameraMode = "";
                return;
            }
            var object = that.scene.getObjectByName(entityName);
            if (object !== undefined && object !== null) {
                // Set the object to track.
                that.cameraTrackObject = object;
                // Set the camera offset to the default one.
                that.currentThirdPersonCameraOffset.copy(that.defaultThirdPersonCameraOffset);
                // Set the camera mode.
                that.cameraMode = that.thirdPersonFollowEntityEvent;
            }
        });
        /**
         * Handle the first-person entity signal ('first_person_entity').
         * @param {string} entityName Name of the entity. Pass in null or an empty
         * string to stop first-person following.
         */
        this.emitter.on(this.firstPersonEntityEvent, function (entityName) {
            // Turn off following if `entity` is null.
            if (entityName === undefined || entityName === null) {
                that.cameraMode = "";
                return;
            }
            var object = that.scene.getObjectByName(entityName);
            if (object !== undefined && object !== null) {
                // Set the object to track.
                that.cameraTrackObject = object;
                // Set the camera mode.
                that.cameraMode = that.firstPersonEntityEvent;
            }
        });
        /**
         * Handle move to entity signal ('move_to_entity').
         * @param {string} entityName: Name of the entity.
         */
        this.emitter.on(this.moveToEntityEvent, function (entityName) {
            var obj = that.scene.getObjectByName(entityName);
            if (obj === undefined || obj === null) {
                return;
            }
            // Starting position of the camera.
            var startPos = new THREE.Vector3();
            that.camera.getWorldPosition(startPos);
            // Center of the target to move to.
            var targetCenter = new THREE.Vector3();
            obj.getWorldPosition(targetCenter);
            // Calculate  direction from start to target
            var dir = new THREE.Vector3();
            dir.subVectors(targetCenter, startPos);
            dir.normalize();
            // Distance from start to target.
            var dist = startPos.distanceTo(targetCenter);
            // Get the bounding box size of the target object.
            var bboxSize = new THREE.Vector3();
            var bbox = new THREE.Box3().setFromObject(obj);
            bbox.getSize(bboxSize);
            var max = Math.max(bboxSize.x, bboxSize.y, bboxSize.z);
            // Compute an offset such that the object's bounding box will fix in the
            // view. I've padded this out a bit by multiplying `max` by 0.75 instead
            // of 0.5
            var offset = (max * 0.75) / Math.tan((that.camera.fov * Math.PI) / 180.0 / 2.0);
            var endPos = dir.clone().multiplyScalar(dist - offset);
            endPos.add(startPos);
            // Make sure that the end position is above the object so that the
            // camera will look down at it.
            if (endPos.z <= targetCenter.z + max) {
                endPos.z += max;
            }
            // Compute the end orientation.
            var endRotMat = new THREE.Matrix4();
            endRotMat.lookAt(endPos, targetCenter, new THREE.Vector3(0, 0, 1));
            // Start the camera moving.
            that.cameraMode = that.moveToEntityEvent;
            that.cameraMoveToClock.start();
            that.cameraLerpStart.copy(startPos);
            that.cameraLerpEnd.copy(endPos);
            that.camera.getWorldQuaternion(that.cameraSlerpStart);
            that.cameraSlerpEnd.setFromRotationMatrix(endRotMat);
        });
    }
    /**
     * Initialize scene
     */
    init() {
        THREE.Object3D.DefaultUp.set(0, 0, 1);
        this.name = "default";
        this.scene = new THREE.Scene();
        // this.scene.name = this.name;
        // only support one heightmap for now.
        this.heightmap = null;
        this.selectedEntity = null;
        this.manipulationMode = "view";
        this.pointerOnMenu = false;
        // loaders
        this.textureLoader = new THREE.TextureLoader();
        this.textureLoader.crossOrigin = "";
        this.colladaLoader = new ColladaLoader();
        this.stlLoader = new STLLoader();
        this.gltfLoader = new GLTFLoader();
        this.ddsLoader = new DDSLoader();
        // Progress and Load events.
        const progressEvent = (url, items, total) => {
            this.emitter.emit("load_progress", url, items, total);
        };
        const loadEvent = () => {
            this.emitter.emit("load_finished");
        };
        // Set the right loading manager for handling websocket assets.
        if (this.findResourceCb) {
            const wsLoadingManager = new WsLoadingManager(loadEvent, progressEvent);
            // Collada Loader uses the findResourceCb internally.
            this.colladaLoader.findResourceCb = this.findResourceCb;
            this.textureLoader.manager = wsLoadingManager;
            this.colladaLoader.manager = wsLoadingManager;
            this.stlLoader.manager = wsLoadingManager;
            this.gltfLoader.manager = wsLoadingManager;
            this.ddsLoader.manager = wsLoadingManager;
        }
        this.textureLoader.manager.onProgress = progressEvent;
        this.colladaLoader.manager.onProgress = progressEvent;
        this.stlLoader.manager.onProgress = progressEvent;
        this.gltfLoader.manager.onProgress = progressEvent;
        this.ddsLoader.manager.onProgress = progressEvent;
        this.textureLoader.manager.onLoad = loadEvent;
        this.colladaLoader.manager.onLoad = loadEvent;
        this.stlLoader.manager.onLoad = loadEvent;
        this.gltfLoader.manager.onLoad = loadEvent;
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.setClearColor(this.backgroundColor);
        this.renderer.autoClear = false;
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        // Particle group to render.
        // Add a default ambient value. This is equivalent to
        // {r: 0.1, g: 0.1, b: 0.1}.
        this.ambient = new THREE.AmbientLight(0x191919);
        this.scene.add(this.ambient);
        // camera
        let width = this.getDomElement().width;
        let height = this.getDomElement().height;
        this.camera = new THREE.PerspectiveCamera(60, width / height, 0.01, 1000);
        this.resetView();
        // Clock used to time the camera 'move_to' motion.
        this.cameraMoveToClock = new THREE.Clock(false);
        // Start position of the camera's move_to
        this.cameraLerpStart = new THREE.Vector3();
        // End position of the camera's move_to
        this.cameraLerpEnd = new THREE.Vector3();
        // Start orientation of the camera's move_to
        this.cameraSlerpStart = new THREE.Quaternion();
        // End orientation of the camera's move_to
        this.cameraSlerpEnd = new THREE.Quaternion();
        // Current camera mode. Empty indicates standard orbit camera.
        this.cameraMode = "";
        // Ortho camera and scene for rendering sprites
        // Currently only used for the radial menu
        /*if (typeof RadialMenu === 'function')
        {
          this.cameraOrtho = new THREE.OrthographicCamera(-width * 0.5, width * 0.5,
              height*0.5, -height*0.5, 1, 10);
          this.cameraOrtho.position.z = 10;
          this.sceneOrtho = new THREE.Scene();
    
          // Radial menu (only triggered by touch)
          // this.radialMenu = new RadialMenu(this.getDomElement());
          // this.sceneOrtho.add(this.radialMenu.menu);
        }*/
        // Grid
        this.grid = new THREE.GridHelper(20, 20, 0xcccccc, 0x4d4d4d);
        this.grid.name = "grid";
        this.grid.position.z = 0.05;
        this.grid.rotation.x = Math.PI * 0.5;
        this.grid.castShadow = false;
        this.grid.material.transparent = true;
        this.grid.material.opacity = 0.5;
        this.grid.visible = false;
        this.scene.add(this.grid);
        this.showCollisions = false;
        this.spawnModel = new SpawnModel(this, this.getDomElement());
        this.simpleShapesMaterial = new THREE.MeshPhongMaterial({
            color: 0xffffff,
            flatShading: false,
        });
        var that = this;
        // Only capture events inside the webgl div element.
        this.getDomElement().addEventListener("mouseup", function (event) {
            that.onPointerUp(event);
        }, false);
        this.getDomElement().addEventListener("mousedown", function (event) {
            that.onPointerDown(event);
        }, false);
        this.getDomElement().addEventListener("wheel", function (event) {
            that.onMouseScroll(event);
        }, false);
        /*this.getDomElement().addEventListener( 'touchstart',
            function(event: TouchEvent) {that.onPointerDown(event);}, false );
    
        this.getDomElement().addEventListener( 'touchend',
            function(event: TouchEvent) {that.onPointerUp(event);}, false );
           */
        // Handles for translating and rotating objects
        //this.modelManipulator = new Manipulator(this.camera, false,
        //    this.getDomElement());
        // this.timeDown = null;
        // Create a ray caster
        this.ray = new THREE.Raycaster();
        this.controls = new OrbitControls(this.camera, this.getDomElement());
        this.controls.mouseButtons = {
            LEFT: THREE.MOUSE.ROTATE,
            MIDDLE: THREE.MOUSE.DOLLY,
            RIGHT: THREE.MOUSE.PAN,
        };
        // an animation loop is required with damping
        this.controls.enableDamping = false;
        this.controls.screenSpacePanning = true;
        // Bounding Box
        var indices = new Uint16Array([
            0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7,
        ]);
        var positions = new Float32Array(8 * 3);
        var boxGeometry = new THREE.BufferGeometry();
        boxGeometry.setIndex(new THREE.BufferAttribute(indices, 1));
        boxGeometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
        this.boundingBox = new THREE.LineSegments(boxGeometry, new THREE.LineBasicMaterial({ color: 0xffffff }));
        this.boundingBox.visible = false;
        // Joint visuals
        this.jointAxis = new THREE.Object3D();
        this.jointAxis.name = "JOINT_VISUAL";
        var geometry, material, mesh;
        // XYZ
        var XYZaxes = new THREE.Object3D();
        geometry = new THREE.CylinderGeometry(0.01, 0.01, 0.3, 10, 1, false);
        material = new THREE.MeshBasicMaterial({
            color: new THREE.Color(0xff0000),
        });
        mesh = new THREE.Mesh(geometry, material);
        mesh.position.x = 0.15;
        mesh.rotation.z = -Math.PI / 2;
        mesh.name = "JOINT_VISUAL";
        XYZaxes.add(mesh);
        material = new THREE.MeshBasicMaterial({
            color: new THREE.Color(0x00ff00),
        });
        mesh = new THREE.Mesh(geometry, material);
        mesh.position.y = 0.15;
        mesh.name = "JOINT_VISUAL";
        XYZaxes.add(mesh);
        material = new THREE.MeshBasicMaterial({
            color: new THREE.Color(0x0000ff),
        });
        mesh = new THREE.Mesh(geometry, material);
        mesh.position.z = 0.15;
        mesh.rotation.x = Math.PI / 2;
        mesh.name = "JOINT_VISUAL";
        XYZaxes.add(mesh);
        geometry = new THREE.CylinderGeometry(0, 0.03, 0.1, 10, 1, true);
        material = new THREE.MeshBasicMaterial({
            color: new THREE.Color(0xff0000),
        });
        mesh = new THREE.Mesh(geometry, material);
        mesh.position.x = 0.3;
        mesh.rotation.z = -Math.PI / 2;
        mesh.name = "JOINT_VISUAL";
        XYZaxes.add(mesh);
        material = new THREE.MeshBasicMaterial({
            color: new THREE.Color(0x00ff00),
        });
        mesh = new THREE.Mesh(geometry, material);
        mesh.position.y = 0.3;
        mesh.name = "JOINT_VISUAL";
        XYZaxes.add(mesh);
        material = new THREE.MeshBasicMaterial({
            color: new THREE.Color(0x0000ff),
        });
        mesh = new THREE.Mesh(geometry, material);
        mesh.position.z = 0.3;
        mesh.rotation.x = Math.PI / 2;
        mesh.name = "JOINT_VISUAL";
        XYZaxes.add(mesh);
        this.jointAxis["XYZaxes"] = XYZaxes;
        var mainAxis = new THREE.Object3D();
        material = new THREE.MeshLambertMaterial();
        material.color = new THREE.Color(0xffff00);
        var mainAxisLen = 0.3;
        geometry = new THREE.CylinderGeometry(0.015, 0.015, mainAxisLen, 36, 1, false);
        mesh = new THREE.Mesh(geometry, material);
        mesh.position.z = mainAxisLen * 0.5;
        mesh.rotation.x = Math.PI / 2;
        mesh.name = "JOINT_VISUAL";
        mainAxis.add(mesh);
        geometry = new THREE.CylinderGeometry(0, 0.035, 0.1, 36, 1, false);
        mesh = new THREE.Mesh(geometry, material);
        mesh.position.z = mainAxisLen;
        mesh.rotation.x = Math.PI / 2;
        mesh.name = "JOINT_VISUAL";
        mainAxis.add(mesh);
        this.jointAxis["mainAxis"] = mainAxis;
        var rotAxis = new THREE.Object3D();
        geometry = new THREE.TorusGeometry(0.04, 0.006, 10, 36, (Math.PI * 3) / 2);
        mesh = new THREE.Mesh(geometry, material);
        mesh.position.z = mainAxisLen;
        mesh.name = "JOINT_VISUAL";
        rotAxis.add(mesh);
        geometry = new THREE.CylinderGeometry(0.015, 0, 0.025, 10, 1, false);
        mesh = new THREE.Mesh(geometry, material);
        mesh.position.y = -0.04;
        mesh.position.z = mainAxisLen;
        mesh.rotation.z = Math.PI / 2;
        mesh.name = "JOINT_VISUAL";
        rotAxis.add(mesh);
        this.jointAxis["rotAxis"] = rotAxis;
        var transAxis = new THREE.Object3D();
        geometry = new THREE.CylinderGeometry(0.01, 0.01, 0.1, 10, 1, true);
        mesh = new THREE.Mesh(geometry, material);
        mesh.position.x = 0.03;
        mesh.position.y = 0.03;
        mesh.position.z = mainAxisLen * 0.5;
        mesh.rotation.x = Math.PI / 2;
        mesh.name = "JOINT_VISUAL";
        transAxis.add(mesh);
        geometry = new THREE.CylinderGeometry(0.02, 0, 0.0375, 10, 1, false);
        mesh = new THREE.Mesh(geometry, material);
        mesh.position.x = 0.03;
        mesh.position.y = 0.03;
        mesh.position.z = mainAxisLen * 0.5 + 0.05;
        mesh.rotation.x = -Math.PI / 2;
        mesh.name = "JOINT_VISUAL";
        transAxis.add(mesh);
        mesh = new THREE.Mesh(geometry, material);
        mesh.position.x = 0.03;
        mesh.position.y = 0.03;
        mesh.position.z = mainAxisLen * 0.5 - 0.05;
        mesh.rotation.x = Math.PI / 2;
        mesh.name = "JOINT_VISUAL";
        transAxis.add(mesh);
        this.jointAxis["transAxis"] = transAxis;
        var screwAxis = new THREE.Object3D();
        mesh = new THREE.Mesh(geometry, material);
        mesh.position.x = -0.04;
        mesh.position.z = mainAxisLen - 0.11;
        mesh.rotation.z = -Math.PI / 4;
        mesh.rotation.x = -Math.PI / 10;
        mesh.name = "JOINT_VISUAL";
        screwAxis.add(mesh);
        var radius = 0.04;
        var length = 0.02;
        var curve = new THREE.CatmullRomCurve3([
            new THREE.Vector3(radius, 0, 0 * length),
            new THREE.Vector3(0, radius, 1 * length),
            new THREE.Vector3(-radius, 0, 2 * length),
            new THREE.Vector3(0, -radius, 3 * length),
            new THREE.Vector3(radius, 0, 4 * length),
            new THREE.Vector3(0, radius, 5 * length),
            new THREE.Vector3(-radius, 0, 6 * length),
        ]);
        geometry = new THREE.TubeGeometry(curve, 36, 0.01, 10, false);
        mesh = new THREE.Mesh(geometry, material);
        mesh.position.z = mainAxisLen - 0.23;
        mesh.name = "JOINT_VISUAL";
        screwAxis.add(mesh);
        this.jointAxis["screwAxis"] = screwAxis;
        var ballVisual = new THREE.Object3D();
        geometry = new THREE.SphereGeometry(0.06);
        mesh = new THREE.Mesh(geometry, material);
        mesh.name = "JOINT_VISUAL";
        ballVisual.add(mesh);
        this.jointAxis["ballVisual"] = ballVisual;
        // center of mass visual
        this.COMvisual = new THREE.Object3D();
        this.COMvisual.name = "COM_VISUAL";
        geometry = new THREE.SphereGeometry(1, 32, 32);
        mesh = new THREE.Mesh(geometry);
        // \todo: This should be fixed to point to a correct material.
        /*this.setMaterial(mesh, {'ambient':[0.5,0.5,0.5,1.000000],
          'texture':'assets/media/materials/textures/com.png'});
          */
        mesh.name = "COM_VISUAL";
        mesh.rotation.z = -Math.PI / 2;
        this.COMvisual.add(mesh);
    }
    addSky(cubemap) {
        if (cubemap === undefined) {
            const cubeLoader = new THREE.CubeTextureLoader();
            this.scene.background = cubeLoader.load([
                "https://fuel.gazebosim.org/1.0/openrobotics/models/skybox/tip/files/materials/textures/skybox-negx.jpg",
                "https://fuel.gazebosim.org/1.0/openrobotics/models/skybox/tip/files/materials/textures/skybox-posx.jpg",
                "https://fuel.gazebosim.org/1.0/openrobotics/models/skybox/tip/files/materials/textures/skybox-posy.jpg",
                "https://fuel.gazebosim.org/1.0/openrobotics/models/skybox/tip/files/materials/textures/skybox-negy.jpg",
                "https://fuel.gazebosim.org/1.0/openrobotics/models/skybox/tip/files/materials/textures/skybox-negz.jpg",
                "https://fuel.gazebosim.org/1.0/openrobotics/models/skybox/tip/files/materials/textures/skybox-posz.jpg",
            ]);
        }
        else {
            this.ddsLoader.load(cubemap, 
            // OnLoad callback that allows us to manipulate the texture.
            (compressedTexture) => {
                const images = [];
                const rawImages = compressedTexture.image;
                // Convert the binary data arrays to images
                for (let i = 0; i < rawImages.length; i++) {
                    const image = rawImages[i]["mipmaps"][0];
                    const imageElem = binaryToImage(image["data"], image["width"], image["height"]);
                    images.push(imageElem);
                }
                // Reorder the images to support ThreeJS coordinate system.
                const reorderImages = [
                    images[1],
                    images[0],
                    images[2],
                    images[3],
                    images[5],
                    images[4],
                ];
                // Create the cube texture
                this.scene.background = new THREE.CubeTexture(reorderImages, compressedTexture.mapping, compressedTexture.wrapS, compressedTexture.wrapT, compressedTexture.magFilter, compressedTexture.minFilter, compressedTexture.format, compressedTexture.type, compressedTexture.anisotropy, compressedTexture.encoding);
                this.scene.background.needsUpdate = true;
            }, 
            // OnProgress, do nothing
            () => { }, 
            // OnError
            (error) => {
                if (this.findResourceCb) {
                    // Get the mesh from the websocket server.
                    this.findResourceCb(cubemap, (material, error) => {
                        if (error !== undefined) {
                            // Mark the texture as error in the loading manager.
                            const manager = this.ddsLoader.manager;
                            manager.markAsError(cubemap);
                            return;
                        }
                        // Parse the DDS data.
                        const texDatas = this.ddsLoader.parse(material.buffer.slice(material.byteOffset), true);
                        const images = [];
                        let texture;
                        // This `if` statement was taken from https://github.com/mrdoob/three.js/blob/master/src/loaders/CompressedTextureLoader.js#L83
                        if (texDatas["isCubemap"]) {
                            const faces = texDatas["mipmaps"].length / texDatas["mipmapCount"];
                            for (let f = 0; f < faces; f++) {
                                for (let i = 0; i < texDatas["mipmapCount"]; i++) {
                                    let data = texDatas["mipmaps"][f * texDatas["mipmapCount"] + i]["data"];
                                    // Convert binary data to an image
                                    let imageElem = binaryToImage(data, texDatas["width"], texDatas["height"]);
                                    images.push(imageElem);
                                }
                            }
                        }
                        else {
                            console.error("Texture is not a cubemap. Sky will not be set.");
                            // Mark the texture as error in the loading manager.
                            const manager = this.ddsLoader.manager;
                            manager.markAsError(cubemap);
                            return;
                        }
                        // Reorder the images to support ThreeJS coordinate system.
                        const reorderImages = [
                            images[1],
                            images[0],
                            images[2],
                            images[3],
                            images[5],
                            images[4],
                        ];
                        this.scene.background = new THREE.CubeTexture(reorderImages);
                        this.scene.background.format = (texDatas["format"]);
                        if (texDatas["mipmapCount"] === 1) {
                            this.scene.background.minFilter = THREE.LinearFilter;
                        }
                        this.scene.background.needsUpdate = true;
                        // Mark the texture as done in the loading manager.
                        const manager = this.ddsLoader.manager;
                        manager.markAsDone(cubemap);
                    });
                }
            });
        }
    }
    /**
     * Add Fog to the scene.
     *
     * @param color Color can be a hexadecimal integer (recommended) or a CSS-style string.
     * @param density Defines how fast the fog will grow dense.
     * @param changeBackground Whether or not change the scene's background color accordingly.
     */
    addFog(color, density, changeBackground) {
        this.scene.fog = new THREE.FogExp2(color, density);
        if (changeBackground === true) {
            this.scene.background = new THREE.Color(color);
        }
    }
    initScene() {
        this.emitter.emit("show_grid", "show");
    }
    setSDFParser(sdfParser) {
        this.spawnModel.sdfParser = sdfParser;
    }
    /**
     * Window event callback
     * @param {} event - mousedown or touchdown events
     */
    onPointerDown(event) {
        event.preventDefault();
        this.mousePointerDown = true;
        if (this.spawnModel.active) {
            return;
        }
        var mainPointer = true;
        let pos;
        /*if (event.touches)
        {
          if (event.touches.length === 1)
          {
            pos = new THREE.Vector2(
                event.touches[0].clientX, event.touches[0].clientY);
          }
          else if (event.touches.length === 2)
          {
            pos = new THREE.Vector2(
                (event.touches[0].clientX + event.touches[1].clientX)/2,
                (event.touches[0].clientY + event.touches[1].clientY)/2);
          }
          else
          {
            return;
          }
        }
        else
        {*/
        pos = new THREE.Vector2(event.clientX, event.clientY);
        if (event.which !== 1) {
            mainPointer = false;
        }
        //}
        var intersect = new THREE.Vector3();
        var model = this.getRayCastModel(pos, intersect);
        if (intersect) {
            this.controls.target = intersect;
        }
        // Cancel in case of multitouch
        /*if (event.touches && event.touches.length !== 1)
        {
          return;
        }*/
        // Manipulation modes
        // Model found
        if (model) {
            // Do nothing to the floor plane
            if (model.name === "plane") {
                // this.timeDown = new Date().getTime();
            }
            /*else if (this.modelManipulator.pickerNames.indexOf(model.name) >= 0)
            {
              // Do not attach manipulator to itself
            }*/
            // Attach manipulator to model
            else if (model.name !== "") {
                if (mainPointer && model.parent === this.scene) {
                    //this.selectEntity(model);
                }
            }
            // Manipulator pickers, for mouse
            /*else if (this.modelManipulator.hovered)
            {
              this.modelManipulator.update();
              this.modelManipulator.object.updateMatrixWorld();
            }*/
            // Sky
            else {
                // this.timeDown = new Date().getTime();
            }
        }
        // Plane from below, for example
        else {
            // this.timeDown = new Date().getTime();
        }
    }
    /**
     * Window event callback
     * @param {} event - mouseup or touchend events
     */
    onPointerUp(event) {
        event.preventDefault();
        this.mousePointerDown = false;
        if (this.cameraMode === this.thirdPersonFollowEntityEvent) {
            // Calculate and store the new relative fixed camera position.
            // The offset we get in this.camera.position is in world coordinates,
            // but we want it relative to the object we are tracking.  Therefore,
            // do the inverse of what we do in render, namely:
            // 1. subtract the position of the tracked object
            // 2. Apply the inverse (conjugate) quaternion of the tracked object
            this.currentThirdPersonCameraOffset = this.camera.position.clone();
            this.currentThirdPersonCameraOffset.sub(this.cameraTrackObject.position);
            this.currentThirdPersonCameraOffset.applyQuaternion(this.cameraTrackObject.quaternion.conjugate());
        }
        // Clicks (<150ms) outside any models trigger view mode
        // var millisecs = new Date().getTime();
        /*if (millisecs - this.timeDown < 150)
        {
          this.setManipulationMode('view');
          // TODO: Remove jquery from scene
          if (typeof Gui === 'function')
          {
            $( '#view-mode' ).click();
            $('input[type="radio"]').checkboxradio('refresh');
          }
        }*/
        // this.timeDown = null;
    }
    /**
     * Window event callback
     * @param {} event - mousescroll event
     */
    onMouseScroll(event) {
        event.preventDefault();
        const pos = new THREE.Vector2(event.clientX, event.clientY);
        let intersect = new THREE.Vector3();
        let model = this.getRayCastModel(pos, intersect);
        if (intersect) {
            this.controls.target = intersect;
        }
    }
    /**
     * Window event callback
     * @param {} event - keydown events
     */
    /*public onKeyDown(event: MouseEvent): void {
      if (event.shiftKey)
      {
        // + and - for zooming
        if (event.keyCode === 187 || event.keyCode === 189)
        {
          var pos = new THREE.Vector2(this.getDomElement().width/2.0,
              this.getDomElement().height/2.0);
  
          var intersect = new THREE.Vector3();
          var model = this.getRayCastModel(pos, intersect);
  
          if (intersect)
          {
            this.controls.target = intersect;
          }
  
          if (event.keyCode === 187)
          {
            this.controls.dollyOut();
          }
          else
          {
            this.controls.dollyIn();
          }
        }
      }
  
      // DEL to delete entities
      if (event.keyCode === 46)
      {
        if (this.selectedEntity)
        {
          this.emitter.emit('delete_entity');
        }
      }
  
      // F2 for turning on effects
      if (event.keyCode === 113)
      {
        // this.effectsEnabled = !this.effectsEnabled;
      }
  
      // Esc/R/T for changing manipulation modes
      // TODO: Remove jquery from scene
      if (typeof Gui === 'function')
      {
        if (event.keyCode === 27) // Esc
        {
          $( '#view-mode' ).click();
          $('input[type="radio"]').checkboxradio('refresh');
        }
        if (event.keyCode === 82) // R
        {
          $( '#rotate-mode' ).click();
          $('input[type="radio"]').checkboxradio('refresh');
        }
        if (event.keyCode === 84) // T
        {
          $( '#translate-mode' ).click();
          $('input[type="radio"]').checkboxradio('refresh');
        }
      }
    }*/
    /**
     * Check if there's a model immediately under canvas coordinate 'pos'
     * @param {THREE.Vector2} pos - Canvas coordinates
     * @param {THREE.Vector3} intersect - Empty at input,
     * contains point of intersection in 3D world coordinates at output
     * @returns {THREE.Object3D} model - Intercepted model closest to the camera
     */
    getRayCastModel(pos, intersect) {
        var rect = this.getDomElement().getBoundingClientRect();
        var vector = new THREE.Vector2(((pos.x - rect.x) / rect.width) * 2 - 1, -((pos.y - rect.y) / rect.height) * 2 + 1);
        this.ray.setFromCamera(vector, this.camera);
        let allObjects = [];
        getDescendants(this.scene, allObjects);
        let objects = this.ray.intersectObjects(allObjects);
        let model = new THREE.Object3D();
        var point;
        if (objects.length > 0) {
            modelsloop: for (var i = 0; i < objects.length; ++i) {
                model = objects[i].object;
                if (model.name.indexOf("_lightHelper") >= 0) {
                    model = model.parent;
                    break;
                }
                /*if (!this.modelManipulator.hovered &&
                    (model.name === 'plane'))
                {
                  // model = null;
                  point = objects[i].point;
                  break;
                }*/
                if (model.name === "grid" ||
                    model.name === "boundingBox" ||
                    model.name === "JOINT_VISUAL" ||
                    model.name === "INERTIA_VISUAL" ||
                    model.name === "COM_VISUAL") {
                    point = objects[i].point;
                    continue;
                }
                while (model.parent !== this.scene) {
                    // Select current mode's handle
                    /*if (model.parent.parent === this.modelManipulator.gizmo &&
                        ((this.manipulationMode === 'translate' &&
                          model.name.indexOf('T') >=0) ||
                         (this.manipulationMode === 'rotate' &&
                           model.name.indexOf('R') >=0)))
                    {
                      break modelsloop;
                    }*/
                    model = model.parent;
                }
                /*if (this.radialMenu && model === this.radialMenu.menu)
                {
                  continue;
                }*/
                if (model.name.indexOf("COLLISION_VISUAL") >= 0) {
                    continue;
                }
                else if (model.name !== "") {
                    point = objects[i].point;
                    break;
                }
            }
        }
        if (point) {
            intersect.x = point.x;
            intersect.y = point.y;
            intersect.z = point.z;
        }
        return model;
    }
    /**
     * Get the renderer's DOM element
     * @returns {domElement}
     */
    getDomElement() {
        return this.renderer.domElement;
    }
    /**
     * Render scene
     */
    render(timeElapsedMs) {
        // Kill camera control when:
        // -manipulating
        // -using radial menu
        // -pointer over menus
        // -spawning
        /* Disabling this for now so that mouse control stays enabled when the
         * mouse leaves the viewport.
         * if (this.modelManipulator.hovered ||
            (this.radialMenu && this.radialMenu.showing) ||
            this.pointerOnMenu ||
            this.spawnModel.active)
        {
          this.controls.enabled = false;
        }
        else
        {
          this.controls.enabled = true;
        }*/
        this.controls.update();
        // If 'follow' mode, then track the specific object.
        if (this.cameraMode === this.followEntityEvent) {
            // Using a hard-coded offset for now.
            var relativeCameraOffset = new THREE.Vector3(-5, 0, 2);
            this.cameraTrackObject.updateMatrixWorld();
            var cameraOffset = relativeCameraOffset.applyMatrix4(this.cameraTrackObject.matrixWorld);
            this.camera.position.lerp(cameraOffset, 0.1);
            this.camera.lookAt(this.cameraTrackObject.position);
        }
        else if (this.cameraMode === this.thirdPersonFollowEntityEvent &&
            !this.mousePointerDown) {
            // Based on https://discoverthreejs.com/book/first-steps/transformations/ ,
            // in THREE.js we have the following coordinate system:
            //
            // +X - Across the camera, to the right
            // -X - Across the camera, to the left
            // +Y - Up relative to the camera
            // -Y - Down relative to the camera
            // +Z - Towards the camera
            // -Z - Away from the camera
            let fixedCameraOffset = this.currentThirdPersonCameraOffset.clone();
            fixedCameraOffset.applyQuaternion(this.cameraTrackObject.quaternion);
            fixedCameraOffset.add(this.cameraTrackObject.position);
            let fixedLookAt = new THREE.Vector3(12, -4, 0);
            fixedLookAt.applyQuaternion(this.cameraTrackObject.quaternion);
            fixedLookAt.add(this.cameraTrackObject.position);
            // The calculation here comes from:
            // https://github.com/simondevyoutube/ThreeJS_Tutorial_ThirdPersonCamera/blob/main/main.js
            const timeElapsedSec = timeElapsedMs * 0.001;
            const timestep = 2.0 * timeElapsedSec;
            this.currentThirdPersonLookAt.lerp(fixedLookAt, timestep);
            this.camera.position.lerp(fixedCameraOffset, timestep);
            this.camera.lookAt(this.currentThirdPersonLookAt);
        }
        else if (this.cameraMode === this.firstPersonEntityEvent) {
            // Based on https://discoverthreejs.com/book/first-steps/transformations/ ,
            // in THREE.js we have the following coordinate system:
            //
            // +X - Across the camera, to the right
            // -X - Across the camera, to the left
            // +Y - Up relative to the camera
            // -Y - Down relative to the camera
            // +Z - Towards the camera
            // -Z - Away from the camera
            let fixedCameraOffset = new THREE.Vector3(-0.12, 0, 0.6);
            fixedCameraOffset.applyQuaternion(this.cameraTrackObject.quaternion);
            fixedCameraOffset.add(this.cameraTrackObject.position);
            let fixedLookAt = new THREE.Vector3(6, 0, 0);
            fixedLookAt.applyQuaternion(this.cameraTrackObject.quaternion);
            fixedLookAt.add(this.cameraTrackObject.position);
            // This is a pretty aggressive timestamp for lerping that makes the camera
            // bob a lot with the motion of the vehicle.  But I think it is what we want;
            // first-person camera should more-or-less feel like it is tied to the vehicle.
            const timestep = 0.5;
            this.currentFirstPersonLookAt.lerp(fixedLookAt, timestep);
            this.camera.position.lerp(fixedCameraOffset, timestep);
            this.camera.lookAt(this.currentFirstPersonLookAt);
        }
        else if (this.cameraMode === this.moveToEntityEvent) {
            // Move the camera if "lerping" to an object.
            // Compute the lerp factor.
            var lerp = this.cameraMoveToClock.getElapsedTime() / 2.0;
            // Stop the clock if the camera has reached it's target
            //if (Math.abs(1.0 - lerp) <= 0.005) {
            if (lerp >= 1.0) {
                this.cameraMoveToClock.stop();
                this.cameraMode = "";
            }
            else {
                // Move the camera's position.
                this.camera.position.lerpVectors(this.cameraLerpStart, this.cameraLerpEnd, lerp);
                // Move the camera's orientation.
                THREE.Quaternion.slerp(this.cameraSlerpStart, this.cameraSlerpEnd, this.camera.quaternion, lerp);
            }
        }
        // this.modelManipulator.update();
        /*if (this.radialMenu)
        {
          this.radialMenu.update();
        }*/
        this.renderer.clear();
        this.renderer.render(this.scene, this.camera);
        this.renderer.clearDepth();
        if (this.sceneOrtho && this.cameraOrtho) {
            this.renderer.render(this.sceneOrtho, this.cameraOrtho);
        }
    }
    /**
     * Set scene size.
     * @param {double} width
     * @param {double} height
     */
    setSize(width, height) {
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        if (this.cameraOrtho) {
            this.cameraOrtho.left = -width / 2;
            this.cameraOrtho.right = width / 2;
            this.cameraOrtho.top = height / 2;
            this.cameraOrtho.bottom = -height / 2;
            this.cameraOrtho.updateProjectionMatrix();
        }
        this.renderer.setSize(width, height);
        this.render(0);
    }
    /**
     * Add object to the scene
     * @param {THREE.Object3D} model
     */
    add(model) {
        if (!model.userData) {
            model.userData = new ModelUserData();
        }
        this.scene.add(model);
    }
    /**
     * Remove object from the scene
     * @param {THREE.Object3D} model
     */
    remove(model) {
        this.scene.remove(model);
    }
    /**
     * Returns the object which has the given name
     * @param {string} name
     * @returns {THREE.Object3D} model
     */
    getByName(name) {
        return this.scene.getObjectByName(name);
    }
    /**
     * Returns the object which has the given property value
     * @param {string} property name to search for
     * @param {string} value of the given property
     * @returns {THREE.Object3D} model
     */
    getByProperty(property, value) {
        return this.scene.getObjectByProperty(property, value);
    }
    /**
     * Update a model's pose
     * @param {THREE.Object3D} model
     * @param {} position
     * @param {} orientation
     */
    updatePose(model, position, orientation) {
        /*if (this.modelManipulator && this.modelManipulator.object &&
            this.modelManipulator.hovered)
        {
          return;
        }*/
        this.setPose(model, position, orientation);
    }
    /**
     * Set a model's pose
     * @param {THREE.Object3D} model
     * @param {} position
     * @param {} orientation
     */
    setPose(model, position, orientation) {
        model.position.x = position.x;
        model.position.y = position.y;
        model.position.z = position.z;
        model.quaternion.w = orientation.w;
        model.quaternion.x = orientation.x;
        model.quaternion.y = orientation.y;
        model.quaternion.z = orientation.z;
    }
    removeAll() {
        while (this.scene.children.length > 0) {
            this.scene.remove(this.scene.children[0]);
        }
    }
    /**
     * Create sphere
     * @param {double} radius
     * @returns {THREE.Mesh}
     */
    createSphere(radius) {
        var geometry = new THREE.SphereGeometry(radius, 32, 32);
        var mesh = new THREE.Mesh(geometry, this.simpleShapesMaterial);
        return mesh;
    }
    /**
     * Create cylinder
     * @param {double} radius
     * @param {double} length
     * @returns {THREE.Mesh}
     */
    createCylinder(radius, length) {
        var geometry = new THREE.CylinderGeometry(radius, radius, length, 32, 1, false);
        var mesh = new THREE.Mesh(geometry, this.simpleShapesMaterial);
        mesh.rotation.x = Math.PI * 0.5;
        return mesh;
    }
    /**
     * Create capsule
     * @param {double} radius
     * @param {double} length
     * @returns {THREE.Mesh}
     */
    createCapsule(radius, length) {
        var geometry = new THREE.CapsuleGeometry(radius, length, 32, 16);
        var mesh = new THREE.Mesh(geometry, this.simpleShapesMaterial);
        mesh.rotation.x = Math.PI * 0.5;
        return mesh;
    }
    /**
     * Create cone
     * @param {double} radius
     * @param {double} length
     * @returns {THREE.Mesh}
     */
    createCone(radius, length) {
        var geometry = new THREE.ConeGeometry(radius, length, 32);
        var mesh = new THREE.Mesh(geometry, this.simpleShapesMaterial);
        mesh.rotation.x = Math.PI * 0.5;
        return mesh;
    }
    /**
     * Create ellipsoid
     * @param {double} radius
     * @param {double} length
     * @returns {THREE.Mesh}
     */
    createEllipsoid(radius1, radius2, radius3) {
        var geometry = new THREE.SphereGeometry(radius1, 32, 32);
        geometry.scale(1, radius3 / radius1, radius2 / radius1);
        var mesh = new THREE.Mesh(geometry, this.simpleShapesMaterial);
        mesh.rotation.x = Math.PI * 0.5;
        return mesh;
    }
    /**
     * Create box
     * @param {double} width
     * @param {double} height
     * @param {double} depth
     * @returns {THREE.Mesh}
     */
    createBox(width, height, depth) {
        let geometry = new THREE.BoxGeometry(width, height, depth, 1, 1, 1);
        // Fix UVs so textures are mapped in a way that is consistent to gazebo
        // Some face uvs need to be rotated clockwise, while others anticlockwise
        // After updating to threejs rev 62, geometries changed from quads (6 faces)
        // to triangles (12 faces).
        var faceUVFixA = [1, 4, 5];
        var faceUVFixB = [0];
        let uvAttribute = geometry.getAttribute("uv");
        /* THREEJS has moved away from faceVertexUvs to BufferGeometry attributes.
         * Need to migrate this code. See https://discourse.threejs.org/t/facevertexuvs-for-buffergeometry/23040
        for (var i = 0; i < faceUVFixA.length; ++i)
        {
          var idx = faceUVFixA[i]*2;
          // Make sure that the index is valid. A threejs box geometry may not
          // have all of the faces if a dimension is sufficiently small.
          if (idx + 1 < geometry.faceVertexUvs.length) {
            var uva = geometry.faceVertexUvs[0][idx][0];
            geometry.faceVertexUvs[0][idx][0] = geometry.faceVertexUvs[0][idx][1];
            geometry.faceVertexUvs[0][idx][1] = geometry.faceVertexUvs[0][idx+1][1];
            geometry.faceVertexUvs[0][idx][2] = uva;
    
            geometry.faceVertexUvs[0][idx+1][0] = geometry.faceVertexUvs[0][idx+1][1];
            geometry.faceVertexUvs[0][idx+1][1] = geometry.faceVertexUvs[0][idx+1][2];
            geometry.faceVertexUvs[0][idx+1][2] = geometry.faceVertexUvs[0][idx][2];
          }
        }
        for (var ii = 0; ii < faceUVFixB.length; ++ii)
        {
          var idxB = faceUVFixB[ii]*2;
    
          // Make sure that the index is valid. A threejs box geometry may not
          // have all of the faces if a dimension is sufficiently small.
          if (idxB+1 < geometry.faceVertexUvs.length) {
            var uvc = geometry.faceVertexUvs[0][idxB][0];
            geometry.faceVertexUvs[0][idxB][0] = geometry.faceVertexUvs[0][idxB][2];
            geometry.faceVertexUvs[0][idxB][1] = uvc;
            geometry.faceVertexUvs[0][idxB][2] = geometry.faceVertexUvs[0][idxB+1][1];
    
            geometry.faceVertexUvs[0][idxB+1][2] = geometry.faceVertexUvs[0][idxB][2];
            geometry.faceVertexUvs[0][idxB+1][1] = geometry.faceVertexUvs[0][idxB+1][0];
            geometry.faceVertexUvs[0][idxB+1][0] = geometry.faceVertexUvs[0][idxB][1];
          }
        }
       */
        uvAttribute.needsUpdate = true;
        var mesh = new THREE.Mesh(geometry, this.simpleShapesMaterial);
        mesh.castShadow = true;
        return mesh;
    }
    /**
     * Create light
     * @param {} type - 1: point, 2: spot, 3: directional
     * @param {} diffuse
     * @param {} intensity
     * @param {} pose
     * @param {} distance
     * @param {} cast_shadows
     * @param {} name
     * @param {} direction
     * @param {} specular
     * @param {} attenuation_constant
     * @param {} attenuation_linear
     * @param {} attenuation_quadratic
     * @returns {THREE.Object3D}
     */
    createLight(type, diffuse, intensity, pose, distance, cast_shadows, name, direction, specular, attenuation_constant, attenuation_linear, attenuation_quadratic, inner_angle, outer_angle, falloff) {
        let obj = new THREE.Object3D();
        if (typeof diffuse === "undefined") {
            diffuse = new Color();
            diffuse.r = 1;
            diffuse.g = 1;
            diffuse.b = 1;
            diffuse.a = 1;
        }
        if (pose) {
            this.setPose(obj, pose.position, pose.orientation);
            obj.matrixWorldNeedsUpdate = true;
        }
        let lightObj;
        if (type === 1) {
            lightObj = this.createPointLight(obj, diffuse, intensity, distance, cast_shadows);
        }
        else if (type === 2) {
            lightObj = this.createSpotLight(obj, diffuse, intensity, distance, cast_shadows, inner_angle, outer_angle, falloff, direction);
        }
        else if (type === 3) {
            lightObj = this.createDirectionalLight(obj, diffuse, intensity, cast_shadows, direction);
        }
        else {
            console.error("Unknown light type", type);
            return obj;
        }
        if (name) {
            lightObj.name = name;
            obj.name = name;
        }
        obj.add(lightObj);
        return obj;
    }
    /**
     * Create point light - called by createLight
     * @param {} obj - light object
     * @param {} color
     * @param {} intensity
     * @param {} distance
     * @param {} cast_shadows
     * @returns {Object.<THREE.Light, THREE.Mesh>}
     */
    createPointLight(obj, color, intensity, distance, cast_shadows) {
        if (typeof intensity === "undefined") {
            intensity = 0.5;
        }
        var lightObj = new THREE.PointLight(color, intensity);
        if (distance) {
            lightObj.distance = distance;
        }
        if (cast_shadows) {
            lightObj.castShadow = cast_shadows;
        }
        return lightObj;
    }
    /**
     * Create spot light - called by createLight
     * @param {} obj - light object
     * @param {} color
     * @param {} intensity
     * @param {} distance
     * @param {} cast_shadows
     * @returns {Object.<THREE.Light, THREE.Mesh>}
     */
    createSpotLight(obj, color, intensity, distance, cast_shadows, inner_angle, outer_angle, falloff, direction) {
        if (typeof intensity === "undefined") {
            intensity = 1;
        }
        if (typeof distance === "undefined") {
            distance = 20;
        }
        let lightObj = new THREE.SpotLight(color, intensity, distance);
        lightObj.position.set(0, 0, 0);
        if (inner_angle !== null && outer_angle !== null) {
            lightObj.angle = outer_angle;
            lightObj.penumbra = Math.max(1, (outer_angle - inner_angle) / ((inner_angle + outer_angle) / 2.0));
        }
        if (falloff !== null) {
            lightObj.decay = falloff;
        }
        if (cast_shadows) {
            lightObj.castShadow = cast_shadows;
        }
        // Set the target
        let dir = new THREE.Vector3(0, 0, -1);
        if (direction) {
            dir.x = direction.x;
            dir.y = direction.y;
            dir.z = direction.z;
        }
        let targetObj = new THREE.Object3D();
        lightObj.add(targetObj);
        targetObj.position.copy(dir);
        targetObj.matrixWorldNeedsUpdate = true;
        lightObj.target = targetObj;
        return lightObj;
    }
    /**
     * Create directional light - called by createLight
     * @param {} obj - light object
     * @param {} color
     * @param {} intensity
     * @param {} cast_shadows
     * @returns {Object.<THREE.Light, THREE.Mesh>}
     */
    createDirectionalLight(obj, color, intensity, cast_shadows, direction) {
        if (typeof intensity === "undefined") {
            intensity = 1;
        }
        var lightObj = new THREE.DirectionalLight(color, intensity);
        lightObj.shadow.camera.near = 1;
        lightObj.shadow.camera.far = 50;
        lightObj.shadow.mapSize.width = 4094;
        lightObj.shadow.mapSize.height = 4094;
        lightObj.shadow.camera.bottom = -100;
        lightObj.shadow.camera.right = 100;
        lightObj.shadow.camera.top = 100;
        lightObj.shadow.bias = 0.0001;
        lightObj.position.set(0, 0, 0);
        if (cast_shadows) {
            lightObj.castShadow = cast_shadows;
        }
        // Set the target
        let dir = new THREE.Vector3(0, 0, -1);
        if (direction) {
            dir.x = direction.x;
            dir.y = direction.y;
            dir.z = direction.z;
        }
        let targetObj = new THREE.Object3D();
        lightObj.add(targetObj);
        targetObj.position.copy(dir);
        targetObj.matrixWorldNeedsUpdate = true;
        lightObj.target = targetObj;
        return lightObj;
    }
    /**
     * Load heightmap
     * @param {} heights Lookup table of heights
     * @param {} width Width of the heightmap in meters
     * @param {} height Height of the heightmap in meters
     * @param {} segmentWidth Size of lookup table
     * @param {} segmentHeight Size of lookup table
     * @param {} origin Heightmap position in the world
     * @param {} textures
     * @param {} blends
     * @param {} parent
     */
    loadHeightmap(heights, width, height, segmentWidth, segmentHeight, origin, textures, blends, parent) {
        if (this.heightmap) {
            console.error("Only one heightmap can be loaded at a time");
            return;
        }
        if (parent === undefined) {
            console.error("Missing parent, heightmap won't be loaded.");
            return;
        }
        // unfortunately large heightmaps kill the fps and freeze everything so
        // we have to scale it down
        let scale = 1;
        const maxHeightmapWidth = 256;
        const maxHeightmapHeight = 256;
        if (segmentWidth - 1 > maxHeightmapWidth) {
            scale = maxHeightmapWidth / (segmentWidth - 1);
        }
        let geometry = new THREE.PlaneGeometry(width, height, (segmentWidth - 1) * scale, (segmentHeight - 1) * scale);
        let posAttribute = geometry.getAttribute("position");
        // Sub-sample
        let col = (segmentWidth - 1) * scale;
        let row = (segmentHeight - 1) * scale;
        for (let r = 0; r < row; ++r) {
            for (let c = 0; c < col; ++c) {
                let index = (r * col * 1) / (scale * scale) + c * (1 / scale);
                posAttribute.setZ(r * col + c, heights[index]);
            }
        }
        posAttribute.needsUpdate = true;
        // Compute normals
        geometry.normalizeNormals();
        geometry.computeVertexNormals();
        let material;
        // NOTE: Texture might be an array of textures, that need to blend in between.
        // For now, it only uses one texture.
        if (textures && textures.length > 0) {
            const texturesLoaded = [];
            const normalsLoaded = [];
            // Auxiliar method to configurate a texture's repeat and wrapping.
            function configTexture(texture, repeat) {
                texture.wrapS = THREE.RepeatWrapping;
                texture.wrapT = THREE.RepeatWrapping;
                texture.repeat.copy(repeat);
            }
            for (let t = 0; t < textures.length; ++t) {
                const diffuseUri = createFuelUri(textures[t].diffuse);
                texturesLoaded[t] = this.loadTexture(diffuseUri);
                configTexture(texturesLoaded[t], new THREE.Vector2(width / textures[t].size, height / textures[t].size));
                let normalUri;
                if (textures[t].normal) {
                    normalUri = createFuelUri(textures[t].normal);
                    normalsLoaded[t] = this.loadTexture(normalUri);
                    configTexture(normalsLoaded[t], new THREE.Vector2(width / textures[t].size, height / textures[t].size));
                }
            }
            const materialOptions = {};
            if (texturesLoaded[0]) {
                materialOptions.map = texturesLoaded[0];
            }
            if (normalsLoaded[0]) {
                materialOptions.normalMap = normalsLoaded[0];
            }
            material = new THREE.MeshStandardMaterial(materialOptions);
        }
        else {
            material = new THREE.MeshPhongMaterial({ color: 0x555555 });
        }
        const mesh = new THREE.Mesh(geometry, material);
        mesh.receiveShadow = true;
        mesh.castShadow = false;
        mesh.position.x = origin.x;
        mesh.position.y = origin.y;
        mesh.position.z = origin.z;
        parent.add(mesh);
        this.heightmap = parent;
    }
    /**
     * Load mesh
     * @example
     * // loading using URI
     * // callback(mesh)
     * loadMeshFromUri('assets/house_1/meshes/house_1.dae', undefined, undefined, function(mesh)
                {
                  // use the mesh
                });
     * @param {string} uri
     * @param {} submesh
     * @param {} centerSubmesh
     * find a resource.
     * @param {function} onLoad
     * @param {function} onError
     */
    loadMeshFromUri(uri, submesh, centerSubmesh, onLoad, onError) {
        var uriPath = uri.substring(0, uri.lastIndexOf("/"));
        var uriFile = uri.substring(uri.lastIndexOf("/") + 1);
        // Check if the mesh has already been loaded.
        // Use it in that case.
        if (this.meshes.has(uri)) {
            let mesh = this.meshes.get(uri).clone();
            if (submesh && this.useSubMesh(mesh, submesh, centerSubmesh)) {
                onLoad(mesh);
            }
            else if (!submesh) {
                onLoad(mesh);
            }
            return;
        }
        // load meshes
        if (uriFile.substr(-4).toLowerCase() === ".dae") {
            return this.loadCollada(uri, submesh, centerSubmesh, onLoad, onError);
        }
        else if (uriFile.substr(-4).toLowerCase() === ".obj") {
            return this.loadOBJ(uri, submesh, centerSubmesh, onLoad, onError);
        }
        else if (uriFile.substr(-4).toLowerCase() === ".stl") {
            return this.loadSTL(uri, submesh, centerSubmesh, onLoad, onError);
        }
        else if (uriFile.substr(-4).toLowerCase() === ".glb" ||
            uriFile.substr(-5).toLowerCase() === ".gltf") {
            return this.loadGLTF(uri, submesh, centerSubmesh, onLoad, onError);
        }
        else if (uriFile.substr(-5).toLowerCase() === ".urdf") {
            console.error("Attempting to load URDF file, but it's not supported.");
        }
    }
    /**
     * Load mesh
     * @example
     * // loading using URI
     * // callback(mesh)
     * @example
     * // loading using file string
     * // callback(mesh)
     * loadMeshFromString('assets/house_1/meshes/house_1.dae', undefined, undefined, function(mesh)
                {
                  // use the mesh
                }, ['<?xml version="1.0" encoding="utf-8"?>
        <COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
          <asset>
            <contributor>
              <author>Cole</author>
              <authoring_tool>OpenCOLLADA for 3ds Max;  Ver.....']);
     * @param {string} uri
     * @param {} submesh
     * @param {} centerSubmesh
     * @param {function} onLoad
     * @param {function} onError
     * @param {array} files - files needed by the loaders[dae] in case of a collada
     * mesh, [obj, mtl] in case of object mesh, all as strings
     */
    loadMeshFromString(uri, submesh, centerSubmesh, onLoad, onError, files) {
        var uriPath = uri.substring(0, uri.lastIndexOf("/"));
        var uriFile = uri.substring(uri.lastIndexOf("/") + 1);
        if (this.meshes.has(uri)) {
            let mesh = this.meshes.get(uri).clone();
            if (submesh && this.useSubMesh(mesh, submesh, centerSubmesh)) {
                onLoad(mesh);
            }
            else if (!submesh) {
                onLoad(mesh);
            }
            return;
        }
        // load mesh
        if (uriFile.substr(-4).toLowerCase() === ".dae") {
            // loadCollada just accepts one file, which is the dae file as string
            if (files.length < 1 || !files[0]) {
                console.error("Missing DAE file");
                return;
            }
            this.loadCollada(uri, submesh, centerSubmesh, onLoad, onError, files[0]);
        }
        else if (uriFile.substr(-4).toLowerCase() === ".obj") {
            if (files.length < 2 || !files[0] || !files[1]) {
                console.error("Missing either OBJ or MTL file");
                return;
            }
            this.loadOBJ(uri, submesh, centerSubmesh, onLoad, onError, files);
        }
    }
    /**
     * Load collada file
     * @param {string} uri - mesh uri which is used by colldaloader to load
     * the mesh file using an XMLHttpRequest.
     * @param {} submesh
     * @param {} centerSubmesh
     * @param {function} onLoad - Callback when the mesh is loaded.
     * @param {function} onError - Callback when an error occurs.
     * @param {string} filestring -optional- the mesh file as a string to be
     * parsed
     * if provided the uri will not be used just as a url, no XMLHttpRequest will
     * be made.
     */
    loadCollada(uri, submesh, centerSubmesh, onLoad, onError, filestring) {
        let dae;
        var mesh = null;
        var that = this;
        /*
        // Crashes: issue #36
        if (this.meshes.has(uri))
        {
          dae = this.meshes.get(uri);
          dae = dae.clone();
          this.useColladaSubMesh(dae, submesh, centerSubmesh);
          onLoad(dae);
          return;
        }
        */
        function meshReady(collada) {
            // check for a scale factor
            /*if(collada.dae.asset.unit)
            {
              var scale = collada.dae.asset.unit;
              collada.scene.scale = new THREE.Vector3(scale, scale, scale);
            }*/
            dae = collada.scene;
            dae.updateMatrix();
            that.prepareColladaMesh(dae);
            that.meshes.set(uri, dae);
            dae = dae.clone();
            dae.name = uri;
            if (submesh && that.useSubMesh(dae, submesh, centerSubmesh)) {
                onLoad(dae);
            }
            else if (!submesh) {
                onLoad(dae);
            }
        }
        if (!filestring) {
            this.colladaLoader.load(uri, 
            // onLoad callback
            function (collada) {
                meshReady(collada);
            }, 
            // onProgress callback
            function (progress) { }, 
            // onError callback
            (error) => {
                if (this.findResourceCb) {
                    // Get the mesh from the websocket server.
                    this.findResourceCb(uri, (mesh, error) => {
                        if (error !== undefined) {
                            // Mark the mesh as error in the loading manager.
                            const manager = this.colladaLoader.manager;
                            manager.markAsError(uri);
                            return;
                        }
                        meshReady(this.colladaLoader.parse(new TextDecoder().decode(mesh), uri));
                        // Mark the mesh as done in the loading manager.
                        const manager = this.colladaLoader.manager;
                        manager.markAsDone(uri);
                    });
                }
            });
        }
        else {
            meshReady(this.colladaLoader.parse(filestring, undefined));
        }
    }
    /**
     * Prepare collada by removing other non-mesh entities such as lights
     * @param {} dae
     */
    prepareColladaMesh(dae) {
        let allChildren = [];
        getDescendants(dae, allChildren);
        for (let i = 0; i < allChildren.length; ++i) {
            if (allChildren[1] &&
                allChildren[i] instanceof THREE.Light &&
                allChildren[i].parent) {
                allChildren[i].parent.remove(allChildren[i]);
            }
        }
    }
    /**
     * Prepare mesh by handling submesh-only loading
     * @param {THREE.Mesh} mesh
     * @param {} submesh
     * @param {} centerSubmesh
     * @returns {THREE.Mesh} mesh
     */
    useSubMesh(mesh, submesh, centerSubmesh) {
        if (!submesh) {
            return null;
        }
        let result;
        // The mesh has children for every submesh. Those children are either
        // meshes or groups that contain meshes. We need to modify the mesh, so
        // only the required submesh is contained in it. Note: If a submesh is
        // contained in a group, we need to preserve that group, as it may apply
        // matrix transformations required by the submesh.
        // Auxiliary function used to look for the required submesh.
        // Checks if the given submesh is the one we look for. If it's a Group, look for it within its children.
        // It returns the submesh, if found.
        function lookForSubmesh(obj, parent) {
            if (obj instanceof THREE.Mesh) {
                // Check if the mesh has the correct name and has geometry.
                if (obj.name === submesh && obj.hasOwnProperty("geometry")) {
                    // Center the submesh.
                    if (centerSubmesh) {
                        // obj file
                        if (obj.geometry instanceof THREE.BufferGeometry) {
                            let geomPosition = obj.geometry.getAttribute("position");
                            let minPos = new THREE.Vector3();
                            let maxPos = new THREE.Vector3();
                            let centerPos = new THREE.Vector3();
                            minPos.fromBufferAttribute(geomPosition, 0);
                            maxPos.fromBufferAttribute(geomPosition, 0);
                            // Get the min and max values.
                            for (let i = 0; i < geomPosition.count; i++) {
                                minPos.x = Math.min(minPos.x, geomPosition.getX(i));
                                minPos.y = Math.min(minPos.y, geomPosition.getY(i));
                                minPos.z = Math.min(minPos.z, geomPosition.getZ(i));
                                maxPos.x = Math.max(maxPos.x, geomPosition.getX(i));
                                maxPos.y = Math.max(maxPos.y, geomPosition.getY(i));
                                maxPos.z = Math.max(maxPos.z, geomPosition.getZ(i));
                            }
                            // Compute center position
                            centerPos = minPos.add(maxPos.sub(minPos).multiplyScalar(0.5));
                            // Update geometry position
                            for (let i = 0; i < geomPosition.count; i++) {
                                let origPos = new THREE.Vector3();
                                origPos.fromBufferAttribute(geomPosition, i);
                                let newPos = origPos.sub(centerPos);
                                geomPosition.setXYZ(i, newPos.x, newPos.y, newPos.z);
                            }
                            geomPosition.needsUpdate = true;
                            // Center the position.
                            obj.position.set(0, 0, 0);
                            var childParent = obj.parent;
                            while (childParent) {
                                childParent.position.set(0, 0, 0);
                                childParent = childParent.parent;
                            }
                        }
                    }
                    // Filter the children of the parent. Only the required submesh
                    // needs to be there.
                    parent.children = [obj];
                    return [true, obj];
                }
            }
            else {
                for (let i = 0; i < obj.children.length; i++) {
                    if (obj.children[i] instanceof THREE.Mesh ||
                        obj.children[i] instanceof THREE.Group) {
                        const [found, result] = lookForSubmesh(obj.children[i], obj);
                        if (found) {
                            // This keeps the Group (obj), and modifies it's children to
                            // contain only the submesh.
                            obj.children = [result];
                            return [true, obj];
                        }
                    }
                }
            }
            return [false, obj];
        }
        // Look for the submesh in the children of the mesh.
        for (var i = 0; i < mesh.children.length; i++) {
            if (mesh.children[i] instanceof THREE.Mesh ||
                mesh.children[i] instanceof THREE.Group) {
                const [found, result] = lookForSubmesh(mesh.children[i], mesh);
                if (found) {
                    return result;
                }
            }
        }
        return null;
    }
    /**
     * Load obj file.
     * Loads obj mesh given using it's uri
     * @param {string} uri
     * @param {} submesh
     * @param {} centerSubmesh
     * @param {function} onLoad
     * @param {function} onError
     */
    loadOBJ(uri, submesh, centerSubmesh, onLoad, onError, files) {
        let objLoader = new GzObjLoader(this, uri, submesh, centerSubmesh, this.findResourceCb, onLoad, onError, files);
        objLoader.load();
    }
    /**
     * Load stl file.
     * Loads stl mesh given using it's uri
     * @param {string} uri
     * @param {} submesh
     * @param {} centerSubmesh
     * @param {function} onLoad
     */
    loadSTL(uri, submesh, centerSubmesh, onLoad, onError) {
        var mesh = null;
        var that = this;
        this.stlLoader.load(uri, 
        // onLoad
        function (geometry) {
            mesh = new THREE.Mesh(geometry);
            mesh.castShadow = true;
            mesh.receiveShadow = true;
            that.meshes.set(uri, mesh);
            mesh = mesh.clone();
            mesh.name = uri;
            if (submesh && that.useSubMesh(mesh, submesh, centerSubmesh)) {
                onLoad(mesh);
            }
            else if (!submesh) {
                onLoad(mesh);
            }
        }, 
        // onProgress
        function (progress) { }, 
        // onError
        function (error) {
            if (that.findResourceCb) {
                // Get the mesh from the websocket server.
                that.findResourceCb(uri, (rawmesh, error) => {
                    if (error !== undefined) {
                        // Mark the mesh as error in the loading manager.
                        const manager = that.stlLoader.manager;
                        manager.markAsError(uri);
                        return;
                    }
                    let decoded = that.stlLoader.parse(rawmesh);
                    decoded.name = uri;
                    onLoad(decoded);
                    // Mark the mesh as done in the loading manager.
                    const manager = that.stlLoader.manager;
                    manager.markAsDone(uri);
                });
            }
        });
    }
    /**
     * Load GLTF/GLB file
     * @param {string} uri
     * @param {} submesh
     * @param {} centerSubmesh
     * @param {function} onLoad - Callback when the mesh is loaded.
     * @param {function} onError - Callback when an error occurs.
     */
    loadGLTF(uri, submesh, centerSubmesh, onLoad, onError) {
        let that = this;
        this.gltfLoader.load(uri, 
        // onLoad callback
        function (gltf) {
            let mesh = gltf.scene;
            mesh.name = uri;
            if (submesh && that.useSubMesh(mesh, submesh, centerSubmesh)) {
                onLoad(mesh);
            }
            else if (!submesh) {
                onLoad(mesh);
            }
        }, 
        // onProgress callback
        function (progress) { }, 
        // onError callback
        (error) => {
            if (this.findResourceCb) {
                // Get the mesh from the websocket server.
                this.findResourceCb(uri, (data, error) => {
                    if (error !== undefined) {
                        // Mark the mesh as error in the loading manager.
                        const manager = this.gltfLoader.manager;
                        manager.markAsError(uri);
                        return;
                    }
                    // Parse the GLTF data
                    this.gltfLoader.parse(data, uri, (gltf) => {
                        let mesh = gltf.scene;
                        mesh.name = uri;
                        if (submesh && that.useSubMesh(mesh, submesh, centerSubmesh)) {
                            onLoad(mesh);
                        }
                        else if (!submesh) {
                            onLoad(mesh);
                        }
                        // Mark the mesh as done in the loading manager.
                        const manager = this.gltfLoader.manager;
                        manager.markAsDone(uri);
                    }, (error) => {
                        console.error("Error parsing GLTF from websocket", error);
                        const manager = this.gltfLoader.manager;
                        manager.markAsError(uri);
                    });
                });
            }
        });
    }
    /**
     * Set material for an object
     * @param {} obj
     * @param {} material
     */
    setMaterial(obj, material) {
        var scope = this;
        if (obj) {
            if (material) {
                // If the material has a PBR tag, use a MeshStandardMaterial,
                // which can have albedo, normal, emissive, roughness and metalness
                // maps. Otherwise use a Phong material.
                if (material.pbr) {
                    obj.material = new THREE.MeshStandardMaterial();
                    // Array of maps in order to facilitate the repetition and scaling process.
                    var maps = [];
                    if (material.pbr.albedoMap) {
                        let albedoMap = this.loadTexture(material.pbr.albedoMap);
                        obj.material.map = albedoMap;
                        maps.push(albedoMap);
                        // enable alpha test for textures with alpha transparency
                        if (albedoMap.format === THREE.RGBAFormat) {
                            obj.material.alphaTest = 0.5;
                        }
                    }
                    if (material.pbr.normalMap) {
                        let normalMap = this.loadTexture(material.pbr.normalMap);
                        obj.material.normalMap = normalMap;
                        maps.push(normalMap);
                    }
                    if (material.pbr.emissiveMap) {
                        let emissiveMap = this.loadTexture(material.pbr.emissiveMap);
                        obj.material.emissiveMap = emissiveMap;
                        maps.push(emissiveMap);
                    }
                    if (material.pbr.roughnessMap) {
                        let roughnessMap = this.loadTexture(material.pbr.roughnessMap);
                        obj.material.roughnessMap = roughnessMap;
                        maps.push(roughnessMap);
                    }
                    if (material.pbr.metalnessMap) {
                        let metalnessMap = this.loadTexture(material.pbr.metalnessMap);
                        obj.material.metalnessMap = metalnessMap;
                        maps.push(metalnessMap);
                    }
                    maps.forEach(function (map) {
                        map.wrapS = map.wrapT = THREE.RepeatWrapping;
                        map.repeat.x = 1.0;
                        map.repeat.y = 1.0;
                        if (material.scale) {
                            map.repeat.x = 1.0 / material.scale[0];
                            map.repeat.y = 1.0 / material.scale[1];
                        }
                    });
                }
                else {
                    obj.material = new THREE.MeshPhongMaterial();
                    const specular = material.specular;
                    if (specular) {
                        obj.material.specular.copy(specular);
                    }
                    if (material.texture) {
                        let texture = this.loadTexture(material.texture);
                        texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
                        texture.repeat.x = 1.0;
                        texture.repeat.y = 1.0;
                        if (material.scale) {
                            texture.repeat.x = 1.0 / material.scale[0];
                            texture.repeat.y = 1.0 / material.scale[1];
                        }
                        obj.material.map = texture;
                        // enable alpha test for textures with alpha transparency
                        if (texture.format === THREE.RGBAFormat) {
                            obj.material.alphaTest = 0.5;
                        }
                    }
                    if (material.normalMap) {
                        obj.material.normalMap = this.loadTexture(material.normalMap);
                    }
                }
                var ambient = material.ambient;
                var diffuse = material.diffuse;
                if (diffuse) {
                    // threejs removed ambient from phong and lambert materials so
                    // aproximate the resulting color by mixing ambient and diffuse
                    var dc = [];
                    dc[0] = diffuse.r;
                    dc[1] = diffuse.g;
                    dc[2] = diffuse.b;
                    if (ambient) {
                        var a = 0.4;
                        var d = 0.6;
                        dc[0] = ambient.r * a + diffuse.r * d;
                        dc[1] = ambient.g * a + diffuse.g * d;
                        dc[2] = ambient.b * a + diffuse.b * d;
                    }
                    obj.material.color.setRGB(dc[0], dc[1], dc[2]);
                }
                var opacity = material.opacity;
                if (opacity) {
                    if (opacity < 1) {
                        obj.material.transparent = true;
                        obj.material.opacity = opacity;
                    }
                }
            }
        }
    }
    /**
     * Set manipulation mode (view/translate/rotate)
     * @param {string} mode
     */
    setManipulationMode(mode) {
        this.manipulationMode = mode;
        if (mode === "view") {
            /*if (this.modelManipulator.object)
            {
              this.emitter.emit('entityChanged', this.modelManipulator.object);
            }*/
            this.selectEntity(null);
        }
        else {
            // Toggle manipulaion space (world / local)
            /*if (this.modelManipulator.mode === this.manipulationMode)
            {
              this.modelManipulator.space =
                (this.modelManipulator.space === 'world') ? 'local' : 'world';
            }
            this.modelManipulator.mode = this.manipulationMode;
            this.modelManipulator.setMode(this.modelManipulator.mode);
           */
            // model was selected during view mode
            if (this.selectedEntity) {
                this.selectEntity(this.selectedEntity);
            }
        }
    }
    /**
     * Show collision visuals
     * @param {boolean} show
     */
    showCollision(show) {
        if (show === this.showCollisions) {
            return;
        }
        let allObjects = [];
        getDescendants(this.scene, allObjects);
        for (let i = 0; i < allObjects.length; ++i) {
            if (allObjects[i] instanceof THREE.Object3D &&
                allObjects[i].name.indexOf("COLLISION_VISUAL") >= 0) {
                let allChildren = [];
                getDescendants(allObjects[i], allChildren);
                for (var j = 0; j < allChildren.length; ++j) {
                    if (allChildren[j] instanceof THREE.Mesh) {
                        allChildren[j].visible = show;
                    }
                }
            }
        }
        this.showCollisions = show;
    }
    /**
     * Attach manipulator to an object
     * @param {THREE.Object3D} model
     * @param {string} mode (translate/rotate)
     */
    attachManipulator(model, mode) {
        /*if (this.modelManipulator.object)
        {
          this.emitter.emit('entityChanged', this.modelManipulator.object);
        }
    
        if (mode !== 'view')
        {
          this.modelManipulator.attach(model);
          this.modelManipulator.mode = mode;
          this.modelManipulator.setMode( this.modelManipulator.mode );
          this.scene.add(this.modelManipulator.gizmo);
        }*/
    }
    /**
     * Toggle light visibility for the given entity. This will turn on/off
     * all lights that are children of the provided entity.
     * @param {string} Name of a THREE.Object3D.
     */
    toggleLights(entityName) {
        // Turn off following if `entity` is null.
        if (entityName === undefined || entityName === null) {
            return;
        }
        /* Helper function to enable all child lights */
        function enableLightsHelper(obj) {
            if (obj === null || obj === undefined) {
                return;
            }
            if (obj.userData.hasOwnProperty("type") &&
                obj.userData.type === "light") {
                obj.visible = !obj.visible;
            }
        }
        // Find the object and set the lights.
        var object = this.scene.getObjectByName(entityName);
        if (object !== null && object !== undefined) {
            object.traverse(enableLightsHelper);
        }
    }
    /**
     * Reset view
     */
    resetView() {
        this.camera.position.copy(this.defaultCameraPosition);
        this.camera.up = new THREE.Vector3(0, 0, 1);
        this.camera.lookAt(this.defaultCameraLookAt);
        this.camera.updateMatrix();
    }
    /**
     * Take a screenshot of the canvas and save it.
     *
     * @param {string} filename - The filename of the screenshot. PNG extension is appended to it.
     */
    saveScreenshot(filename) {
        // An explicit call to render is required. Otherwise the obtained image will be black.
        // See https://threejsfundamentals.org/threejs/lessons/threejs-tips.html, "Taking A Screenshot of the Canvas"
        this.render(0);
        this.getDomElement().toBlob(function (blob) {
            let url = URL.createObjectURL(blob);
            let linkElement = document.createElement("a");
            linkElement.href = url;
            linkElement.download = filename + ".png";
            document.body.appendChild(linkElement);
            linkElement.dispatchEvent(new MouseEvent("click"));
            document.body.removeChild(linkElement);
            URL.revokeObjectURL(url);
        });
    }
    /**
     * Generate thumbnails of the scene.
     *
     * The models on the scene should be previously scaled so that their maximum dimension equals 1.
     *
     * @param {string} filename - The filename of the generated zip file.
     * @param {THREE.Vector3} center - The point where the camera will point to.
     */
    createThumbnails(filename, center) {
        // Auxiliary method to return the canvas as a Promise.
        // This allows us to download all the images when they are ready.
        function getCanvasBlob(canvas) {
            return new Promise((resolve, reject) => {
                canvas.toBlob(function (blob) {
                    resolve(blob);
                });
            });
        }
        let zip = new JSZip();
        const canvas = this.getDomElement();
        const promises = [];
        // Directional light and target.
        const lightTarget = new THREE.Object3D();
        lightTarget.name = "thumbnails_light_target";
        lightTarget.position.copy(center);
        this.scene.add(lightTarget);
        const light = new THREE.DirectionalLight(0xffffff, 1.0);
        light.name = "thumbnails_light";
        this.scene.add(light);
        light.target = lightTarget;
        // Note: An explicit call to render is required for each image. Otherwise the obtained image will be black.
        // See https://threejsfundamentals.org/threejs/lessons/threejs-tips.html, "Taking A Screenshot of the Canvas"
        // Perspective
        this.camera.position.copy(center);
        this.camera.position.add(new THREE.Vector3(1.6, -1.6, 1.2));
        this.camera.lookAt(center);
        light.position.copy(this.camera.position);
        this.render(0);
        const perspective = getCanvasBlob(canvas);
        perspective.then(function (blob) {
            zip.file("thumbnails/1.png", blob);
        });
        promises.push(perspective);
        // Top
        this.camera.position.copy(center);
        this.camera.position.add(new THREE.Vector3(0, 0, 2.2));
        this.camera.rotation.copy(new THREE.Euler(0, 0, (-90 * Math.PI) / 180));
        light.position.copy(this.camera.position);
        this.render(0);
        const top = getCanvasBlob(canvas);
        top.then(function (blob) {
            zip.file("thumbnails/2.png", blob);
        });
        promises.push(top);
        // Front
        this.camera.position.copy(center);
        this.camera.position.add(new THREE.Vector3(2.2, 0, 0));
        this.camera.rotation.copy(new THREE.Euler(0, (90 * Math.PI) / 180, (90 * Math.PI) / 180));
        light.position.copy(this.camera.position);
        this.render(0);
        const front = getCanvasBlob(canvas);
        front.then(function (blob) {
            zip.file("thumbnails/3.png", blob);
        });
        promises.push(front);
        // Side
        this.camera.position.copy(center);
        this.camera.position.add(new THREE.Vector3(0, 2.2, 0));
        this.camera.rotation.copy(new THREE.Euler((-90 * Math.PI) / 180, 0, (180 * Math.PI) / 180));
        light.position.copy(this.camera.position);
        this.render(0);
        const side = getCanvasBlob(canvas);
        side.then(function (blob) {
            zip.file("thumbnails/4.png", blob);
        });
        promises.push(side);
        // Back
        this.camera.position.copy(center);
        this.camera.position.add(new THREE.Vector3(-2.2, 0, 0));
        this.camera.rotation.copy(new THREE.Euler((90 * Math.PI) / 180, (-90 * Math.PI) / 180, 0));
        light.position.copy(this.camera.position);
        light.position.add(new THREE.Vector3(-2000, 0, 0));
        this.render(0);
        const back = getCanvasBlob(canvas);
        back.then(function (blob) {
            zip.file("thumbnails/5.png", blob);
        });
        promises.push(back);
        Promise.all(promises).then(() => {
            zip.generateAsync({ type: "blob" }).then(function (content) {
                const url = URL.createObjectURL(content);
                const linkElement = document.createElement("a");
                linkElement.href = url;
                linkElement.download = filename + ".zip";
                document.body.appendChild(linkElement);
                linkElement.dispatchEvent(new MouseEvent("click"));
                document.body.removeChild(linkElement);
                URL.revokeObjectURL(url);
            });
            this.scene.remove(light);
            this.scene.remove(lightTarget);
        });
    }
    /**
     * Show radial menu
     * @param {} event
     */
    showRadialMenu(e) {
        /*if (!this.radialMenu)
        {
          return;
        }
    
        var event = e.originalEvent;
    
        var pointer = event.touches ? event.touches[ 0 ] : event;
        var pos = new THREE.Vector2(pointer.clientX, pointer.clientY);
    
        var intersect = new THREE.Vector3();
        var model = this.getRayCastModel(pos, intersect);
    
        if (model && model.name !== '' && model.name !== 'plane'
            && this.modelManipulator.pickerNames.indexOf(model.name) === -1)
        {
          this.radialMenu.show(event,model);
          this.selectEntity(model);
        }*/
    }
    /**
     * Sets the bounding box of an object while ignoring the addtional visuals.
     * @param {THREE.Box3} - box
     * @param {THREE.Object3D} - object
     */
    setFromObject(box, object) {
        box.min.x = box.min.y = box.min.z = +Infinity;
        box.max.x = box.max.y = box.max.z = -Infinity;
        var v = new THREE.Vector3();
        object.updateMatrixWorld(true);
        object.traverse(function (node) {
            let i, l;
            if (node instanceof THREE.Mesh) {
                let geometry = node.geometry;
                if (node.name !== "INERTIA_VISUAL" && node.name !== "COM_VISUAL") {
                    if (geometry.isBufferGeometry) {
                        let attribute = geometry.getAttribute("position");
                        if (attribute !== undefined) {
                            for (i = 0, l = attribute.count; i < l; i++) {
                                v.fromBufferAttribute(attribute, i).applyMatrix4(node.matrixWorld);
                                expandByPoint(v);
                            }
                        }
                    }
                    else {
                        console.error("Unable to setFromObject");
                    }
                }
            }
        });
        function expandByPoint(point) {
            box.min.min(point);
            box.max.max(point);
        }
    }
    /**
     * Show bounding box for a model. The box is aligned with the world.
     * @param {THREE.Object3D} model
     */
    showBoundingBox(model) {
        if (typeof model === "string") {
            model = this.scene.getObjectByName(model);
        }
        if (this.boundingBox.visible) {
            if (this.boundingBox.parent === model) {
                return;
            }
            else {
                this.hideBoundingBox();
            }
        }
        var box = new THREE.Box3();
        // w.r.t. world
        this.setFromObject(box, model);
        // center vertices with object
        box.min.x = box.min.x - model.position.x;
        box.min.y = box.min.y - model.position.y;
        box.min.z = box.min.z - model.position.z;
        box.max.x = box.max.x - model.position.x;
        box.max.y = box.max.y - model.position.y;
        box.max.z = box.max.z - model.position.z;
        let position = this.boundingBox.geometry.getAttribute("position");
        //var array = position.array;
        position.setXYZ(0, box.max.x, box.max.y, box.max.z);
        position.setXYZ(1, box.min.x, box.max.y, box.max.z);
        position.setXYZ(2, box.min.x, box.min.y, box.max.z);
        position.setXYZ(3, box.max.x, box.min.y, box.max.z);
        position.setXYZ(4, box.max.x, box.max.y, box.min.z);
        position.setXYZ(5, box.min.x, box.max.y, box.min.z);
        position.setXYZ(6, box.min.x, box.min.y, box.min.z);
        position.setXYZ(7, box.max.x, box.min.y, box.min.z);
        position.needsUpdate = true;
        this.boundingBox.geometry.computeBoundingSphere();
        // rotate the box back to the world
        var modelRotation = new THREE.Matrix4();
        modelRotation.extractRotation(model.matrixWorld);
        var modelInverse = new THREE.Matrix4();
        modelInverse.getInverse(modelRotation);
        this.boundingBox.quaternion.setFromRotationMatrix(modelInverse);
        this.boundingBox.name = "boundingBox";
        this.boundingBox.visible = true;
        // Add box as model's child
        model.add(this.boundingBox);
    }
    /**
     * Hide bounding box
     */
    hideBoundingBox() {
        if (this.boundingBox.parent) {
            this.boundingBox.parent.remove(this.boundingBox);
        }
        this.boundingBox.visible = false;
    }
    /**
     * Mouse right click
     * @param {} event
     * @param {} callback - function to be executed to the clicked model
     */
    onRightClick(event, callback) {
        var pos = new THREE.Vector2(event.clientX, event.clientY);
        var model = this.getRayCastModel(pos, new THREE.Vector3());
        if (model &&
            model.name !== "" &&
            model.name !== "plane" /* &&
          this.modelManipulator.pickerNames.indexOf(model.name) === -1*/) {
            callback(model);
        }
    }
    /**
     * Set model's view mode
     * @param {} model
     * @param {} viewAs (normal/transparent/wireframe)
     */
    setViewAs(model, viewAs) {
        // Toggle
        if (model.userData.viewAs === viewAs) {
            viewAs = "normal";
        }
        var showWireframe = viewAs === "wireframe";
        function materialViewAs(material) {
            if (materials.indexOf(material.id) === -1) {
                materials.push(material.id);
                if (viewAs === "transparent") {
                    if (material.opacity) {
                        material.originalOpacity = material.opacity;
                    }
                    else {
                        material.originalOpacity = 1.0;
                    }
                    material.opacity = 0.25;
                    material.transparent = true;
                }
                else {
                    material.opacity = material.originalOpacity
                        ? material.originalOpacity
                        : 1.0;
                    if (material.opacity >= 1.0) {
                        material.transparent = false;
                    }
                }
                // wireframe handling
                material.wireframe = showWireframe;
            }
        }
        let wireframe;
        let descendants = [];
        let materials = [];
        getDescendants(model, descendants);
        for (var i = 0; i < descendants.length; ++i) {
            if (descendants[i].material &&
                descendants[i].name.indexOf("boundingBox") === -1 &&
                descendants[i].name.indexOf("COLLISION_VISUAL") === -1 &&
                !this.getParentByPartialName(descendants[i], "COLLISION_VISUAL") &&
                descendants[i].name.indexOf("wireframe") === -1 &&
                descendants[i].name.indexOf("JOINT_VISUAL") === -1 &&
                descendants[i].name.indexOf("COM_VISUAL") === -1 &&
                descendants[i].name.indexOf("INERTIA_VISUAL") === -1) {
                if (Array.isArray(descendants[i].material)) {
                    for (var k = 0; k < descendants[i].material.length; ++k) {
                        materialViewAs(descendants[i].material[k]);
                    }
                }
                else {
                    materialViewAs(descendants[i].material);
                }
            }
        }
        if (!model.userData) {
            model.userData = new ModelUserData();
        }
        model.userData.viewAs = viewAs;
    }
    /**
     * Returns the closest parent whose name contains the given string
     * @param {} object
     * @param {} name
     */
    getParentByPartialName(object, name) {
        var parent = object.parent;
        while (parent && parent !== this.scene) {
            if (parent.name.indexOf(name) !== -1) {
                return parent;
            }
            parent = parent.parent;
        }
        return null;
    }
    /**
     * Select entity
     * @param {} object
     */
    selectEntity(object) {
        if (object) {
            if (object !== this.selectedEntity) {
                this.showBoundingBox(object);
                this.selectedEntity = object;
            }
            this.attachManipulator(object, this.manipulationMode);
            this.emitter.emit("setTreeSelected", object.name);
        }
        else {
            /*if (this.modelManipulator.object)
            {
              this.modelManipulator.detach();
              this.scene.remove(this.modelManipulator.gizmo);
            }*/
            this.hideBoundingBox();
            this.selectedEntity = null;
            this.emitter.emit("setTreeDeselected");
        }
    }
    /**
     * View joints
     * Toggle: if there are joints, hide, otherwise, show.
     * @param {} model
     */
    viewJoints(model) {
        if (model.joint === undefined || model.joint.length === 0) {
            return;
        }
        var child;
        // Visuals already exist
        if (model.jointVisuals) {
            // Hide = remove from parent
            if (model.jointVisuals[0].parent !== undefined &&
                model.jointVisuals[0].parent !== null) {
                for (var v = 0; v < model.jointVisuals.length; ++v) {
                    model.jointVisuals[v].parent.remove(model.jointVisuals[v]);
                }
            }
            // Show: attach to parent
            else {
                for (var s = 0; s < model.joint.length; ++s) {
                    child = model.getObjectByName(model.joint[s].child);
                    if (!child) {
                        continue;
                    }
                    child.add(model.jointVisuals[s]);
                }
            }
        }
        // Create visuals
        else {
            model.jointVisuals = [];
            for (var j = 0; j < model.joint.length; ++j) {
                child = model.getObjectByName(model.joint[j].child);
                if (!child) {
                    continue;
                }
                // XYZ expressed w.r.t. child
                var jointVisual = this.jointAxis["XYZaxes"].clone();
                child.add(jointVisual);
                model.jointVisuals.push(jointVisual);
                jointVisual.scale.set(0.7, 0.7, 0.7);
                this.setPose(jointVisual, model.joint[j].pose.position, model.joint[j].pose.orientation);
                var mainAxis = null;
                if (model.joint[j].type !== JointTypes.BALL &&
                    model.joint[j].type !== JointTypes.FIXED) {
                    mainAxis = this.jointAxis["mainAxis"].clone();
                    jointVisual.add(mainAxis);
                }
                var secondAxis = null;
                if (model.joint[j].type === JointTypes.REVOLUTE2 ||
                    model.joint[j].type === JointTypes.UNIVERSAL) {
                    secondAxis = this.jointAxis["mainAxis"].clone();
                    jointVisual.add(secondAxis);
                }
                if (model.joint[j].type === JointTypes.REVOLUTE ||
                    model.joint[j].type === JointTypes.GEARBOX) {
                    mainAxis.add(this.jointAxis["rotAxis"].clone());
                }
                else if (model.joint[j].type === JointTypes.REVOLUTE2 ||
                    model.joint[j].type === JointTypes.UNIVERSAL) {
                    mainAxis.add(this.jointAxis["rotAxis"].clone());
                    secondAxis.add(this.jointAxis["rotAxis"].clone());
                }
                else if (model.joint[j].type === JointTypes.BALL) {
                    jointVisual.add(this.jointAxis["ballVisual"].clone());
                }
                else if (model.joint[j].type === JointTypes.PRISMATIC) {
                    mainAxis.add(this.jointAxis["transAxis"].clone());
                }
                else if (model.joint[j].type === JointTypes.SCREW) {
                    mainAxis.add(this.jointAxis["screwAxis"].clone());
                }
                var direction, tempMatrix, rotMatrix;
                if (mainAxis) {
                    // main axis expressed w.r.t. parent model or joint frame
                    if (!model.joint[j].axis1) {
                        console.error("no joint axis " + model.joint[j].type + "vs " + JointTypes.FIXED);
                    }
                    if (model.joint[j].axis1.use_parent_model_frame === undefined) {
                        model.joint[j].axis1.use_parent_model_frame = true;
                    }
                    direction = new THREE.Vector3(model.joint[j].axis1.xyz.x, model.joint[j].axis1.xyz.y, model.joint[j].axis1.xyz.z);
                    direction.normalize();
                    tempMatrix = new THREE.Matrix4();
                    if (model.joint[j].axis1.use_parent_model_frame) {
                        tempMatrix.extractRotation(jointVisual.matrix);
                        tempMatrix.getInverse(tempMatrix);
                        direction.applyMatrix4(tempMatrix);
                        tempMatrix.extractRotation(child.matrix);
                        tempMatrix.getInverse(tempMatrix);
                        direction.applyMatrix4(tempMatrix);
                    }
                    rotMatrix = new THREE.Matrix4();
                    rotMatrix.lookAt(direction, new THREE.Vector3(0, 0, 0), mainAxis.up);
                    mainAxis.quaternion.setFromRotationMatrix(rotMatrix);
                }
                if (secondAxis) {
                    if (model.joint[j].axis2.use_parent_model_frame === undefined) {
                        model.joint[j].axis2.use_parent_model_frame = true;
                    }
                    direction = new THREE.Vector3(model.joint[j].axis2.xyz.x, model.joint[j].axis2.xyz.y, model.joint[j].axis2.xyz.z);
                    direction.normalize();
                    tempMatrix = new THREE.Matrix4();
                    if (model.joint[j].axis2.use_parent_model_frame) {
                        tempMatrix.extractRotation(jointVisual.matrix);
                        tempMatrix.getInverse(tempMatrix);
                        direction.applyMatrix4(tempMatrix);
                        tempMatrix.extractRotation(child.matrix);
                        tempMatrix.getInverse(tempMatrix);
                        direction.applyMatrix4(tempMatrix);
                    }
                    secondAxis.position = direction.multiplyScalar(0.3);
                    rotMatrix = new THREE.Matrix4();
                    rotMatrix.lookAt(direction, new THREE.Vector3(0, 0, 0), secondAxis.up);
                    secondAxis.quaternion.setFromRotationMatrix(rotMatrix);
                }
            }
        }
    }
    /**
     * View Center Of Mass
     * Toggle: if there are COM visuals, hide, otherwise, show.
     * @param {} model
     */
    // This function needs to be migrated to ES6 and the latest THREE
    /*public viewCOM(model: any): void {
      if (model === undefined || model === null)
      {
        return;
      }
      if (model.children.length === 0)
      {
        return;
      }
  
      var child;
  
      // Visuals already exist
      if (model.COMVisuals)
      {
        // Hide = remove from parent
        if (model.COMVisuals[0].parent !== undefined &&
          model.COMVisuals[0].parent !== null)
        {
          for (var v = 0; v < model.COMVisuals.length; ++v)
          {
            for (var k = 0; k < 3; k++)
            {
              model.COMVisuals[v].parent.remove(model.COMVisuals[v].crossLines[k]);
            }
            model.COMVisuals[v].parent.remove(model.COMVisuals[v]);
          }
        }
        // Show: attach to parent
        else
        {
          for (var s = 0; s < model.children.length; ++s)
          {
            child = model.getObjectByName(model.children[s].name);
  
            if (!child || child.name === 'boundingBox')
            {
              continue;
            }
  
            child.add(model.COMVisuals[s].crossLines[0]);
            child.add(model.COMVisuals[s].crossLines[1]);
            child.add(model.COMVisuals[s].crossLines[2]);
            child.add(model.COMVisuals[s]);
          }
        }
      }
      // Create visuals
      else
      {
        model.COMVisuals = [];
        let COMVisual: THREE.Object3D;
        let helperGeometry_1: THREE.BufferGeometry;
        let helperGeometry_2: THREE.BufferGeometry;
        let helperGeometry_3: THREE.BufferGeometry;
  
        var box, line_1, line_2, line_3, helperMaterial, points = new Array(6);
        for (var j = 0; j < model.children.length; ++j)
        {
          child = model.getObjectByName(model.children[j].name);
  
          if (!child) {
            continue;
          }
  
          if (child.userData.inertial)
          {
            let inertialPose: Pose = new Pose();
            let userdatapose: Pose = new Pose();
            let inertialMass: number = 0;
            let radius: number = 0;
            var mesh = {};
            var inertial = child.userData.inertial;
  
            userdatapose = child.userData.inertial.pose;
            inertialMass = inertial.mass;
  
            // calculate the radius using lead density
            radius = Math.cbrt((0.75 * inertialMass ) / (Math.PI * 11340));
  
            COMVisual = this.COMvisual.clone();
            child.add(COMVisual);
            model.COMVisuals.push(COMVisual);
            COMVisual.scale.set(radius, radius, radius);
  
            var position = new THREE.Vector3(0, 0, 0);
  
            // get euler rotation and convert it to Quaternion
            var quaternion = new THREE.Quaternion();
            var euler = new THREE.Euler(0, 0, 0, 'XYZ');
            quaternion.setFromEuler(euler);
  
            inertialPose = {
              position: position,
              orientation: quaternion
            };
  
            if (userdatapose !== undefined) {
              this.setPose(COMVisual, userdatapose.position,
                userdatapose.orientation);
                inertialPose = userdatapose;
            }
  
            (COMVisual as any).crossLines = [];
  
            // Store link's original rotation (w.r.t. the model)
            var originalRotation = new THREE.Euler();
            originalRotation.copy(child.rotation);
  
            // Align link with world (reverse parent rotation w.r.t. the world)
            child.setRotationFromMatrix(
              new THREE.Matrix4().getInverse(child.parent.matrixWorld));
  
            // Get its bounding box
            box = new THREE.Box3();
  
            box.setFromObject(child);
  
            // Rotate link back to its original rotation
            child.setRotationFromEuler(originalRotation);
  
            // w.r.t child
            var worldToLocal = new THREE.Matrix4();
            worldToLocal.getInverse(child.matrixWorld);
            box.applyMatrix4(worldToLocal);
  
            // X
            points[0] = new THREE.Vector3(box.min.x, inertialPose.position.y,
              inertialPose.position.z);
            points[1] = new THREE.Vector3(box.max.x, inertialPose.position.y,
                inertialPose.position.z);
            // Y
            points[2] = new THREE.Vector3(inertialPose.position.x, box.min.y,
                  inertialPose.position.z);
            points[3] = new THREE.Vector3(inertialPose.position.x, box.max.y,
                    inertialPose.position.z);
            // Z
            points[4] = new THREE.Vector3(inertialPose.position.x,
              inertialPose.position.y, box.min.z);
            points[5] = new THREE.Vector3(inertialPose.position.x,
              inertialPose.position.y, box.max.z);
  
            helperGeometry_1 = new THREE.BufferGeometry();
            helperGeometry_1.vertices.push(points[0]);
            helperGeometry_1.vertices.push(points[1]);
  
            helperGeometry_2 = new THREE.BufferGeometry();
            helperGeometry_2.vertices.push(points[2]);
            helperGeometry_2.vertices.push(points[3]);
  
            helperGeometry_3 = new THREE.Geometry();
            helperGeometry_3.vertices.push(points[4]);
            helperGeometry_3.vertices.push(points[5]);
  
            helperMaterial = new THREE.LineBasicMaterial({color: 0x00ff00});
  
            line_1 = new THREE.Line(helperGeometry_1, helperMaterial,
                THREE.LineSegments);
            line_2 = new THREE.Line(helperGeometry_2, helperMaterial,
                THREE.LineSegments);
            line_3 = new THREE.Line(helperGeometry_3, helperMaterial,
                THREE.LineSegments);
  
            line_1.name = 'COM_VISUAL';
            line_2.name = 'COM_VISUAL';
            line_3.name = 'COM_VISUAL';
            COMVisual.crossLines.push(line_1);
            COMVisual.crossLines.push(line_2);
            COMVisual.crossLines.push(line_3);
  
            // show lines
            child.add(line_1);
            child.add(line_2);
            child.add(line_3);
           }
        }
      }
    }*/
    // TODO: Issue https://bitbucket.org/osrf/gzweb/issues/138
    /**
     * View inertia
     * Toggle: if there are inertia visuals, hide, otherwise, show.
     * @param {} model
     */
    // This function needs to be migrated to ES6 and the latest THREE
    /*public viewInertia(model: any): void {
      if (model === undefined || model === null)
      {
        return;
      }
  
      if (model.children.length === 0)
      {
        return;
      }
  
      var child;
  
      // Visuals already exist
      if (model.inertiaVisuals)
      {
        // Hide = remove from parent
        if (model.inertiaVisuals[0].parent !== undefined &&
          model.inertiaVisuals[0].parent !== null)
        {
          for (var v = 0; v < model.inertiaVisuals.length; ++v)
          {
            for (var k = 0; k < 3; k++)
            {
              model.inertiaVisuals[v].parent.remove(
                model.inertiaVisuals[v].crossLines[k]);
            }
            model.inertiaVisuals[v].parent.remove(model.inertiaVisuals[v]);
          }
        }
        // Show: attach to parent
        else
        {
          for (var s = 0; s < model.children.length; ++s)
          {
            child = model.getObjectByName(model.children[s].name);
  
            if (!child || child.name === 'boundingBox')
            {
              continue;
            }
            child.add(model.inertiaVisuals[s].crossLines[0]);
            child.add(model.inertiaVisuals[s].crossLines[1]);
            child.add(model.inertiaVisuals[s].crossLines[2]);
            child.add(model.inertiaVisuals[s]);
          }
        }
      }
      // Create visuals
      else
      {
        model.inertiaVisuals = [];
        var box , line_1, line_2, line_3, helperGeometry_1, helperGeometry_2,
        helperGeometry_3, helperMaterial, inertial, inertiabox,
        points = new Array(6);
        for (var j = 0; j < model.children.length; ++j)
        {
          child = model.getObjectByName(model.children[j].name);
  
          if (!child)
          {
            continue;
          }
  
          inertial = child.userData.inertial;
          if (inertial)
          {
            var mesh, boxScale, Ixx, Iyy, Izz, mass, inertia, material = {};
            let inertialPose: Pose;
  
            if (inertial.pose)
            {
              inertialPose = child.userData.inertial.pose;
            }
            else if (child.position)
            {
              inertialPose.position = child.position;
              inertialPose.orientation = child.quaternion;
            }
            else
            {
              console.warn('Link pose not found!');
              continue;
            }
  
            mass = inertial.mass;
            inertia = inertial.inertia;
            Ixx = inertia.ixx;
            Iyy = inertia.iyy;
            Izz = inertia.izz;
            boxScale = new THREE.Vector3();
  
            if (mass < 0 || Ixx < 0 || Iyy < 0 || Izz < 0 ||
              Ixx + Iyy < Izz || Iyy + Izz < Ixx || Izz + Ixx < Iyy)
            {
              // Unrealistic inertia, load with default scale
              console.warn('The link ' + child.name + ' has unrealistic inertia, '
                    +'unable to visualize box of equivalent inertia.');
            }
            else
            {
              // Compute dimensions of box with uniform density
              // and equivalent inertia.
              boxScale.x = Math.sqrt(6*(Izz +  Iyy - Ixx) / mass);
              boxScale.y = Math.sqrt(6*(Izz +  Ixx - Iyy) / mass);
              boxScale.z = Math.sqrt(6*(Ixx  + Iyy - Izz) / mass);
  
              inertiabox = new THREE.Object3D();
              inertiabox.name = 'INERTIA_VISUAL';
  
              // Inertia indicator: equivalent box of uniform density
              mesh = this.createBox(1, 1, 1);
              mesh.name = 'INERTIA_VISUAL';
              material = {'ambient':[1,0.0,1,1],'diffuse':[1,0.0,1,1],
                'depth_write':false,'opacity':0.5};
              this.setMaterial(mesh, material);
              inertiabox.add(mesh);
              inertiabox.name = 'INERTIA_VISUAL';
              child.add(inertiabox);
  
              model.inertiaVisuals.push(inertiabox);
              inertiabox.scale.set(boxScale.x, boxScale.y, boxScale.z);
              inertiabox.crossLines = [];
  
              this.setPose(inertiabox, inertialPose.position,
                inertialPose.orientation);
              // show lines
              box = new THREE.Box3();
              // w.r.t. world
              box.setFromObject(child);
              points[0] = new THREE.Vector3(inertialPose.position.x,
                inertialPose.position.y,
                -2 * boxScale.z + inertialPose.position.z);
              points[1] = new THREE.Vector3(inertialPose.position.x,
                inertialPose.position.y, 2 * boxScale.z + inertialPose.position.z);
              points[2] = new THREE.Vector3(inertialPose.position.x,
                -2 * boxScale.y + inertialPose.position.y ,
                inertialPose.position.z);
              points[3] = new THREE.Vector3(inertialPose.position.x,
                2 * boxScale.y + inertialPose.position.y, inertialPose.position.z);
              points[4] = new THREE.Vector3(
                -2 * boxScale.x + inertialPose.position.x,
                inertialPose.position.y, inertialPose.position.z);
              points[5] = new THREE.Vector3(
                2 * boxScale.x + inertialPose.position.x,
                inertialPose.position.y, inertialPose.position.z);
  
              helperGeometry_1 = new THREE.Geometry();
              helperGeometry_1.vertices.push(points[0]);
              helperGeometry_1.vertices.push(points[1]);
  
              helperGeometry_2 = new THREE.Geometry();
              helperGeometry_2.vertices.push(points[2]);
              helperGeometry_2.vertices.push(points[3]);
  
              helperGeometry_3 = new THREE.Geometry();
              helperGeometry_3.vertices.push(points[4]);
              helperGeometry_3.vertices.push(points[5]);
  
              helperMaterial = new THREE.LineBasicMaterial({color: 0x00ff00});
              line_1 = new THREE.Line(helperGeometry_1, helperMaterial,
                  THREE.LineSegments);
              line_2 = new THREE.Line(helperGeometry_2, helperMaterial,
                THREE.LineSegments);
              line_3 = new THREE.Line(helperGeometry_3, helperMaterial,
                THREE.LineSegments);
  
              line_1.name = 'INERTIA_VISUAL';
              line_2.name = 'INERTIA_VISUAL';
              line_3.name = 'INERTIA_VISUAL';
              inertiabox.crossLines.push(line_1);
              inertiabox.crossLines.push(line_2);
              inertiabox.crossLines.push(line_3);
  
              // attach lines
              child.add(line_1);
              child.add(line_2);
              child.add(line_3);
            }
          }
        }
      }
    }*/
    /**
     * Update a light entity from a message
     * @param {} entity
     * @param {} msg
     */
    // This function needs to be migrated to ES6 and the latest THREE
    /*public updateLight(entity: any, msg: any): void {
      // TODO: Generalize this and createLight
      var lightObj = entity.children[0];
      var dir;
  
      var color = new THREE.Color();
  
      if (msg.diffuse)
      {
        color.r = msg.diffuse.r;
        color.g = msg.diffuse.g;
        color.b = msg.diffuse.b;
        lightObj.color = color.clone();
      }
      if (msg.specular)
      {
        color.r = msg.specular.r;
        color.g = msg.specular.g;
        color.b = msg.specular.b;
      }
  
      var matrixWorld;
      if (msg.pose)
      {
        // needed to update light's direction
        this.setPose(entity, msg.pose.position, msg.pose.orientation);
        entity.matrixWorldNeedsUpdate = true;
      }
  
      if (msg.range)
      {
        // THREE.js's light distance impacts the attenuation factor defined in the
        // shader:
        // attenuation factor = 1.0 - distance-to-enlighted-point / light.distance
        // Gazebo's range (taken from OGRE 3D API) does not contribute to
        // attenuation; it is a hard limit for light scope.
        // Nevertheless, we identify them for sake of simplicity.
        lightObj.distance = msg.range;
      }
  
      if (msg.cast_shadows)
      {
        lightObj.castShadow = msg.cast_shadows;
      }
  
      if (msg.attenuation_constant)
      {
        // no-op
      }
      if (msg.attenuation_linear)
      {
        lightObj.intensity = lightObj.intensity/(1+msg.attenuation_linear);
      }
      if (msg.attenuation_quadratic)
      {
        lightObj.intensity = lightObj.intensity/(1+msg.attenuation_quadratic);
      }
  
    //  Not handling these on gzweb for now
    //
    //  if (lightObj instanceof THREE.SpotLight) {
    //    if (msg.spot_outer_angle) {
    //      lightObj.angle = msg.spot_outer_angle;
    //    }
    //    if (msg.spot_falloff) {
    //      lightObj.exponent = msg.spot_falloff;
    //    }
    //  }
  
      if (msg.direction)
      {
        dir = new THREE.Vector3(msg.direction.x, msg.direction.y,
            msg.direction.z);
  
        entity.direction = new THREE.Vector3();
        entity.direction.copy(dir);
  
        if (lightObj.target)
        {
          lightObj.target.position.copy(dir);
        }
      }
    }*/
    /**
     * Adds an sdf model to the scene.
     * @param {object} sdf - It is either SDF XML string or SDF XML DOM object
     * @returns {THREE.Object3D}
     */
    // This function needs to be migrated to ES6 and the latest THREE
    /*public createFromSdf(sdf: any): THREE.Object3D {
      if (sdf === undefined)
      {
        console.error(' No argument provided ');
        return;
      }
  
      var obj = new THREE.Object3D();
  
      var sdfXml = this.spawnModel.sdfParser.parseXML(sdf);
      // sdfXML is always undefined, the XML parser doesn't work while testing
      // while it does work during normal usage.
      var myjson = xmlParser.xml2json(sdfXml, '\t');
      var sdfObj = JSON.parse(myjson).sdf;
  
      var mesh = this.spawnModel.sdfParser.spawnFromSDF(sdf);
      if (!mesh)
      {
        return;
      }
  
      obj.name = mesh.name;
      obj.add(mesh);
  
      return obj;
    }*/
    /**
     * Adds a lighting setup that is great for single model visualization. This
     * will not alter existing lights.
     */
    addModelLighting() {
        this.ambient.color = new THREE.Color(0x666666);
        // And light1. Upper back fill light.
        var light1 = this.createLight(3, 
        // Diffuse
        new Color(0.2, 0.2, 0.2, 1.0), 
        // Intensity
        0.5, 
        // Pose
        new Pose(new THREE.Vector3(0, 10, 10), new THREE.Quaternion(0, 0, 0, 1)), 
        // Distance
        undefined, 
        // Cast shadows
        true, 
        // Name
        "__model_light1__", 
        // Direction
        new THREE.Vector3(0, -0.707, -0.707), 
        // Specular
        new Color(0.3, 0.3, 0.3, 1.0));
        this.add(light1);
        // And light2. Lower back fill light
        var light2 = this.createLight(3, 
        // Diffuse
        new Color(0.4, 0.4, 0.4, 1.0), 
        // Intensity
        0.5, 
        // Pose
        new Pose(new THREE.Vector3(0, 10, -10), new THREE.Quaternion(0, 0, 0, -1)), 
        // Distance
        undefined, 
        // Cast shadows
        true, 
        // Name
        "__model_light2__", 
        // Direction
        new THREE.Vector3(0, -0.707, 0.707), 
        // Specular
        new Color(0.3, 0.3, 0.3, 1.0));
        this.add(light2);
        // And light3. Front fill light.
        var light3 = this.createLight(3, 
        // Diffuse
        new Color(0.5, 0.5, 0.5, 1.0), 
        // Intensity
        0.4, 
        // Pose
        new Pose(new THREE.Vector3(-10, -10, 10), new THREE.Quaternion(0, 0, 0, 1)), 
        // Distance
        undefined, 
        // Cast shadows
        true, 
        // Name
        "__model_light2__", 
        // Direction
        new THREE.Vector3(0.707, 0.707, 0), 
        // Specular
        new Color(0.3, 0.3, 0.3, 1.0));
        this.add(light3);
        // And light4. Front key light.
        var light4 = this.createLight(3, 
        // Diffuse
        new Color(1, 1, 1, 1.0), 
        // Intensity
        0.8, 
        // Pose
        new Pose(new THREE.Vector3(10, -10, 10), new THREE.Quaternion(0, 0, 0, 1)), 
        // Distance
        undefined, 
        // Cast shadows
        true, 
        // Name
        "__model_light2__", 
        // Direction
        new THREE.Vector3(-0.707, 0.707, 0), 
        // Specular
        new Color(0.8, 0.8, 0.8, 1.0));
        this.add(light4);
    }
    /**
     * Dispose all the resources used by the scene.
     *
     * This should be called whenever the visualization stops, in order to free resources.
     * See: https://threejs.org/docs/index.html#manual/en/introduction/How-to-dispose-of-objects
     */
    cleanup() {
        let objects = [];
        getDescendants(this.scene, objects);
        var that = this;
        objects.forEach(function (obj) {
            that.scene.remove(obj);
            // Dispose geometries.
            if (obj.geometry) {
                obj.geometry.dispose();
            }
            // Dispose materials and their textures.
            if (obj.material) {
                // Materials can be an array. If there is only one, convert it to an array for easier handling.
                if (!(obj.material instanceof Array)) {
                    obj.material = [obj.material];
                }
                // Materials can have different texture maps, depending on their type.
                // We check each property of the Material and dispose them if they are Textures.
                obj.material.forEach(function (material) {
                    Object.keys(material).forEach(function (property) {
                        if (material[property] instanceof THREE.Texture) {
                            material[property].dispose();
                        }
                    });
                    material.dispose();
                });
            }
        });
        // Destroy particles.
        if (this.nebulaSystem) {
            this.nebulaSystem.destroy();
        }
        // Clean scene and renderer.
        this.renderer.renderLists.dispose();
        this.renderer.dispose();
    }
    /**
     * Set a request header for internal requests.
     *
     * @param {string} header - The header to send in the request.
     * @param {string} value - The value to set to the header.
     */
    setRequestHeader(header, value) {
        // ES6 syntax for computed object keys.
        const headerObject = { [header]: value };
        this.textureLoader.requestHeader = headerObject;
        this.colladaLoader.requestHeader = headerObject;
        this.stlLoader.requestHeader = headerObject;
        this.requestHeader = headerObject;
        // Change the texture loader, if the requestHeader is present.
        // Texture Loaders use an Image Loader internally, instead of a File Loader.
        // Image Loader uses an img tag, and their src request doesn't accept
        // custom headers.
        // See https://github.com/mrdoob/three.js/issues/10439
        if (this.requestHeader) {
            this.textureLoader.load = function (url, onLoad, onProgress, onError) {
                var fileLoader = new THREE.FileLoader();
                fileLoader.setResponseType("blob");
                fileLoader.setRequestHeader(this.requestHeader);
                let texture = new THREE.Texture();
                let image = (document.createElementNS("http://www.w3.org/1999/xhtml", "img"));
                // Once the image is loaded, we need to revoke the ObjectURL.
                image.onload = function () {
                    image.onload = null;
                    URL.revokeObjectURL(image.src);
                    texture.image = image;
                    texture.needsUpdate = true;
                    if (onLoad) {
                        onLoad(texture);
                    }
                };
                image.onerror = onError;
                // Once the image is loaded, we need to revoke the ObjectURL.
                fileLoader.load(url, function (blob) {
                    image.src = URL.createObjectURL(blob);
                }, onProgress, onError);
                return texture;
            };
        }
    }
    /**
     * Get the Nebula System.
     *
     * The System is usually required by render loops in order to be updated.
     *
     * @returns The Nebula System, or undefined if it wasn't set.
     */
    getParticleSystem() {
        return this.nebulaSystem;
    }
    /**
     * Get the Nebula Renderer.
     *
     * Used by emitters to render particles.
     *
     * @returns The Nebula Renderer, or undefined if it wasn't set.
     */
    getParticleRenderer() {
        return this.nebulaRenderer;
    }
    /**
     * Set the Nebula System in order to use particles.
     *
     * @param system The Nebula System.
     * @param renderer The renderer the Nebula System will use.
     */
    setupParticleSystem(system, renderer) {
        this.nebulaSystem = system;
        this.nebulaRenderer = renderer;
    }
    /**
     * Print out the scene graph with position of each node.
     */
    printScene() {
        const printGraph = (obj) => {
            console.group(`<${obj.type}> ${obj.name} pos: ${obj.position.x}, ${obj.position.y}, ${obj.position.z}`);
            obj.children.forEach(printGraph);
            console.groupEnd();
        };
        printGraph(this.scene);
    }
    loadTexture(url, onLoad, onProgress) {
        // Return the cached texture if it exists.
        if (this.textureCache.has(url)) {
            return this.textureCache.get(url);
        }
        let fallbackLoader = (map, texture) => {
            if (this.findResourceCb) {
                // Get the image using the find resource callback.
                this.findResourceCb(map, (image, error) => {
                    if (error !== undefined) {
                        // Mark the texture as error in the loading manager.
                        const manager = this.textureLoader.manager;
                        manager.markAsError(map);
                        return;
                    }
                    // Create the image element
                    let imageElem = (document.createElementNS("http://www.w3.org/1999/xhtml", "img"));
                    const isJPEG = map.search(/\.jpe?g($|\?)/i) > 0 ||
                        map.search(/^data\:image\/jpeg/) === 0;
                    let binary = "";
                    const len = image.byteLength;
                    for (var i = 0; i < len; i++) {
                        binary += String.fromCharCode(image[i]);
                    }
                    // Set the image source using base64 encoding
                    imageElem.src = isJPEG
                        ? "data:image/jpg;base64,"
                        : "data:image/png;base64,";
                    imageElem.src += window.btoa(binary);
                    texture.format = isJPEG ? THREE.RGBFormat : THREE.RGBAFormat;
                    texture.needsUpdate = true;
                    texture.image = imageElem;
                    // Mark the texture as done in the loading manager.
                    const manager = this.textureLoader.manager;
                    manager.markAsDone(map);
                });
            }
        };
        let result = this.textureLoader.load(url, onLoad, onProgress, (_error) => {
            let scopeTexture = result;
            fallbackLoader(url, scopeTexture);
        });
        // Cache the texture so that we don't try to load it multiple times.
        this.textureCache.set(url, result);
        return result;
    }
}
