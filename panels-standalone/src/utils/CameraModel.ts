/**
 * Camera Model Implementation
 * Ported from Lichtblick packages/suite-base/src/panels/ThreeDeeRender
 * 
 * Implements pinhole camera model with distortion correction
 * for projecting 3D points to 2D image coordinates.
 */

/**
 * ROS2 sensor_msgs/CameraInfo message type
 */
export interface CameraInfo {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  height: number;
  width: number;
  distortion_model: string;
  K: number[]; // Length 9: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
  D: number[]; // Distortion coefficients [k1, k2, p1, p2, k3, ...]
  R: number[]; // Length 9: Rectification matrix (row-major)
  P: number[]; // Length 12: Projection matrix (row-major)
  binning_x: number;
  binning_y: number;
  roi: {
    x_offset: number;
    y_offset: number;
    height: number;
    width: number;
    do_rectify: boolean;
  };
}

/**
 * Interface for camera models
 */
export interface ICameraModel {
  width: number;
  height: number;
  fx: number;  // Focal length X (pixels)
  fy: number;  // Focal length Y (pixels)
  cx: number;  // Principal point X (pixels)
  cy: number;  // Principal point Y (pixels)
  distortionModel: string;
  D: number[]; // Distortion coefficients
  
  /**
   * Project a 3D point (in camera frame) to 2D pixel coordinates
   * @param point3D - 3D point in camera coordinate frame
   * @returns 2D pixel coordinates, or null if point is behind camera or outside image
   */
  project(point3D: { x: number; y: number; z: number }): { u: number; v: number } | null;
}

/**
 * Pinhole camera model with lens distortion
 * Implements the standard ROS camera model used in image_geometry
 */
export class PinholeCameraModel implements ICameraModel {
  public readonly width: number;
  public readonly height: number;
  public readonly fx: number;
  public readonly fy: number;
  public readonly cx: number;
  public readonly cy: number;
  public readonly distortionModel: string;
  public readonly D: number[];
  
  private readonly K: number[]; // 3x3 intrinsic matrix
  private readonly P: number[]; // 3x4 projection matrix
  
  constructor(cameraInfo: CameraInfo) {
    this.width = cameraInfo.width;
    this.height = cameraInfo.height;
    this.distortionModel = cameraInfo.distortion_model;
    this.D = cameraInfo.D;
    this.K = cameraInfo.K;
    this.P = cameraInfo.P;
    
    // Extract focal lengths and principal point from K matrix
    // K is stored row-major:
    // [fx,  0, cx,
    //   0, fy, cy,
    //   0,  0,  1]
    this.fx = this.K[0];
    this.fy = this.K[4];
    this.cx = this.K[2];
    this.cy = this.K[5];
  }
  
  /**
   * Project 3D point to 2D pixel coordinates
   * Implements pinhole camera model with distortion correction
   * 
   * Reference: ROS image_geometry PinholeCameraModel
   * http://docs.ros.org/en/api/image_geometry/html/c++/pinhole__camera__model_8cpp_source.html
   */
  public project(point3D: { x: number; y: number; z: number }): { u: number; v: number } | null {
    const { x: X, y: Y, z: Z } = point3D;
    
    // Point must be in front of camera
    if (Z <= 0) {
      return null;
    }
    
    // 1. Normalize by depth (perspective projection)
    let x = X / Z;
    let y = Y / Z;
    
    // 2. Apply lens distortion
    if (this.distortionModel === 'plumb_bob' && this.D.length >= 5) {
      const result = this.applyPlumbBobDistortion(x, y);
      x = result.x;
      y = result.y;
    } else if (this.distortionModel === 'rational_polynomial' && this.D.length >= 8) {
      const result = this.applyRationalPolynomialDistortion(x, y);
      x = result.x;
      y = result.y;
    }
    // For other models or empty D, use undistorted coordinates
    
    // 3. Apply intrinsic matrix to get pixel coordinates
    const u = this.fx * x + this.cx;
    const v = this.fy * y + this.cy;
    
    // 4. Check if point is within image bounds
    if (u < 0 || u >= this.width || v < 0 || v >= this.height) {
      return null; // Outside image
    }
    
    return { u, v };
  }
  
  /**
   * Apply plumb bob (Brown-Conrady) distortion model
   * This is the most common distortion model used in ROS
   * 
   * D = [k1, k2, p1, p2, k3, k4, k5, k6]
   * where:
   *   k1, k2, k3, k4, k5, k6 = radial distortion coefficients
   *   p1, p2 = tangential distortion coefficients
   */
  private applyPlumbBobDistortion(x: number, y: number): { x: number; y: number } {
    const [k1, k2, p1, p2, k3 = 0, k4 = 0, k5 = 0, k6 = 0] = this.D;
    
    const r2 = x * x + y * y;
    const r4 = r2 * r2;
    const r6 = r4 * r2;
    
    // Radial distortion factor
    let radialDistortion: number;
    if (k6 !== 0 || k5 !== 0 || k4 !== 0) {
      // Rational model: (1 + k1*r^2 + k2*r^4 + k3*r^6) / (1 + k4*r^2 + k5*r^4 + k6*r^6)
      radialDistortion = (1 + k1 * r2 + k2 * r4 + k3 * r6) / (1 + k4 * r2 + k5 * r4 + k6 * r6);
    } else {
      // Standard polynomial: 1 + k1*r^2 + k2*r^4 + k3*r^6
      radialDistortion = 1 + k1 * r2 + k2 * r4 + k3 * r6;
    }
    
    // Tangential distortion
    const tangentialX = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    const tangentialY = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
    
    // Apply distortion
    const xDistorted = x * radialDistortion + tangentialX;
    const yDistorted = y * radialDistortion + tangentialY;
    
    return { x: xDistorted, y: yDistorted };
  }
  
  /**
   * Apply rational polynomial distortion model
   * D = [k1, k2, p1, p2, k3, k4, k5, k6]
   */
  private applyRationalPolynomialDistortion(x: number, y: number): { x: number; y: number } {
    // Same as plumb bob for this model
    return this.applyPlumbBobDistortion(x, y);
  }
}

/**
 * Create a fallback camera model when no CameraInfo is available
 * Uses default focal length and assumes principal point at image center
 * 
 * @param width - Image width in pixels
 * @param height - Image height in pixels
 * @param frameId - Camera frame ID
 * @param focalLength - Default focal length in pixels (default: 500)
 * @returns Camera model with default parameters
 */
export function createFallbackCameraModel(
  width: number,
  height: number,
  frameId: string,
  focalLength: number = 500
): ICameraModel {
  const cx = width / 2;
  const cy = height / 2;
  
  const cameraInfo: CameraInfo = {
    header: {
      stamp: { sec: 0, nanosec: 0 },
      frame_id: frameId,
    },
    width,
    height,
    distortion_model: 'plumb_bob',
    K: [
      focalLength, 0, cx,
      0, focalLength, cy,
      0, 0, 1
    ],
    D: [0, 0, 0, 0, 0], // No distortion
    R: [
      1, 0, 0,
      0, 1, 0,
      0, 0, 1
    ], // Identity (no rectification)
    P: [
      focalLength, 0, cx, 0,
      0, focalLength, cy, 0,
      0, 0, 1, 0
    ],
    binning_x: 1,
    binning_y: 1,
    roi: {
      x_offset: 0,
      y_offset: 0,
      height: 0,
      width: 0,
      do_rectify: false
    }
  };
  
  return new PinholeCameraModel(cameraInfo);
}

/**
 * Normalize incoming CameraInfo to handle ROS1/ROS2 differences
 * ROS1 uses lowercase (d, k, r, p) while ROS2 uses uppercase (D, K, R, P)
 */
export function normalizeCameraInfo(info: any): CameraInfo {
  return {
    header: info.header,
    height: info.height,
    width: info.width,
    distortion_model: info.distortion_model,
    K: info.K || info.k || [],
    D: info.D || info.d || [],
    R: info.R || info.r || [],
    P: info.P || info.p || [],
    binning_x: info.binning_x ?? 1,
    binning_y: info.binning_y ?? 1,
    roi: info.roi || {
      x_offset: 0,
      y_offset: 0,
      height: 0,
      width: 0,
      do_rectify: false
    }
  };
}

