/** ---------- Common message structs ---------- */

export interface BuiltinTime {
  sec: number;
  nanosec: number;
}

export interface StdHeader {
  stamp: BuiltinTime;
  frame_id: string;
}

export interface GeometryVector3 {
  x: number;
  y: number;
  z: number;
}

export interface GeometryPoint {
  x: number;
  y: number;
  z: number;
}

export interface GeometryQuaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface GeometryPose {
  position: GeometryPoint;
  orientation: GeometryQuaternion;
}

export interface GeometryTwist {
  linear: GeometryVector3;
  angular: GeometryVector3;
}

export interface GeometryPoseWithCovariance {
  pose: GeometryPose;
  covariance: number[];
}

export interface GeometryTwistWithCovariance {
  twist: GeometryTwist;
  covariance: number[];
}

export interface GeometryPoseStamped {
  header: StdHeader;
  pose: GeometryPose;
}

export interface GeometryTwistStamped {
  header: StdHeader;
  twist: GeometryTwist;
}

export interface NavMsgsOdometry {
  header: StdHeader;
  child_frame_id: string;
  pose: GeometryPoseWithCovariance;
  twist: GeometryTwistWithCovariance;
}

export interface SensorMsgsNavSatStatus {
  status: number;
  service: number;
}

export interface SensorMsgsNavSatFix {
  header: StdHeader;
  status: SensorMsgsNavSatStatus;
  latitude: number;
  longitude: number;
  altitude: number;
  position_covariance: number[];
  position_covariance_type: number;
}

export interface StdMsgsFloat64 {
  data: number;
}

export interface GeographicMsgsGeoPoint {
  latitude: number;
  longitude: number;
  altitude: number;
}

export interface MavrosMsgsAltitude {
  header: StdHeader;
  monotonic: number;
  amsl: number;
  local: number;
  relative: number;
  terrain: number;
  bottom_clearance: number;
  agl?: number;
}

export interface MavrosMsgsHomePosition {
  header: StdHeader;
  geo: GeographicMsgsGeoPoint;
  position: GeometryPoint;
  orientation: GeometryQuaternion;
  approach: GeometryVector3;
}

/** MAVROS State & ExtendedState */
export interface MavrosMsgsState {
  header?: StdHeader;
  connected: boolean;
  armed: boolean;
  guided: boolean;
  manual_input: boolean;
  mode: string;
  system_status: number;
}

export interface MavrosMsgsExtendedState {
  header?: StdHeader;
  landed_state: number;
  vtol_state: number;
}

/** Battery & IMU */
export interface SensorMsgsBatteryState {
  header: StdHeader;
  voltage: number;
  temperature?: number | null;
  current?: number;
  charge?: number;
  capacity?: number;
  design_capacity?: number;
  percentage?: number;
  power_supply_status?: number;
  power_supply_health?: number;
  power_supply_technology?: number;
  present?: boolean;
  cell_voltage?: number[];
  cell_temperature?: number[];
  location?: string;
  serial_number?: string;
}

export interface MavrosMsgsVFRHUD {
  airspeed?: number;
  groundspeed?: number;
  heading?: number;
  throttle?: number;
  altitude?: number;
  climb?: number;
}

export interface SensorMsgsImu {
  header: StdHeader;
  orientation: GeometryQuaternion;
  orientation_covariance?: number[];
  angular_velocity: GeometryVector3;
  angular_velocity_covariance?: number[];
  linear_acceleration: GeometryVector3;
  linear_acceleration_covariance?: number[];
}

/** Aliases for geometry_msgs names used elsewhere */
export type GeometryMsgsPoseStamped = GeometryPoseStamped;
export type GeometryMsgsTwistStamped = GeometryTwistStamped;

/** Convenience for consumers that expect decoded images */
export interface ImageMessage {
  topic: string;
  timestamp: string;          // ISO string
  timestampNanos?: number;    // nanoseconds since epoch
  frameId: string;
  encoding: string;
  width: number;
  height: number;
  data: string;               // data URI
  messageType: "raw" | "compressed";
}

export interface TwistMessage {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
}

/** ---------- MAVROS service request/response types ---------- */
/** mavros_msgs/srv/CommandBool */
export interface CommandBool_Request {
  value: boolean;
}
export interface CommandBool_Response {
  success: boolean;
  result: number;
}

/** mavros_msgs/srv/SetMode */
export interface SetMode_Request {
  base_mode: number;
  custom_mode: string;
}
export interface SetMode_Response {
  mode_sent: boolean;
}

/** mavros_msgs/srv/CommandTOL */
export interface CommandTOL_Request {
  min_pitch: number;
  yaw: number;
  latitude: number;
  longitude: number;
  altitude: number;
}
export interface CommandTOL_Response {
  success: boolean;
  result: number;
}

/** mavros_msgs/srv/ParamSet */
export interface ParamValue {
  integer: number;
  real: number;
}
export interface ParamSet_Request {
  param_id: string;
  value: ParamValue;
}
export interface ParamSet_Response {
  success: boolean;
  value: ParamValue;
}

/** mavros_msgs/srv/CommandLong */
export interface CommandLong_Request {
  command: number;
  confirmation?: number;
  param1?: number;
  param2?: number;
  param3?: number;
  param4?: number;
  param5?: number;
  param6?: number;
  param7?: number;
  broadcast?: boolean;
}
export interface CommandLong_Response {
  success: boolean;
  result: number;
}
