// ros2-bridge-api.ts

import type * as RosTypes from "./ros-types";

export interface Subscription {
  topic: string;
  type: string;
}

export type MessageHandler = (message: any) => void;
export type UnsubscribeFn = () => void;
export type TopicsChangedHandler = (topics: Subscription[]) => void;

export interface ROS2BridgeApi {
  isConnected(): boolean;

  /** Topic subscription API (returns callable unsubscribe) */
  subscribe(subscription: Subscription, handler: MessageHandler): UnsubscribeFn;
  unsubscribe(topic: string, handler: MessageHandler): void;

  /** Topic publishing API */
  publish(topic: string, messageType: string, message: any): void;

  /**
   * Arrange for a topic publish to be sent immediately after connect
   * and on every reconnect (e.g. latched configs).
   */
  publishSetup(topic: string, type: string, message: any): void;

  /**
   * Arrange for a service call to run once on every (re)connect
   * before normal operations.
   */
  registerSetupServiceCall(name: string, request: any): void;

  /**
   * Arrange for a ROS parameter set to run once on every (re)connect
   * before normal operations.
   */
  registerSetupROSParameterSet(name: string, value: any): void;

  /** Topic discovery & helpers */
  getAvailableTopics(): Subscription[];
  getTopicType(topic: string): string | undefined;
  getAvailableImageTopics(): Subscription[];

  /** Generic service call API (Foxglove-backed) */
  callService<T = any>(name: string, request: any): Promise<T>;

  /** MAVROS service helpers */
  mavrosCommandLong(
    req: RosTypes.CommandLong_Request,
  ): Promise<RosTypes.CommandLong_Response>;

  mavrosArmDisarm(
    value: boolean,
  ): Promise<RosTypes.CommandBool_Response>;

  mavrosSetMode(
    custom_mode: string,
    base_mode?: number,
  ): Promise<RosTypes.SetMode_Response>;

  mavrosTakeoff(args: {
    altitude: number;
    min_pitch?: number;
    yaw?: number;
    latitude?: number;
    longitude?: number;
  }): Promise<RosTypes.CommandTOL_Response>;

  mavrosLand(args?: {
    altitude?: number;
    yaw?: number;
    latitude?: number;
    longitude?: number;
  }): Promise<RosTypes.CommandTOL_Response>;

  mavrosParamSet(
    param_id: string,
    value: RosTypes.ParamValue,
  ): Promise<RosTypes.ParamSet_Response>;

  /** Frame/introspection helpers */
  getKnownFrames(): string[];
  getFrameSources(frameId: string): string[];

  /** Available-topics change listener (returns callable unsubscribe) */
  onAvailableTopicsChanged(cb: TopicsChangedHandler): UnsubscribeFn;
}
