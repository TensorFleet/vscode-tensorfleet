export type TopicCategory = "time" | "tf" | "image" | "calibration" | "navigation" | "status" | "misc";

export type TopicSuggestion = {
  topic: string;
  type: string;
  category?: TopicCategory;
  description?: string;
};

export type TopicSuggestionGroup = {
  id: TopicCategory | "common";
  label: string;
  description?: string;
  suggestions: TopicSuggestion[];
};

const DEFAULT_TOPIC_SUGGESTIONS: TopicSuggestion[] = [
  { topic: "/clock", type: "rosgraph_msgs/msg/Clock", category: "time", description: "Simulation or hardware clock source" },
  { topic: "/tf", type: "tf2_msgs/msg/TFMessage", category: "tf", description: "Transform tree" },
  { topic: "/tf_static", type: "tf2_msgs/msg/TFMessage", category: "tf", description: "Static transforms" },
  { topic: "/diagnostics", type: "diagnostic_msgs/msg/DiagnosticArray", category: "status", description: "System diagnostics" },
  { topic: "/cmd_vel", type: "geometry_msgs/msg/Twist", category: "navigation", description: "Velocity commands" },
  { topic: "/odom", type: "nav_msgs/msg/Odometry", category: "navigation", description: "Robot odometry" },
  { topic: "/joint_states", type: "sensor_msgs/msg/JointState", category: "status", description: "Joint telemetry" },
  { topic: "/clock/raw", type: "std_msgs/msg/String", category: "time", description: "Raw time feed" },
  { topic: "/camera/image_raw", type: "sensor_msgs/msg/Image", category: "image", description: "Primary camera stream" },
  { topic: "/camera/image_compressed", type: "sensor_msgs/msg/CompressedImage", category: "image", description: "Compressed camera stream" },
  { topic: "/camera/color/image_raw", type: "sensor_msgs/msg/Image", category: "image", description: "RGB camera stream" },
  { topic: "/camera/depth/image_raw", type: "sensor_msgs/msg/Image", category: "image", description: "Depth camera stream" },
  { topic: "/camera_info", type: "sensor_msgs/msg/CameraInfo", category: "calibration", description: "Camera calibration" },
  { topic: "/camera/camera_info", type: "sensor_msgs/msg/CameraInfo", category: "calibration", description: "Camera calibration" },
  { topic: "/usb_cam/image_raw", type: "sensor_msgs/msg/Image", category: "image", description: "USB camera feed" },
  { topic: "/image", type: "sensor_msgs/msg/Image", category: "image" },
  { topic: "/image_raw", type: "sensor_msgs/msg/Image", category: "image" },
  { topic: "/image_compressed", type: "sensor_msgs/msg/CompressedImage", category: "image" },
];

const CATEGORY_METADATA: Record<TopicCategory | "common", { label: string; description?: string }> = {
  common: { label: "Common", description: "Frequently inspected topics" },
  time: { label: "Time & Clock", description: "Clock synchronization sources" },
  tf: { label: "Transforms", description: "Robot and sensor frames" },
  image: { label: "Imaging", description: "Camera and perception feeds" },
  calibration: { label: "Calibration", description: "Sensor configuration topics" },
  navigation: { label: "Navigation", description: "Motion and localization" },
  status: { label: "Status", description: "System health and diagnostics" },
  misc: { label: "Other", description: "Miscellaneous streams" },
};

const COMMON_TOPICS = ["/clock", "/tf", "/diagnostics"];

function cloneSuggestions(suggestions: TopicSuggestion[]): TopicSuggestion[] {
  return suggestions.map((suggestion) => ({ ...suggestion }));
}

export function getTopicSuggestions(): TopicSuggestion[] {
  return cloneSuggestions(DEFAULT_TOPIC_SUGGESTIONS);
}

export function getSuggestionByTopic(topic: string): TopicSuggestion | undefined {
  return DEFAULT_TOPIC_SUGGESTIONS.find((suggestion) => suggestion.topic === topic);
}

export function getImageTopicSuggestions(): TopicSuggestion[] {
  return cloneSuggestions(DEFAULT_TOPIC_SUGGESTIONS.filter((suggestion) => suggestion.category === "image"));
}

export function getTopicSuggestionGroups(): TopicSuggestionGroup[] {
  const grouped = new Map<TopicSuggestionGroup["id"], TopicSuggestionGroup>();

  const ensureGroup = (id: TopicSuggestionGroup["id"]) => {
    if (!grouped.has(id)) {
      const meta = CATEGORY_METADATA[id];
      grouped.set(id, {
        id,
        label: meta?.label ?? id,
        description: meta?.description,
        suggestions: [],
      });
    }
    return grouped.get(id)!;
  };

  DEFAULT_TOPIC_SUGGESTIONS.forEach((suggestion) => {
    const category = suggestion.category ?? "misc";
    if (COMMON_TOPICS.includes(suggestion.topic)) {
      ensureGroup("common").suggestions.push({ ...suggestion });
    }
    ensureGroup(category).suggestions.push({ ...suggestion });
  });

  return Array.from(grouped.values()).filter((group) => group.suggestions.length > 0);
}
