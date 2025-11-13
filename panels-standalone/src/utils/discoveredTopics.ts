import { ros2Bridge } from "../ros2-bridge";

export type DiscoveredTopic = {
  topic: string;
  type: string;
};

export type TopicGroup = {
  id: string;
  label: string;
  suggestions: DiscoveredTopic[];
};

function getDiscoveredTopics(): DiscoveredTopic[] {
  return ros2Bridge.getAvailableTopics().map(({ topic, type }) => ({ topic, type }));
}

export function getTopicSuggestions(): DiscoveredTopic[] {
  return getDiscoveredTopics();
}

export function getSuggestionByTopic(topic: string): DiscoveredTopic | undefined {
  const type = ros2Bridge.getTopicType(topic);
  return type ? { topic, type } : undefined;
}

export function getImageTopicSuggestions(): DiscoveredTopic[] {
  return ros2Bridge.getAvailableImageTopics().map(({ topic, type }) => ({ topic, type }));
}

export function getTopicSuggestionGroups(): TopicGroup[] {
  const topics = getDiscoveredTopics();
  
  if (topics.length === 0) {
    return [];
  }

  // Return a single group with all discovered topics
  return [
    {
      id: "discovered",
      label: "Available Topics",
      suggestions: topics,
    },
  ];
}

// Export type alias for backward compatibility
export type TopicSuggestion = DiscoveredTopic;
export type TopicSuggestionGroup = TopicGroup;

