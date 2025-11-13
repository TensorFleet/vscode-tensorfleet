// SPDX-FileCopyrightText: Copyright (C) 2023-2025 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)<lichtblick@bmwgroup.com>
// SPDX-License-Identifier: MPL-2.0

import { type ChangeEvent, useCallback, useMemo, useState } from "react";
import { JSONTree } from "react-json-tree";

import Panel from "@lichtblick/suite-base/components/Panel";
import { useMessageDataItem } from "@lichtblick/suite-base/components/MessagePathSyntax/useMessageDataItem";
import { SaveConfig } from "@lichtblick/suite-base/types/panels";

import { RawMessagesPanelConfig } from "./types";
import "./RawMessagesPanel.css";
import { getSuggestionByTopic, getTopicSuggestionGroups } from "../../../../utils/discoveredTopics";

const DEFAULT_HISTORY = 20;

type Props = {
  config: Readonly<RawMessagesPanelConfig>;
  saveConfig: SaveConfig<RawMessagesPanelConfig>;
};

const jsonTreeTheme = {
  scheme: "lichtblick",
  author: "lichtblick",
  base00: "#1c1f26",
  base01: "#2b303b",
  base02: "#343944",
  base03: "#4f5667",
  base04: "#cbd5e1",
  base05: "#f8fafc",
  base06: "#f8fafc",
  base07: "#ffffff",
  base08: "#f38ba0",
  base09: "#f9d000",
  base0A: "#f7c948",
  base0B: "#5fb9a2",
  base0C: "#5f9df9",
  base0D: "#75bdfc",
  base0E: "#c678dd",
  base0F: "#7f8493",
};

function formatReceiveTime(time?: { sec: number; nsec: number }): string {
  if (!time) {
    return "—";
  }
  const millis = time.sec * 1000 + time.nsec / 1e6;
  return new Date(millis).toISOString();
}

function buildDisplayValue(values: unknown[]): unknown {
  if (values.length === 0) {
    return undefined;
  }
  if (values.length === 1) {
    const single = values[0];
    if (Array.isArray(single) && single.length === 1) {
      return single[0];
    }
    return single;
  }
  return values;
}

function formatPreview(value: unknown): string {
  if (value == undefined) {
    return "Payload unavailable";
  }
  if (typeof value === "object") {
    try {
      const serialized = JSON.stringify(value);
      return serialized.length > 120 ? `${serialized.slice(0, 117)}…` : serialized;
    } catch {
      return "[object]";
    }
  }
  const asText = String(value);
  return asText.length > 120 ? `${asText.slice(0, 117)}…` : asText;
}

function RawMessages(props: Props) {
  const { config, saveConfig } = props;
  const { topicPath, historySize = DEFAULT_HISTORY } = config;
  const resolvedHistorySize = Math.max(1, historySize);
  const [expandedMessages, setExpandedMessages] = useState<Record<string, boolean>>({});
  const [isHeaderExpanded, setIsHeaderExpanded] = useState(true);

  const messages = useMessageDataItem(topicPath ? topicPath : "", {
    historySize: resolvedHistorySize,
  });

  const orderedMessages = useMemo(() => {
    return [...messages].slice(-resolvedHistorySize).reverse();
  }, [messages, resolvedHistorySize]);

  const topicSuggestionGroups = useMemo(() => getTopicSuggestionGroups(), []);
  const selectedSuggestion = useMemo(
    () => (topicPath ? getSuggestionByTopic(topicPath.trim()) : undefined),
    [topicPath],
  );
  const latestMessage = orderedMessages[0];
  const latestPayloadValues = latestMessage?.queriedData?.map(({ value }) => value) ?? [];
  const latestPayload = buildDisplayValue(latestPayloadValues);
  const latestPreview = latestMessage ? formatPreview(latestPayload) : "Waiting for data";

  const onTopicPathChange = useCallback(
    (event: ChangeEvent<HTMLInputElement>) => {
      saveConfig({ topicPath: event.target.value });
    },
    [saveConfig],
  );

  const onTopicSuggestionSelect = useCallback(
    (event: ChangeEvent<HTMLSelectElement>) => {
      const nextTopic = event.target.value;
      saveConfig({ topicPath: nextTopic });
    },
    [saveConfig],
  );

  const onHistorySizeChange = useCallback(
    (event: ChangeEvent<HTMLInputElement>) => {
      const parsed = Number(event.target.value);
      const nextValue = Number.isFinite(parsed) ? Math.max(1, Math.round(parsed)) : DEFAULT_HISTORY;
      saveConfig({ historySize: nextValue });
    },
    [saveConfig],
  );

  const toggleMessageExpansion = useCallback((key: string) => {
    setExpandedMessages((prev) => ({ ...prev, [key]: !prev[key] }));
  }, []);

  const hasSelectedTopic = topicPath.trim().length > 0;
  const hasMessages = orderedMessages.length > 0;
  const statusLabel = hasSelectedTopic ? (hasMessages ? "Receiving" : "Waiting…") : "Idle";
  const statusClassName = [
    "raw-messages-panel__chip",
    !hasSelectedTopic
      ? "raw-messages-panel__chip--idle"
      : hasMessages
        ? "raw-messages-panel__chip--active"
        : "raw-messages-panel__chip--waiting",
  ].join(" ");

  const toggleHeaderExpansion = () => setIsHeaderExpanded((prev) => !prev);

  const body =
    topicPath.length === 0 ? (
      <div className="raw-messages-panel__empty">Select a topic to start streaming data.</div>
    ) : orderedMessages.length === 0 ? (
      <div className="raw-messages-panel__empty">Waiting for messages on {topicPath}…</div>
    ) : (
      <div className="raw-messages-panel__list">
        {orderedMessages.map((item, index) => {
          const values = item.queriedData?.map(({ value }) => value) ?? [];
          const payload = buildDisplayValue(values);
          const key =
            item.messageEvent?.receiveTime?.sec != undefined
              ? `${item.messageEvent.receiveTime.sec}-${item.messageEvent.receiveTime.nsec}-${index}`
              : `${topicPath}-${index}`;
          const hasObjectPayload = payload != undefined && typeof payload === "object";
          const isExpanded = expandedMessages[key] ?? index === 0;
          return (
            <article className="raw-message-card" key={key}>
              <header className="raw-message-card__header">
                <div className="raw-message-card__meta">
                  <div className="raw-message-card__topic">{item.messageEvent?.topic ?? topicPath}</div>
                  <div className="raw-message-card__subtitle">{formatReceiveTime(item.messageEvent?.receiveTime)}</div>
                </div>
                <button
                  className="raw-message-card__toggle"
                  type="button"
                  onClick={() => toggleMessageExpansion(key)}
                  aria-expanded={isExpanded}
                >
                  {isExpanded ? "Hide payload" : "Show payload"}
                  <span aria-hidden="true">{isExpanded ? "▾" : "▸"}</span>
                </button>
              </header>
              {!isExpanded && <div className="raw-message-card__preview">{formatPreview(payload)}</div>}
              {isExpanded && (
                <div className="raw-message-card__payload">
                  {hasObjectPayload ? (
                    <JSONTree
                      data={payload as object}
                      theme={jsonTreeTheme}
                      invertTheme={false}
                      hideRoot
                      shouldExpandNode={() => true}
                    />
                  ) : (
                    <pre className="raw-message-card__plaintext">
                      {payload == undefined ? "undefined" : String(payload)}
                    </pre>
                  )}
                </div>
              )}
            </article>
          );
        })}
      </div>
    );

  return (
    <div className="raw-messages-panel">
      <header
        className={[
          "raw-messages-panel__header",
          isHeaderExpanded ? "raw-messages-panel__header--expanded" : "raw-messages-panel__header--collapsed",
        ].join(" ")}
      >
        <div className="raw-messages-panel__header-top">
          <div>
            <div className="raw-messages-panel__title-row">
              <span className="raw-messages-panel__title">Raw Messages</span>
              <span className={statusClassName}>{statusLabel}</span>
            </div>
            <div className="raw-messages-panel__status-row">
              <span>{hasSelectedTopic ? `Subscribed to ${topicPath}` : "Select a topic to subscribe"}</span>
              {hasMessages && <span>{orderedMessages.length} cached</span>}
            </div>
          </div>
          <button
            className="raw-messages-panel__header-toggle"
            type="button"
            onClick={toggleHeaderExpansion}
            aria-expanded={isHeaderExpanded}
          >
            {isHeaderExpanded ? "Hide controls" : "Show controls"}
            <span aria-hidden="true">{isHeaderExpanded ? "▴" : "▾"}</span>
          </button>
        </div>
        {isHeaderExpanded && (
          <>
            <div className="raw-messages-panel__insights">
              <div className="raw-messages-panel__insight">
                <span className="raw-messages-panel__insight-label">Topic</span>
                <span className="raw-messages-panel__insight-value">{hasSelectedTopic ? topicPath : "Not set"}</span>
              </div>
              <div className="raw-messages-panel__insight">
                <span className="raw-messages-panel__insight-label">Message type</span>
                <span className="raw-messages-panel__insight-value">
                  {selectedSuggestion?.type ?? "Unknown message type"}
                </span>
              </div>
              <div className="raw-messages-panel__insight">
                <span className="raw-messages-panel__insight-label">Last payload</span>
                <span className="raw-messages-panel__insight-value raw-messages-panel__insight-value--truncate">
                  {hasMessages ? latestPreview : "Waiting for messages"}
                </span>
              </div>
            </div>
            <section className="raw-messages-panel__controls" aria-label="Raw message controls">
              <label className="raw-messages-panel__form-group raw-messages-panel__form-group--topic">
                <span className="raw-messages-panel__label">Topic</span>
                <div className="raw-messages-panel__topic-control">
                  <select
                    className="raw-messages-panel__select"
                    value={selectedSuggestion?.topic ?? ""}
                    onChange={onTopicSuggestionSelect}
                  >
                    <option value="">Select a known topic…</option>
                    {topicSuggestionGroups.map((group) => (
                      <optgroup key={group.id} label={group.label}>
                        {group.suggestions.map((suggestion) => (
                          <option key={suggestion.topic} value={suggestion.topic}>
                            {suggestion.topic} — {suggestion.type}
                          </option>
                        ))}
                      </optgroup>
                    ))}
                  </select>
                  <input
                    className="raw-messages-panel__input"
                    value={topicPath}
                    onChange={onTopicPathChange}
                    placeholder="/clock"
                  />
                </div>
                <span className="raw-messages-panel__hint">
                  {selectedSuggestion
                    ? `${selectedSuggestion.type}${
                        selectedSuggestion.description ? ` • ${selectedSuggestion.description}` : ""
                      }`
                    : "Pick a topic from the dropdown or type a custom topic path"}
                </span>
              </label>
              <label className="raw-messages-panel__form-group" style={{ maxWidth: 160 }}>
                <span className="raw-messages-panel__label">History size</span>
                <input
                  className="raw-messages-panel__input"
                  type="number"
                  min={1}
                  value={resolvedHistorySize}
                  onChange={onHistorySizeChange}
                />
                <span className="raw-messages-panel__hint">Messages retained</span>
              </label>
            </section>
          </>
        )}
      </header>
      {!isHeaderExpanded && (
        <section className="raw-messages-panel__collapsed-controls">
          <div>
            <span className="raw-messages-panel__label">Topic</span>
            <span className="raw-messages-panel__collapsed-value">{topicPath || "Not set"}</span>
          </div>
          <button className="raw-messages-panel__collapsed-button" type="button" onClick={toggleHeaderExpansion}>
            Edit
          </button>
        </section>
      )}
      {body}
    </div>
  );
}

const defaultConfig: RawMessagesPanelConfig = {
  topicPath: "",
  historySize: DEFAULT_HISTORY,
};

export default Panel(
  Object.assign(RawMessages, {
    panelType: "RawMessages",
    defaultConfig,
  }),
);
