import { useMemo } from "react";

import type {
  RosGraphEdge,
  RosGraphNode,
  TopicState,
} from "../types/telemetry";

interface RosGraphInspectorProps {
  selected?: RosGraphNode;
  nodes: RosGraphNode[];
  edges: RosGraphEdge[];
  topics: Record<string, TopicState>;
}

function formatAge(timestamp: number | undefined): string {
  if (!timestamp) return "No sample";
  const ageSeconds = Math.max(0, Date.now() / 1000 - timestamp);
  if (ageSeconds < 1) return "< 1s ago";
  if (ageSeconds < 60) return `${ageSeconds.toFixed(0)}s ago`;
  return `${(ageSeconds / 60).toFixed(1)}m ago`;
}

function formatPayload(value: unknown): string {
  if (value === undefined) return "Payload not sampled by robot_ui";
  try {
    return JSON.stringify(value, null, 2);
  } catch {
    return String(value);
  }
}

export function RosGraphInspector({
  selected,
  nodes,
  edges,
  topics,
}: RosGraphInspectorProps) {
  const details = useMemo(() => {
    if (!selected) return null;
    const nodeById = new Map(nodes.map((node) => [node.id, node]));
    const selectedEdges = edges.filter(
      (edge) => edge.source === selected.id || edge.target === selected.id,
    );
    const exchanges = selectedEdges.map((edge) => {
      const topicId = edge.kind === "publish" ? edge.target : edge.source;
      const endpointId = edge.kind === "publish" ? edge.source : edge.target;
      const topicNode = nodeById.get(topicId);
      const endpointNode = nodeById.get(endpointId);
      const topicName = topicNode?.label ?? topicId.replace("topic:", "");
      const topicState = topics[topicName];
      const relation = selected.kind === "node"
        ? edge.kind === "publish" ? "PUBLISHES" : "SUBSCRIBES"
        : edge.kind === "publish" ? "PUBLISHER" : "SUBSCRIBER";
      return {
        edge,
        topicName,
        topicState,
        relation,
        title: selected.kind === "node" ? topicName : endpointNode?.label ?? endpointId,
        messageTypes: topicNode?.message_types ?? edge.message_types ?? [],
      };
    }).sort((first, second) => {
      if (first.edge.kind !== second.edge.kind) {
        return first.edge.kind === "publish" ? -1 : 1;
      }
      return first.title.localeCompare(second.title);
    });

    const publishCount = selected.kind === "node"
      ? selectedEdges.filter((edge) => edge.kind === "publish" && edge.source === selected.id).length
      : selectedEdges.filter((edge) => edge.kind === "publish" && edge.target === selected.id).length;
    const subscribeCount = selected.kind === "node"
      ? selectedEdges.filter((edge) => edge.kind === "subscribe" && edge.target === selected.id).length
      : selectedEdges.filter((edge) => edge.kind === "subscribe" && edge.source === selected.id).length;
    const messageTypes = selected.kind === "topic"
      ? selected.message_types ?? []
      : [...new Set(exchanges.flatMap((exchange) => exchange.messageTypes))];

    return {
      exchanges,
      messageTypes,
      publishCount,
      subscribeCount,
    };
  }, [edges, nodes, selected, topics]);

  if (!selected || !details) {
    return <aside className="ros-inspector ros-inspector--empty">Select a node or topic.</aside>;
  }

  return (
    <aside className="ros-inspector">
      <header className="ros-inspector__header">
        <div>
          <span>{selected.kind.toUpperCase()} INSPECTOR</span>
          <strong>{selected.label}</strong>
          <small>{selected.namespace ? `Namespace ${selected.namespace}` : selected.id}</small>
        </div>
        <i className={`ros-inspector__kind ros-inspector__kind--${selected.kind}`} />
      </header>

      <div className="ros-inspector__stats">
        <div><span>Publish</span><strong>{details.publishCount}</strong></div>
        <div><span>Subscribe</span><strong>{details.subscribeCount}</strong></div>
        <div><span>Connections</span><strong>{details.exchanges.length}</strong></div>
      </div>

      <div className="ros-inspector__meta">
        <span>MESSAGE TYPES</span>
        <strong>{details.messageTypes.join(", ") || "Not discovered"}</strong>
      </div>

      <div className="ros-inspector__section-title">
        <span>DATA EXCHANGE</span>
        <strong>{details.exchanges.length}</strong>
      </div>

      <div className="ros-inspector__exchanges">
        {details.exchanges.length ? details.exchanges.map((exchange) => {
          const topicState = exchange.topicState;
          const messageType = exchange.messageTypes.join(", ") || topicState?.message_type || "Unknown type";
          return (
            <article className="exchange-card" key={exchange.edge.id}>
              <header>
                <span className={`exchange-card__direction exchange-card__direction--${exchange.edge.kind}`}>
                  {exchange.relation}
                </span>
                <span className={`exchange-card__state exchange-card__state--${topicState?.state?.toLowerCase() ?? "idle"}`}>
                  {topicState?.state ?? "UNOBSERVED"}
                </span>
              </header>
              <strong className="exchange-card__title">{exchange.title}</strong>
              <small>{messageType}</small>

              <dl className="exchange-card__metrics">
                <div><dt>Rate</dt><dd>{topicState?.rate_hz?.toFixed(1) ?? "-"} Hz</dd></div>
                <div><dt>Expected</dt><dd>{topicState?.expected_rate_hz?.toFixed(1) ?? "-"} Hz</dd></div>
                <div><dt>Messages</dt><dd>{topicState?.message_count ?? 0}</dd></div>
                <div><dt>Last sample</dt><dd>{formatAge(topicState?.last_message_at)}</dd></div>
              </dl>

              <div className="exchange-card__qos">
                <span>QoS</span>
                <i>{exchange.edge.qos?.reliability ?? "-"}</i>
                <i>{exchange.edge.qos?.durability ?? "-"}</i>
                <i>{exchange.edge.qos?.history ?? "-"} {exchange.edge.qos?.depth ?? "-"}</i>
              </div>

              <div className="exchange-card__payload">
                <span>LATEST PAYLOAD</span>
                <pre>{formatPayload(topicState?.latest)}</pre>
              </div>
            </article>
          );
        }) : <p className="empty-copy">No topic connections discovered.</p>}
      </div>
    </aside>
  );
}
