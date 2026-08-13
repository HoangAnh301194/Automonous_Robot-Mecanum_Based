import { useEffect, useMemo, useState } from "react";

import type {
  RosGraphEdge,
  RosGraphNode,
  RosGraphState,
  TopicState,
} from "../types/telemetry";
import { RosGraphInspector } from "./RosGraphInspector";


interface RosGraphProps {
  graph?: RosGraphState;
  topics?: Record<string, TopicState>;
  compact?: boolean;
  detailed?: boolean;
}

interface Point {
  x: number;
  y: number;
}

const WIDTH = 900;
const HEIGHT = 520;
const CENTER: Point = { x: WIDTH / 2, y: HEIGHT / 2 };


function shortLabel(label: string): string {
  if (label.length <= 28) return label;
  return `${label.slice(0, 12)}…${label.slice(-13)}`;
}


function curvePath(source: Point, target: Point, edge: RosGraphEdge): string {
  const dx = target.x - source.x;
  const dy = target.y - source.y;
  const distance = Math.max(1, Math.hypot(dx, dy));
  const normalX = -dy / distance;
  const normalY = dx / distance;
  const bend = edge.kind === "publish" ? 17 : -17;
  const controlX = (source.x + target.x) / 2 + normalX * bend;
  const controlY = (source.y + target.y) / 2 + normalY * bend;
  return `M ${source.x} ${source.y} Q ${controlX} ${controlY} ${target.x} ${target.y}`;
}


export function RosGraph({
  graph,
  topics = {},
  compact = false,
  detailed = false,
}: RosGraphProps) {
  const nodes = graph?.nodes ?? [];
  const edges = graph?.edges ?? [];
  const visibleLimit = compact ? 12 : 22;
  const [selectedId, setSelectedId] = useState<string>("");
  const [search, setSearch] = useState("");
  const [zoom, setZoom] = useState(1);

  useEffect(() => {
    if (selectedId && nodes.some((node) => node.id === selectedId)) return;
    const preferred = nodes.find((node) => node.label.includes("robot_ui_bridge"));
    setSelectedId(preferred?.id ?? nodes.find((node) => node.kind === "node")?.id ?? nodes[0]?.id ?? "");
  }, [nodes, selectedId]);

  const selectedNode = nodes.find((node) => node.id === selectedId);

  const visible = useMemo(() => {
    if (!nodes.length) return { nodes: [] as RosGraphNode[], edges: [] as RosGraphEdge[] };

    const nodeById = new Map(nodes.map((node) => [node.id, node]));
    const adjacency = new Map<string, Set<string>>();
    edges.forEach((edge) => {
      if (!adjacency.has(edge.source)) adjacency.set(edge.source, new Set());
      if (!adjacency.has(edge.target)) adjacency.set(edge.target, new Set());
      adjacency.get(edge.source)?.add(edge.target);
      adjacency.get(edge.target)?.add(edge.source);
    });

    const query = search.trim().toLowerCase();
    const rootId = query
      ? nodes.find((node) => node.label.toLowerCase().includes(query))?.id ?? selectedId
      : selectedId;
    const queue = rootId ? [rootId] : [];
    const included = new Set<string>();

    while (queue.length && included.size < visibleLimit) {
      const current = queue.shift()!;
      if (included.has(current) || !nodeById.has(current)) continue;
      included.add(current);
      for (const neighbor of adjacency.get(current) ?? []) {
        if (!included.has(neighbor)) queue.push(neighbor);
      }
    }

    if (query) {
      nodes
        .filter((node) => node.label.toLowerCase().includes(query))
        .slice(0, 6)
        .forEach((node) => included.add(node.id));
    }

    if (!included.size) nodes.slice(0, visibleLimit).forEach((node) => included.add(node.id));

    return {
      nodes: [...included].map((id) => nodeById.get(id)!).filter(Boolean),
      edges: edges.filter((edge) => included.has(edge.source) && included.has(edge.target)),
    };
  }, [edges, nodes, search, selectedId, visibleLimit]);

  const positions = useMemo(() => {
    const result = new Map<string, Point>();
    if (!visible.nodes.length) return result;

    const centerId = visible.nodes.some((node) => node.id === selectedId)
      ? selectedId
      : visible.nodes[0].id;
    result.set(centerId, CENTER);

    const directIds = new Set<string>();
    visible.edges.forEach((edge) => {
      if (edge.source === centerId) directIds.add(edge.target);
      if (edge.target === centerId) directIds.add(edge.source);
    });

    const direct = visible.nodes.filter((node) => directIds.has(node.id)).slice(0, 11);
    const outer = visible.nodes.filter((node) => node.id !== centerId && !directIds.has(node.id));

    direct.forEach((node, index) => {
      const angle = -Math.PI / 2 + (index * Math.PI * 2) / Math.max(1, direct.length);
      result.set(node.id, {
        x: CENTER.x + Math.cos(angle) * 165,
        y: CENTER.y + Math.sin(angle) * 145,
      });
    });

    outer.forEach((node, index) => {
      const angle = -Math.PI / 2 + (index * Math.PI * 2) / Math.max(1, outer.length);
      result.set(node.id, {
        x: CENTER.x + Math.cos(angle) * 330,
        y: CENTER.y + Math.sin(angle) * 225,
      });
    });

    return result;
  }, [selectedId, visible.edges, visible.nodes]);

  return (
    <div className={`ros-graph${compact ? " ros-graph--compact" : ""}${detailed ? " ros-graph--detailed" : ""}`}>
      <div className="ros-graph__toolbar">
        <div>
          <strong>ROS 2 Graph</strong>
          <span>{compact ? `${nodes.length} graph entities` : "Click a node or topic to inspect and recenter"}</span>
        </div>
        {!compact && <div className="ros-graph__actions">
          <input
            aria-label="Search ROS graph"
            onChange={(event) => setSearch(event.target.value)}
            placeholder="Search node or topic"
            value={search}
          />
          <button onClick={() => setZoom((value) => Math.min(1.5, value + 0.1))} type="button">+</button>
          <button onClick={() => setZoom((value) => Math.max(0.65, value - 0.1))} type="button">−</button>
          <button onClick={() => { setZoom(1); setSearch(""); }} type="button">Reset</button>
        </div>}
      </div>

      <div className={`ros-graph__workspace${detailed ? " ros-graph__workspace--detailed" : ""}`}>
        <div className="ros-graph__canvas">
        {!visible.nodes.length ? (
          <div className="ros-graph__empty">Waiting for ROS graph...</div>
        ) : (
          <svg viewBox={`0 0 ${WIDTH} ${HEIGHT}`} role="img" aria-label="ROS 2 graph">
            <defs>
              <filter id="selected-glow" x="-80%" y="-80%" width="260%" height="260%">
                <feGaussianBlur stdDeviation="8" result="blur" />
                <feMerge><feMergeNode in="blur" /><feMergeNode in="SourceGraphic" /></feMerge>
              </filter>
              <marker id="arrow-publish" markerHeight="5" markerWidth="6" orient="auto" refX="5" refY="2.5">
                <path d="M0,0 L0,5 L6,2.5 z" fill="#ff7048" />
              </marker>
              <marker id="arrow-subscribe" markerHeight="5" markerWidth="6" orient="auto" refX="5" refY="2.5">
                <path d="M0,0 L0,5 L6,2.5 z" fill="#49d8c2" />
              </marker>
            </defs>
            <g transform={`translate(${CENTER.x * (1 - zoom)} ${CENTER.y * (1 - zoom)}) scale(${zoom})`}>
              {visible.edges.map((edge) => {
                const source = positions.get(edge.source);
                const target = positions.get(edge.target);
                if (!source || !target) return null;
                return (
                  <path
                    className={`ros-edge ros-edge--${edge.kind}`}
                    d={curvePath(source, target, edge)}
                    key={edge.id}
                    markerEnd={`url(#arrow-${edge.kind})`}
                  />
                );
              })}
              {visible.nodes.map((node) => {
                const position = positions.get(node.id);
                if (!position) return null;
                const selected = node.id === selectedId;
                const width = Math.min(190, Math.max(98, shortLabel(node.label).length * 7.2 + 34));
                return (
                  <g
                    className={`ros-node ros-node--${node.kind}${selected ? " ros-node--selected" : ""}`}
                    key={node.id}
                    onClick={() => setSelectedId(node.id)}
                    onKeyDown={(event) => {
                      if (event.key === "Enter" || event.key === " ") setSelectedId(node.id);
                    }}
                    role="button"
                    style={{ filter: selected ? "url(#selected-glow)" : undefined }}
                    tabIndex={0}
                    transform={`translate(${position.x - width / 2} ${position.y - 20})`}
                  >
                    <rect height="40" rx="7" width={width} />
                    <circle cx="15" cy="20" r="5" />
                    <text x="27" y="24">{shortLabel(node.label)}</text>
                  </g>
                );
              })}
            </g>
          </svg>
        )}
        </div>
        {detailed && (
          <RosGraphInspector
            edges={edges}
            nodes={nodes}
            selected={selectedNode}
            topics={topics}
          />
        )}
      </div>

      <div className="ros-graph__footer">
        <div className="graph-legend">
          <span><i className="legend-dot legend-dot--node" />Node</span>
          <span><i className="legend-dot legend-dot--topic" />Topic</span>
          <span><i className="legend-line legend-line--publish" />Publish</span>
          <span><i className="legend-line legend-line--subscribe" />Subscribe</span>
        </div>
        <span>{selectedNode?.label ?? "No selection"}</span>
      </div>
    </div>
  );
}
