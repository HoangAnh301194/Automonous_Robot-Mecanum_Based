import { MetricCard } from "../components/MetricCard";
import { RosGraph } from "../components/RosGraph";
import { StatusBadge } from "../components/StatusBadge";
import type { DashboardState } from "../types/telemetry";

interface RosLogsPageProps {
  state: DashboardState | null;
}

function statusTone(status?: string): "ok" | "warn" | "error" | "neutral" {
  if (status === "OK" || status === "INFO") return "ok";
  if (status === "WARN" || status === "WAITING") return "warn";
  if (status === "ERROR" || status === "FATAL" || status === "STALE") return "error";
  return "neutral";
}

function formatAge(age?: number | null): string {
  if (age === null || age === undefined) return "waiting";
  if (age < 1) return `${Math.round(age * 1000)} ms`;
  return `${age.toFixed(1)} s`;
}

function formatTime(timestamp?: number): string {
  if (!timestamp) return "-";
  return new Date(timestamp * 1000).toLocaleTimeString();
}

export function RosLogsPage({ state }: RosLogsPageProps) {
  const topicRows = Object.entries(state?.topics ?? {}).sort(([left], [right]) => left.localeCompare(right));
  const monitoredTopics = topicRows.filter(([, topic]) => topic.health_monitored || topic.expected_rate_hz !== undefined);
  const topicStateCount = (topicState: string) => monitoredTopics.filter(([, topic]) => topic.state === topicState).length;
  const diagnosticSummary = state?.diagnostics?.summary ?? {};
  const diagnosticIssues = (diagnosticSummary.warn_count ?? 0)
    + (diagnosticSummary.error_count ?? 0)
    + (diagnosticSummary.stale_count ?? 0);
  const rosLogs = (state?.ros_logs ?? []).slice().reverse().slice(0, 100);
  const events = (state?.events ?? []).slice().reverse().slice(0, 50);
  const system = state?.system;

  return (
    <section className="page-section">
      <div className="section-heading">
        <div>
          <span className="section-heading__eyebrow">ROS GRAPH & LOGS</span>
          <h2>Graph topology, node exchange, diagnostics, and logs</h2>
        </div>
      </div>

      <div className="metric-grid metric-grid--five diagnostics-summary">
        <MetricCard
          title="Healthy Topics"
          value={`${topicStateCount("OK")}/${monitoredTopics.length}`}
          detail="Topics with configured rate checks"
        />
        <MetricCard
          title="Rate Warnings"
          value={`${topicStateCount("WARN")}`}
          detail="Below configured expected rate"
        />
        <MetricCard
          title="Stale Topics"
          value={`${topicStateCount("STALE")}`}
          detail={`${topicStateCount("WAITING")} still waiting`}
        />
        <MetricCard
          title="Diagnostics"
          value={`${diagnosticIssues}`}
          detail={`${diagnosticSummary.total_count ?? 0} reported statuses`}
        />
        <MetricCard
          title="Jetson Load"
          value={`${system?.cpu_percent?.toFixed(1) ?? "-"}% CPU`}
          detail={`${system?.gpu_percent?.toFixed(1) ?? "-"}% GPU · ${system?.temperature_celsius?.toFixed(1) ?? "-"} °C`}
        />
      </div>

      <RosGraph detailed graph={state?.ros_graph} topics={state?.topics} />

      <div className="panel-grid panel-grid--two">
        <article className="panel panel--wide">
          <div className="panel__heading">
            <div>
              <h3>Topic health</h3>
              <span>Rate, age, stale threshold, message count</span>
            </div>
            <StatusBadge label={`${monitoredTopics.length} MONITORED`} tone="neutral" />
          </div>
          <div className="topic-health-table">
            <div className="topic-health-row topic-health-row--header">
              <span>Topic / Type</span><span>State</span><span>Rate</span><span>Age</span><span>Messages</span>
            </div>
            {topicRows.map(([topic, info]) => (
              <div className="topic-health-row" key={topic}>
                <div className="topic-health-name">
                  <strong>{topic}</strong>
                  <small>{info.message_type ?? "unknown type"}</small>
                </div>
                <StatusBadge label={info.state ?? "UNKNOWN"} tone={statusTone(info.state)} />
                <span>{info.rate_hz?.toFixed(1) ?? "-"} / {info.expected_rate_hz?.toFixed(1) ?? "-"} Hz</span>
                <span>{formatAge(info.age_seconds)}</span>
                <span>{info.message_count?.toLocaleString() ?? "0"}</span>
              </div>
            ))}
            {!topicRows.length && <p className="empty-copy">No observed topics yet.</p>}
          </div>
        </article>

        <article className="panel">
          <div className="panel__heading">
            <div><h3>Diagnostic statuses</h3><span>Latest `/diagnostics` snapshot</span></div>
            <StatusBadge label={`${diagnosticIssues} ISSUES`} tone={diagnosticIssues ? "warn" : "ok"} />
          </div>
          <div className="diagnostic-list">
            {(state?.diagnostics?.statuses ?? []).map((diagnostic) => (
              <article
                className={`diagnostic-item diagnostic-item--${diagnostic.level_name.toLowerCase()}`}
                key={`${diagnostic.name}-${diagnostic.hardware_id ?? ""}`}
              >
                <div className="diagnostic-item__header">
                  <div><strong>{diagnostic.name}</strong><small>{diagnostic.hardware_id || "no hardware id"}</small></div>
                  <StatusBadge label={diagnostic.level_name} tone={statusTone(diagnostic.level_name)} />
                </div>
                <p>{diagnostic.message || "No diagnostic message"}</p>
                {!!Object.keys(diagnostic.values).length && (
                  <dl className="diagnostic-values">
                    {Object.entries(diagnostic.values).map(([key, value]) => (
                      <div key={key}><dt>{key}</dt><dd>{value}</dd></div>
                    ))}
                  </dl>
                )}
              </article>
            ))}
            {!state?.diagnostics?.statuses?.length && <p className="empty-copy">Waiting for `/diagnostics`.</p>}
          </div>
        </article>

        <article className="panel">
          <div className="panel__heading">
            <div><h3>ROS output</h3><span>Latest 100 `/rosout` messages</span></div>
            <StatusBadge label={`${rosLogs.length} LOGS`} tone="neutral" />
          </div>
          <div className="ros-log-list">
            {rosLogs.map((log, index) => (
              <div className={`ros-log-row ros-log-row--${log.level_name.toLowerCase()}`} key={`${log.timestamp}-${log.name}-${index}`}>
                <div className="ros-log-row__meta">
                  <StatusBadge label={log.level_name} tone={statusTone(log.level_name)} />
                  <strong>{log.name || "anonymous node"}</strong>
                  <time>{formatTime(log.timestamp)}</time>
                </div>
                <p>{log.message}</p>
                {(log.file || log.function) && <small>{log.file || "?"}:{log.line ?? 0} · {log.function || "?"}</small>}
              </div>
            ))}
            {!rosLogs.length && <p className="empty-copy">Waiting for `/rosout`.</p>}
          </div>
        </article>

        <article className="panel panel--wide">
          <div className="panel__heading">
            <div><h3>Runtime events</h3><span>Health transitions and application events</span></div>
            <StatusBadge label={`${events.length} EVENTS`} tone="neutral" />
          </div>
          <ul className="data-list runtime-event-list">
            {events.map((event) => (
              <li key={`${event.timestamp}-${event.source}-${event.message}`}>
                <strong>{event.level} · {event.source} · {formatTime(event.timestamp)}</strong>
                <span>{event.message}</span>
              </li>
            ))}
            {!events.length && <p className="empty-copy">No runtime events yet.</p>}
          </ul>
        </article>
      </div>
    </section>
  );
}
