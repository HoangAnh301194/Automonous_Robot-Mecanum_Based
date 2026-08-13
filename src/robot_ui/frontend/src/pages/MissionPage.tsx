import { MetricCard } from "../components/MetricCard";
import { StatusBadge } from "../components/StatusBadge";
import type { DashboardState } from "../types/telemetry";

interface MissionPageProps {
  state: DashboardState | null;
}

const missionFlow = [
  "WAIT_FOR_B",
  "GO_TO_B",
  "GO_TO_PERSON",
  "WAIT_10S",
  "WAIT_AT_WAYPOINT",
  "DONE",
];

function statusTone(status?: string): "ok" | "warn" | "error" | "neutral" {
  if (status === "EXECUTING" || status === "ACCEPTED" || status === "SUCCEEDED" || status === "DONE") return "ok";
  if (status === "WAIT_FOR_B" || status === "WAIT_AT_WAYPOINT" || status === "WAIT_10S" || status === "SENDING") return "warn";
  if (status === "ABORTED" || status === "REJECTED" || status === "SERVER_UNAVAILABLE" || status === "ERROR") return "error";
  return "neutral";
}

export function MissionPage({ state }: MissionPageProps) {
  const mission = state?.mission;
  const waypoint = mission?.waypoint;
  const nav2 = mission?.nav2;
  const intercept = mission?.intercept;
  const topic = state?.topics["/mission/status"];
  const progress = waypoint?.total
    ? Math.min(100, Math.max(0, ((waypoint.display_index ?? 0) / waypoint.total) * 100))
    : 0;

  return (
    <section className="page-section">
      <div className="section-heading">
        <div>
          <span className="section-heading__eyebrow">MISSION DEBUG</span>
          <h2>Mission state, waypoint progress, Nav2, and person intercept</h2>
        </div>
        <StatusBadge label={topic?.state ?? "WAITING"} tone={statusTone(topic?.state)} />
      </div>

      <div className="metric-grid metric-grid--five">
        <MetricCard title="Mission Mode" value={mission?.mode ?? "WAITING"} detail={mission?.active ? "Mission active" : "Mission idle"} />
        <MetricCard title="Waypoint" value={`${waypoint?.display_index ?? 0}/${waypoint?.total ?? 0}`} detail={waypoint?.name || "No active waypoint"} />
        <MetricCard title="Nav2" value={nav2?.state ?? "IDLE"} detail={nav2?.label || "No active goal"} />
        <MetricCard title="Distance" value={`${nav2?.distance_remaining?.toFixed(2) ?? "-"} m`} detail={nav2?.goal_active ? "Goal active" : "No active Nav2 goal"} />
        <MetricCard title="Wait" value={`${mission?.wait_remaining_seconds?.toFixed(1) ?? "0.0"} s`} detail={intercept?.done_this_trip ? "Intercept completed" : "No completed intercept"} />
      </div>

      <div className="mission-flow mission-flow--live">
        {missionFlow.map((mode) => (
          <span className={mission?.mode === mode ? "mission-flow__step--active" : ""} key={mode}>{mode}</span>
        ))}
      </div>

      <article className="panel mission-progress-panel">
        <div className="panel__heading">
          <div><h3>Waypoint progress</h3><span>{waypoint?.name || "Waiting for waypoint batch"}</span></div>
          <StatusBadge label={`${progress.toFixed(0)}%`} tone={progress >= 100 ? "ok" : "neutral"} />
        </div>
        <div className="mission-progress"><span style={{ width: `${progress}%` }} /></div>
      </article>

      <div className="panel-grid panel-grid--three">
        <article className="panel">
          <div className="panel__heading">
            <div><h3>Current goal</h3><span>Published mission target</span></div>
            <StatusBadge label={nav2?.state ?? "IDLE"} tone={statusTone(nav2?.state)} />
          </div>
          <dl className="debug-values">
            <div><dt>Frame</dt><dd>{mission?.goal?.frame_id ?? "-"}</dd></div>
            <div><dt>X / Y</dt><dd>{mission?.goal ? `${mission.goal.x?.toFixed(2) ?? "-"} / ${mission.goal.y?.toFixed(2) ?? "-"}` : "-"}</dd></div>
            <div><dt>Result</dt><dd>{nav2?.result_status || "-"}</dd></div>
            <div><dt>Topic rate</dt><dd>{topic?.rate_hz?.toFixed(1) ?? "-"} Hz</dd></div>
          </dl>
        </article>

        <article className="panel">
          <div className="panel__heading">
            <div><h3>Person intercept</h3><span>Wave-triggered mission branch</span></div>
            <StatusBadge label={intercept?.person_detected ? "PERSON" : "CLEAR"} tone={intercept?.person_detected ? "warn" : "ok"} />
          </div>
          <dl className="debug-values">
            <div><dt>Enabled</dt><dd>{intercept?.enabled ? "Yes" : "No"}</dd></div>
            <div><dt>Track</dt><dd>{intercept?.track_id || "-"}</dd></div>
            <div><dt>Hand / Hold</dt><dd>{intercept?.hand || "-"} / {intercept?.wave_hold_seconds?.toFixed(1) ?? "0.0"} s</dd></div>
            <div><dt>Distance</dt><dd>{intercept?.distance_m?.toFixed(2) ?? "-"} m</dd></div>
          </dl>
        </article>

        <article className="panel">
          <div className="panel__heading">
            <div><h3>Status source</h3><span>Read-only mission telemetry</span></div>
            <StatusBadge label={mission?.last_error ? "ERROR" : "OK"} tone={mission?.last_error ? "error" : "ok"} />
          </div>
          <dl className="debug-values">
            <div><dt>Topic</dt><dd>{mission?.source_topic ?? "/mission/status"}</dd></div>
            <div><dt>Message age</dt><dd>{topic?.age_seconds?.toFixed(2) ?? "-"} s</dd></div>
            <div><dt>Messages</dt><dd>{topic?.message_count?.toLocaleString() ?? "0"}</dd></div>
          </dl>
          {mission?.last_error && <p className="panel-error-copy">{mission.last_error}</p>}
        </article>
      </div>
    </section>
  );
}
