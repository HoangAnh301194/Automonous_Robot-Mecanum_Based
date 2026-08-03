import { MetricCard } from "../components/MetricCard";
import { RosGraph } from "../components/RosGraph";
import { SlamMap } from "../components/SlamMap";
import type { DashboardState } from "../types/telemetry";

interface OverviewPageProps {
  state: DashboardState | null;
}

export function OverviewPage({ state }: OverviewPageProps) {
  const summary = state?.ros_graph?.summary ?? {};
  const events = (state?.events ?? []).slice().reverse().slice(0, 10);
  const map = state?.navigation.map;
  const odom = state?.navigation.odom;
  const scan = state?.navigation.scan;

  return (
    <section className="page-section">
      <div className="analytics-toolbar">
        <div className="segment-control">
          <button className="segment-control__active" type="button">Overview</button>
          <button type="button">Details</button>
        </div>
        <div className="range-control">
          <button className="range-control__active" type="button">Live</button>
          <button type="button">1m</button>
          <button type="button">5m</button>
          <button type="button">15m</button>
          <button type="button">1h</button>
        </div>
      </div>

      <div className="metric-grid metric-grid--five">
        <MetricCard
          title="ROS Nodes"
          value={`${summary.node_count ?? 0}`}
          detail={`${summary.topic_count ?? 0} discovered topics`}
        />
        <MetricCard
          title="SLAM Map"
          value={map?.width && map?.height ? `${map.width}×${map.height}` : "-"}
          detail={`${map?.resolution?.toFixed(3) ?? "-"} m resolution`}
        />
        <MetricCard
          title="Robot Pose"
          value={typeof odom?.x === "number" && typeof odom.y === "number" ? `${odom.x.toFixed(1)}, ${odom.y.toFixed(1)}` : "-"}
          detail={`${odom?.yaw?.toFixed(3) ?? "-"} rad yaw`}
        />
        <MetricCard
          title="LiDAR Nearest"
          value={`${scan?.nearest_range?.toFixed(2) ?? "-"} m`}
          detail={`${scan?.valid_point_count ?? 0} valid points`}
        />
        <MetricCard
          title="System Load"
          value={`${state?.system.cpu_percent?.toFixed(1) ?? "-"}%`}
          detail={`${state?.system.memory_percent?.toFixed(1) ?? "-"}% RAM`}
        />
      </div>

      <div className="overview-layout">
        <SlamMap navigation={state?.navigation} />
        <div className="overview-side-stack">
          <RosGraph compact graph={state?.ros_graph} topics={state?.topics} />
          <aside className="recent-panel recent-panel--overview">
          <div className="recent-panel__header">
            <strong>RECENT EVENTS</strong>
            <span>{events.length}</span>
          </div>
          <div className="recent-panel__columns"><span>Source</span><span>Level / Event</span></div>
          <div className="recent-panel__body">
            {events.length ? events.map((event) => (
              <div className="event-row" key={`${event.timestamp}-${event.message}`}>
                <span><i className={`event-dot event-dot--${event.level.toLowerCase()}`} />{event.source}</span>
                <div><strong>{event.level}</strong><small>{event.message}</small></div>
              </div>
            )) : <p className="empty-copy">No runtime events yet.</p>}
          </div>
          </aside>
        </div>
      </div>
    </section>
  );
}
