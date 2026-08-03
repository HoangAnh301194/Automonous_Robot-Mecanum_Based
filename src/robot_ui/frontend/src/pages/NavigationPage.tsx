import { CostmapPreview } from "../components/CostmapPreview";
import { SlamMap } from "../components/SlamMap";
import type { DashboardState } from "../types/telemetry";

interface NavigationPageProps {
  state: DashboardState | null;
}

function fixed(value: number | undefined, digits = 3): string {
  return typeof value === "number" ? value.toFixed(digits) : "-";
}

function timeLabel(timestamp: number | undefined): string {
  if (!timestamp) return "-";
  return new Date(timestamp * 1000).toLocaleTimeString();
}

export function NavigationPage({ state }: NavigationPageProps) {
  const navigation = state?.navigation;
  const pose = navigation?.pose ?? navigation?.odom;
  const nav2 = navigation?.nav2;
  const history = nav2?.history ?? [];
  const statusTone = nav2?.status === "SUCCEEDED"
    ? "ok"
    : nav2?.status === "ABORTED" || nav2?.status === "CANCELED"
      ? "error"
      : nav2?.status === "EXECUTING" || nav2?.status === "ACCEPTED"
        ? "live"
        : "idle";

  return (
    <section className="page-section">
      <div className="section-heading">
        <div>
          <span className="section-heading__eyebrow">NAVIGATION DEBUG</span>
          <h2>TF, LaserScan, Nav2 paths, goals, and costmaps</h2>
        </div>
        <span className="section-heading__hint">READ-ONLY TELEMETRY</span>
      </div>

      <SlamMap detailed navigation={navigation} />

      <div className="navigation-status-grid">
        <article className="navigation-status-card">
          <header><span>TF LOCALIZATION</span><i className={`nav-state nav-state--${navigation?.tf?.state === "OK" ? "ok" : "error"}`}>{navigation?.tf?.state ?? "WAIT"}</i></header>
          <strong>{navigation?.tf?.map_frame ?? "map"} → {navigation?.tf?.base_frame ?? "base_footprint"}</strong>
          <p>X {fixed(pose?.x)} · Y {fixed(pose?.y)} · Yaw {fixed(pose?.yaw)}</p>
          <small>{navigation?.tf?.error || `Last TF ${timeLabel(navigation?.tf?.last_success_at)}`}</small>
        </article>

        <article className="navigation-status-card">
          <header><span>LASERSCAN</span><i className={`nav-state nav-state--${navigation?.scan?.transform_state === "OK" ? "ok" : "error"}`}>{navigation?.scan?.transform_state ?? "WAIT"}</i></header>
          <strong>{navigation?.scan?.points_xy?.length ?? 0} rendered points</strong>
          <p>{navigation?.scan?.valid_point_count ?? 0} valid · {fixed(navigation?.scan?.nearest_range, 2)} m nearest</p>
          <small>{navigation?.scan?.frame_id ?? "scan"} → {navigation?.scan?.target_frame_id ?? "map"}</small>
        </article>

        <article className="navigation-status-card">
          <header><span>NAV2 ACTION</span><i className={`nav-state nav-state--${statusTone}`}>{nav2?.status ?? "IDLE"}</i></header>
          <strong>{nav2?.active_goal_count ?? 0} active goals</strong>
          <p>Tracked statuses: {nav2?.status_count ?? 0}</p>
          <small>Goal {nav2?.goal_id ? nav2.goal_id.slice(0, 12) : "-"}</small>
        </article>

        <article className="navigation-status-card">
          <header><span>CURRENT GOAL</span><i className="nav-state nav-state--idle">{navigation?.goal ? "SET" : "NONE"}</i></header>
          <strong>X {fixed(navigation?.goal?.x)} · Y {fixed(navigation?.goal?.y)}</strong>
          <p>Yaw {fixed(navigation?.goal?.yaw)} rad</p>
          <small>Frame {navigation?.goal?.frame_id ?? "-"}</small>
        </article>
      </div>

      <div className="costmap-grid">
        <CostmapPreview grid={navigation?.global_costmap} title="Global Costmap" tone="blue" />
        <CostmapPreview grid={navigation?.local_costmap} title="Local Costmap" tone="orange" />
      </div>

      <div className="navigation-detail-grid">
        <article className="panel navigation-path-panel">
          <h3>Path telemetry</h3>
          <div className="path-stat-row">
            <span><i className="path-line path-line--global" />Global path</span>
            <strong>{navigation?.global_path?.point_count ?? 0} points</strong>
            <small>{navigation?.global_path?.frame_id ?? "-"}</small>
          </div>
          <div className="path-stat-row">
            <span><i className="path-line path-line--local" />Local path</span>
            <strong>{navigation?.local_path?.point_count ?? 0} points</strong>
            <small>{navigation?.local_path?.frame_id ?? "-"}</small>
          </div>
          <div className="path-stat-row">
            <span><i className="path-line path-line--scan" />LaserScan</span>
            <strong>{navigation?.scan?.point_count ?? 0} raw points</strong>
            <small>{navigation?.scan?.points_xy?.length ?? 0} sampled</small>
          </div>
        </article>

        <article className="panel navigation-history-panel">
          <h3>Nav2 transition history</h3>
          <div className="nav-history-list">
            {history.length ? history.slice(0, 10).map((item, itemIndex) => (
              <div key={`${item.goal_id}-${item.status_code}-${itemIndex}`}>
                <i className={`nav-history-dot nav-history-dot--${item.status?.toLowerCase() ?? "unknown"}`} />
                <span><strong>{item.status ?? "UNKNOWN"}</strong><small>{item.goal_id?.slice(0, 12) ?? "-"}</small></span>
                <time>{timeLabel(item.transition_at)}</time>
              </div>
            )) : <p className="empty-copy">No Nav2 status transitions yet.</p>}
          </div>
        </article>
      </div>
    </section>
  );
}
