import { MetricCard } from "../components/MetricCard";
import { StatusBadge } from "../components/StatusBadge";
import type { DashboardState, TopicState } from "../types/telemetry";

interface VisionPageProps {
  state: DashboardState | null;
}

function statusTone(status?: string): "ok" | "warn" | "error" | "neutral" {
  if (status === "OK") return "ok";
  if (status === "WARN" || status === "WAITING") return "warn";
  if (status === "STALE" || status === "ERROR") return "error";
  return "neutral";
}

function PipelineStage({ title, topic, detail }: { title: string; topic?: TopicState; detail: string }) {
  return (
    <article className={`pipeline-stage pipeline-stage--${(topic?.state ?? "waiting").toLowerCase()}`}>
      <div><strong>{title}</strong><small>{detail}</small></div>
      <StatusBadge label={topic?.state ?? "WAITING"} tone={statusTone(topic?.state)} />
    </article>
  );
}

export function VisionPage({ state }: VisionPageProps) {
  const vision = state?.vision;
  const topics = state?.topics ?? {};
  const colorTopic = topics["/camera/color/camera_info"];
  const depthTopic = topics["/camera/depth/camera_info"];
  const yoloTopic = topics["/yolo/detections"];
  const obstacleTopic = topics["/scan_obstacles"];
  const waveTopic = topics["/pose/wave_detected"];
  const detections = vision?.yolo?.detections ?? [];

  return (
    <section className="page-section">
      <div className="section-heading">
        <div>
          <span className="section-heading__eyebrow">VISION DEBUG</span>
          <h2>Camera, YOLO, pose, and depth-obstacle pipeline health</h2>
        </div>
      </div>

      <div className="metric-grid metric-grid--five">
        <MetricCard title="Color Camera" value={colorTopic?.state ?? "WAITING"} detail={vision?.color_camera?.width ? `${vision.color_camera.width}×${vision.color_camera.height}` : "No CameraInfo"} />
        <MetricCard title="Depth Camera" value={depthTopic?.state ?? "WAITING"} detail={vision?.depth_camera?.width ? `${vision.depth_camera.width}×${vision.depth_camera.height}` : "No CameraInfo"} />
        <MetricCard title="YOLO Rate" value={`${yoloTopic?.rate_hz?.toFixed(1) ?? "-"} Hz`} detail={`${yoloTopic?.expected_rate_hz?.toFixed(1) ?? "-"} Hz expected`} />
        <MetricCard title="Detections" value={`${vision?.yolo?.detection_count ?? 0}`} detail={`${vision?.yolo?.person_count ?? 0} persons · ${vision?.yolo?.tracked_count ?? 0} tracked`} />
        <MetricCard title="Nearest Obstacle" value={`${vision?.obstacle?.nearest_range?.toFixed(2) ?? "-"} m`} detail={`${vision?.obstacle?.obstacle_point_count ?? 0} obstacle points`} />
      </div>

      <div className="vision-pipeline-flow">
        <PipelineStage title="Color Camera" topic={colorTopic} detail={vision?.color_camera?.frame_id || "/camera/color/camera_info"} />
        <span className="pipeline-connector">YOLO</span>
        <PipelineStage title="YOLO Detection" topic={yoloTopic} detail={`${vision?.yolo?.detection_count ?? 0} current objects`} />
        <span className="pipeline-connector">POSE</span>
        <PipelineStage title="Wave Detection" topic={waveTopic} detail={vision?.pose?.status || "Event-driven output"} />
        <PipelineStage title="Depth Camera" topic={depthTopic} detail={vision?.depth_camera?.frame_id || "/camera/depth/camera_info"} />
        <span className="pipeline-connector">DEPTH</span>
        <PipelineStage title="Obstacle Detector" topic={obstacleTopic} detail={`${vision?.obstacle?.obstacle_point_count ?? 0} valid scan points`} />
      </div>

      <div className="panel-grid panel-grid--two">
        <article className="panel panel--wide">
          <div className="panel__heading">
            <div><h3>YOLO detections</h3><span>Highest-confidence objects from latest frame</span></div>
            <StatusBadge label={`${detections.length} SHOWN`} tone={detections.length ? "ok" : "neutral"} />
          </div>
          <div className="detection-table">
            <div className="detection-row detection-row--header"><span>Class</span><span>Score</span><span>Track ID</span><span>Center</span><span>Size</span><span>Keypoints</span></div>
            {detections.map((detection, index) => (
              <div className="detection-row" key={`${detection.id ?? "detection"}-${index}`}>
                <strong>{detection.class_name || (detection.class_id !== undefined ? detection.class_id : "unknown")}</strong>
                <span>{((detection.score ?? 0) * 100).toFixed(1)}%</span>
                <span>{detection.id || "-"}</span>
                <span>{detection.center_x?.toFixed(0) ?? "-"}, {detection.center_y?.toFixed(0) ?? "-"}</span>
                <span>{detection.width?.toFixed(0) ?? "-"}×{detection.height?.toFixed(0) ?? "-"}</span>
                <span>{detection.keypoint_count ?? 0}</span>
              </div>
            ))}
            {!detections.length && <p className="empty-copy">Waiting for `/yolo/detections`.</p>}
          </div>
        </article>

        <article className="panel">
          <div className="panel__heading">
            <div><h3>Pose and wave</h3><span>Event-driven pose output</span></div>
            <StatusBadge label={vision?.pose?.wave_detected ? "WAVING" : "CLEAR"} tone={vision?.pose?.wave_detected ? "warn" : "ok"} />
          </div>
          <dl className="debug-values">
            <div><dt>Detected</dt><dd>{vision?.pose?.wave_detected ? "Yes" : "No"}</dd></div>
            <div><dt>Status</dt><dd>{vision?.pose?.status || "-"}</dd></div>
            <div><dt>Topic rate</dt><dd>{waveTopic?.rate_hz?.toFixed(1) ?? "event"} Hz</dd></div>
          </dl>
        </article>

        <article className="panel">
          <div className="panel__heading">
            <div><h3>Depth obstacle</h3><span>LaserScan generated from depth image</span></div>
            <StatusBadge label={obstacleTopic?.state ?? "WAITING"} tone={statusTone(obstacleTopic?.state)} />
          </div>
          <dl className="debug-values">
            <div><dt>Frame</dt><dd>{vision?.obstacle?.frame_id || "-"}</dd></div>
            <div><dt>Nearest</dt><dd>{vision?.obstacle?.nearest_range?.toFixed(2) ?? "-"} m</dd></div>
            <div><dt>Obstacle points</dt><dd>{vision?.obstacle?.obstacle_point_count ?? 0} / {vision?.obstacle?.point_count ?? 0}</dd></div>
            <div><dt>Range</dt><dd>{vision?.obstacle?.range_min?.toFixed(1) ?? "-"}–{vision?.obstacle?.range_max?.toFixed(1) ?? "-"} m</dd></div>
          </dl>
        </article>
      </div>
    </section>
  );
}
