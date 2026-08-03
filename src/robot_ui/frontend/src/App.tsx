import { useState } from "react";

import { useTelemetry } from "./api/useTelemetry";
import { Sidebar, type PageId } from "./components/Sidebar";
import { StatusBadge } from "./components/StatusBadge";
import { HardwarePage } from "./pages/HardwarePage";
import { MissionPage } from "./pages/MissionPage";
import { NavigationPage } from "./pages/NavigationPage";
import { OverviewPage } from "./pages/OverviewPage";
import { RosLogsPage } from "./pages/RosLogsPage";
import { VisionPage } from "./pages/VisionPage";


export default function App() {
  const [activePage, setActivePage] = useState<PageId>("overview");
  const { connection, state } = useTelemetry();

  const page = (() => {
    switch (activePage) {
      case "navigation": return <NavigationPage state={state} />;
      case "hardware": return <HardwarePage state={state} />;
      case "vision": return <VisionPage state={state} />;
      case "mission": return <MissionPage state={state} />;
      case "ros-logs": return <RosLogsPage state={state} />;
      default: return <OverviewPage state={state} />;
    }
  })();

  const connectionTone = connection === "online" ? "ok" : connection === "connecting" ? "warn" : "error";
  const rosTone = state?.robot.ros_connected ? "ok" : "error";
  const pageMeta: Record<PageId, { title: string; subtitle: string; icon: string }> = {
    overview: { title: "Usage & Analytics", subtitle: "Monitor ROS activity, system resources, and runtime events", icon: "?" },
    navigation: { title: "Navigation Debug", subtitle: "Inspect map, localization, LiDAR, costmaps, and Nav2", icon: "?" },
    hardware: { title: "Drive & Hardware", subtitle: "Inspect ESP32, encoder, battery, motor, and IMU telemetry", icon: "?" },
    vision: { title: "Vision Pipeline", subtitle: "Inspect camera, YOLO, pose, and obstacle detection", icon: "?" },
    mission: { title: "Mission State", subtitle: "Inspect waypoint progress, interaction, and state transitions", icon: "?" },
    "ros-logs": { title: "ROS Graph & Logs", subtitle: "Inspect graph topology, topic health, and runtime events", icon: "?" },
  };
  const meta = pageMeta[activePage];

  return (
    <div className="app-shell">
      <Sidebar activePage={activePage} onChange={setActivePage} />
      <main className="main-content">
        <header className="topbar">
          <div className="topbar__title">
            <span>{meta.icon}</span>
            <div>
              <h1>{meta.title}</h1>
              <p>{meta.subtitle}</p>
            </div>
          </div>
          <div className="topbar__status">
            <StatusBadge label={`WEB ${connection.toUpperCase()}`} tone={connectionTone} />
            <StatusBadge label={state?.robot.ros_connected ? "ROS ONLINE" : "ROS OFFLINE"} tone={rosTone} />
            <button className="topbar-button" type="button">?</button>
            <button className="topbar-button topbar-button--text" type="button">VN</button>
            <button className="topbar-button" type="button">?</button>
          </div>
        </header>
        {page}
      </main>
    </div>
  );
}
