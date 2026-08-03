import type { DashboardState } from "../types/telemetry";

interface HardwarePageProps {
  state: DashboardState | null;
}

export function HardwarePage({ state }: HardwarePageProps) {
  const hardware = state?.hardware as {
    battery?: { voltage?: number; temperature?: number; percentage?: number | null };
    encoders?: number[];
  };

  return (
    <section className="page-section">
      <div className="section-heading">
        <div>
          <span className="section-heading__eyebrow">DRIVE & HARDWARE</span>
          <h2>ESP32, encoder, battery, IMU</h2>
        </div>
      </div>
      <div className="panel-grid panel-grid--two">
        <article className="panel">
          <h3>Battery</h3>
          <p>Voltage: {hardware?.battery?.voltage?.toFixed(2) ?? "-"} V</p>
          <p>Temperature: {hardware?.battery?.temperature?.toFixed(1) ?? "-"} ?C</p>
          <p>Percentage: {hardware?.battery?.percentage ?? "Ch?a hi?u chu?n"}</p>
        </article>
        <article className="panel">
          <h3>Encoders</h3>
          <p>{hardware?.encoders?.join(" | ") ?? "?ang ch? /dataenc"}</p>
        </article>
        <article className="panel panel--wide">
          <h3>Motor telemetry chart</h3>
          <p>Desired/applied/measured motor commands c?n topic `/esp/telemetry`.</p>
        </article>
      </div>
    </section>
  );
}
