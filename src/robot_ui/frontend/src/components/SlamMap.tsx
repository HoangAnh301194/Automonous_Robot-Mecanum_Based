import { useEffect, useRef } from "react";

import type { NavigationState } from "../types/telemetry";

interface SlamMapProps {
  navigation?: NavigationState;
  detailed?: boolean;
}

function fixed(value: number | undefined, digits = 2): string {
  return typeof value === "number" ? value.toFixed(digits) : "-";
}

export function SlamMap({ navigation, detailed = false }: SlamMapProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const map = navigation?.map;
  const pose = navigation?.pose ?? navigation?.odom;
  const scan = navigation?.scan;
  const hasMap = Boolean(
    map?.cells?.length && map.preview_width && map.preview_height,
  );

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const draw = () => {
      const bounds = canvas.getBoundingClientRect();
      if (!bounds.width || !bounds.height) return;

      const pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
      canvas.width = Math.round(bounds.width * pixelRatio);
      canvas.height = Math.round(bounds.height * pixelRatio);

      const context = canvas.getContext("2d");
      if (!context) return;

      context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
      context.fillStyle = "#151615";
      context.fillRect(0, 0, bounds.width, bounds.height);

      const previewWidth = map?.preview_width ?? 0;
      const previewHeight = map?.preview_height ?? 0;
      const cells = map?.cells ?? [];
      if (!previewWidth || !previewHeight || !cells.length) return;

      const raster = document.createElement("canvas");
      raster.width = previewWidth;
      raster.height = previewHeight;
      const rasterContext = raster.getContext("2d");
      if (!rasterContext) return;

      const image = rasterContext.createImageData(previewWidth, previewHeight);
      const cellCount = Math.min(cells.length, previewWidth * previewHeight);
      for (let cellIndex = 0; cellIndex < cellCount; cellIndex += 1) {
        const occupancy = cells[cellIndex];
        const pixelIndex = cellIndex * 4;
        if (occupancy < 0) {
          image.data[pixelIndex] = 47;
          image.data[pixelIndex + 1] = 51;
          image.data[pixelIndex + 2] = 49;
        } else if (occupancy >= 50) {
          image.data[pixelIndex] = 24;
          image.data[pixelIndex + 1] = 25;
          image.data[pixelIndex + 2] = 24;
        } else {
          const shade = Math.max(155, 224 - Math.round(occupancy * 1.2));
          image.data[pixelIndex] = shade;
          image.data[pixelIndex + 1] = shade + 2;
          image.data[pixelIndex + 2] = shade - 2;
        }
        image.data[pixelIndex + 3] = 255;
      }
      rasterContext.putImageData(image, 0, 0);

      const padding = 28;
      const scale = Math.min(
        (bounds.width - padding * 2) / previewWidth,
        (bounds.height - padding * 2) / previewHeight,
      );
      const drawWidth = previewWidth * scale;
      const drawHeight = previewHeight * scale;
      const offsetX = (bounds.width - drawWidth) / 2;
      const offsetY = (bounds.height - drawHeight) / 2;

      context.save();
      context.imageSmoothingEnabled = false;
      context.shadowColor = "rgba(0, 0, 0, .5)";
      context.shadowBlur = 24;
      context.translate(offsetX, offsetY + drawHeight);
      context.scale(drawWidth / previewWidth, -drawHeight / previewHeight);
      context.drawImage(raster, 0, 0);
      context.restore();

      context.strokeStyle = "rgba(255, 255, 255, .14)";
      context.lineWidth = 1;
      context.strokeRect(offsetX, offsetY, drawWidth, drawHeight);

      const resolution = map?.resolution;
      if (typeof resolution !== "number" || resolution <= 0) return;
      const originX = map?.origin_x ?? 0;
      const originY = map?.origin_y ?? 0;
      const originYaw = map?.origin_yaw ?? 0;
      const sampleStep = map?.sample_step ?? 1;
      const cosine = Math.cos(originYaw);
      const sine = Math.sin(originYaw);
      const worldToCanvas = (worldX: number, worldY: number) => {
        const deltaX = worldX - originX;
        const deltaY = worldY - originY;
        const mapCellX = (cosine * deltaX + sine * deltaY) / resolution / sampleStep;
        const mapCellY = (-sine * deltaX + cosine * deltaY) / resolution / sampleStep;
        return {
          x: offsetX + mapCellX * (drawWidth / previewWidth),
          y: offsetY + drawHeight - mapCellY * (drawHeight / previewHeight),
        };
      };
      const drawPath = (
        points: Array<[number, number]> | undefined,
        color: string,
        lineWidth: number,
        dashed = false,
      ) => {
        if (!points || points.length < 2) return;
        context.save();
        context.strokeStyle = color;
        context.lineWidth = lineWidth;
        context.lineJoin = "round";
        context.lineCap = "round";
        context.setLineDash(dashed ? [6, 5] : []);
        context.beginPath();
        points.forEach(([worldX, worldY], pointIndex) => {
          const point = worldToCanvas(worldX, worldY);
          if (pointIndex === 0) context.moveTo(point.x, point.y);
          else context.lineTo(point.x, point.y);
        });
        context.stroke();
        context.restore();
      };

      context.save();
      context.fillStyle = "rgba(73, 216, 194, .72)";
      for (const [worldX, worldY] of scan?.points_xy ?? []) {
        const point = worldToCanvas(worldX, worldY);
        context.fillRect(point.x - 1, point.y - 1, 2, 2);
      }
      context.restore();

      if (detailed) {
        drawPath(navigation?.global_path?.points_xy, "rgba(77, 141, 255, .95)", 2.4);
        drawPath(navigation?.local_path?.points_xy, "rgba(255, 197, 76, .95)", 2, true);

        if (
          typeof navigation?.goal?.x === "number" &&
          typeof navigation.goal.y === "number"
        ) {
          const goalPoint = worldToCanvas(navigation.goal.x, navigation.goal.y);
          context.save();
          context.translate(goalPoint.x, goalPoint.y);
          context.strokeStyle = "#d387ff";
          context.fillStyle = "rgba(211, 135, 255, .16)";
          context.lineWidth = 2;
          context.beginPath();
          context.arc(0, 0, 10, 0, Math.PI * 2);
          context.fill();
          context.stroke();
          context.beginPath();
          context.moveTo(-15, 0);
          context.lineTo(15, 0);
          context.moveTo(0, -15);
          context.lineTo(0, 15);
          context.stroke();
          context.restore();
        }
      }

      if (typeof pose?.x === "number" && typeof pose.y === "number") {
        const robotPoint = worldToCanvas(pose.x, pose.y);
        if (
          robotPoint.x >= offsetX &&
          robotPoint.x <= offsetX + drawWidth &&
          robotPoint.y >= offsetY &&
          robotPoint.y <= offsetY + drawHeight
        ) {
          context.save();
          context.translate(robotPoint.x, robotPoint.y);
          context.rotate(-(pose.yaw ?? 0) + originYaw);
          context.shadowColor = "rgba(255, 112, 72, .8)";
          context.shadowBlur = 14;
          context.fillStyle = "#ff7048";
          context.beginPath();
          context.moveTo(14, 0);
          context.lineTo(-9, -8);
          context.lineTo(-5, 0);
          context.lineTo(-9, 8);
          context.closePath();
          context.fill();
          context.strokeStyle = "#fff0ea";
          context.lineWidth = 1.5;
          context.stroke();
          context.restore();
        }
      }
    };

    draw();
    const observer = new ResizeObserver(draw);
    observer.observe(canvas);
    return () => observer.disconnect();
  }, [detailed, map, navigation, pose, scan]);

  return (
    <article className={`slam-map${detailed ? " slam-map--detailed" : ""}`}>
      <header className="slam-map__header">
        <div>
          <strong>SLAM Occupancy Map</strong>
          <span>{detailed ? "TF pose, LaserScan, Nav2 goal and path overlays" : "Live `/map` with robot pose overlay"}</span>
        </div>
        <span className={`slam-map__status${hasMap ? " slam-map__status--live" : ""}`}>
          {hasMap ? "LIVE" : "WAITING"}
        </span>
      </header>

      <div className="slam-map__canvas-wrap">
        <canvas ref={canvasRef} aria-label="SLAM occupancy map" />
        {!hasMap && <div className="slam-map__empty">Waiting for `/map` OccupancyGrid...</div>}
        <div className="slam-map__hud slam-map__hud--pose">
          <span>{navigation?.pose ? "TF POSE" : "ODOM POSE"}</span>
          <strong>X {fixed(pose?.x)} m</strong>
          <strong>Y {fixed(pose?.y)} m</strong>
          <small>Yaw {fixed(pose?.yaw, 3)} rad</small>
        </div>
        <div className="slam-map__hud slam-map__hud--scan">
          <span>LIDAR</span>
          <strong>{fixed(scan?.nearest_range)} m nearest</strong>
          <small>{scan?.valid_point_count ?? 0} valid points</small>
        </div>
        <div className="slam-map__legend">
          <span><i className="map-swatch map-swatch--free" />Free</span>
          <span><i className="map-swatch map-swatch--occupied" />Occupied</span>
          <span><i className="map-swatch map-swatch--unknown" />Unknown</span>
          <span><i className="map-swatch map-swatch--robot" />Robot</span>
          <span><i className="map-swatch map-swatch--scan" />Scan</span>
          {detailed && <>
            <span><i className="map-swatch map-swatch--global-path" />Global path</span>
            <span><i className="map-swatch map-swatch--local-path" />Local path</span>
            <span><i className="map-swatch map-swatch--goal" />Goal</span>
          </>}
        </div>
      </div>

      <footer className="slam-map__footer">
        <span>Frame <strong>{map?.frame_id ?? "-"}</strong></span>
        <span>Grid <strong>{map?.width ?? 0} × {map?.height ?? 0}</strong></span>
        <span>Resolution <strong>{fixed(map?.resolution, 3)} m</strong></span>
        <span>Preview <strong>1:{map?.sample_step ?? 1}</strong></span>
      </footer>
    </article>
  );
}
