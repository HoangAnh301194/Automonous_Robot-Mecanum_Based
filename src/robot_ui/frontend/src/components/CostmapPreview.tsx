import { useEffect, useRef } from "react";

import type { SlamMapState } from "../types/telemetry";

interface CostmapPreviewProps {
  title: string;
  grid?: SlamMapState;
  tone: "blue" | "orange";
}

export function CostmapPreview({ title, grid, tone }: CostmapPreviewProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const hasGrid = Boolean(
    grid?.cells?.length && grid.preview_width && grid.preview_height,
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

      const previewWidth = grid?.preview_width ?? 0;
      const previewHeight = grid?.preview_height ?? 0;
      const cells = grid?.cells ?? [];
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
          image.data[pixelIndex] = 36;
          image.data[pixelIndex + 1] = 38;
          image.data[pixelIndex + 2] = 37;
        } else if (occupancy === 0) {
          image.data[pixelIndex] = 24;
          image.data[pixelIndex + 1] = 26;
          image.data[pixelIndex + 2] = 25;
        } else if (tone === "blue") {
          image.data[pixelIndex] = 38 + Math.round(occupancy * 0.3);
          image.data[pixelIndex + 1] = 74 + Math.round(occupancy * 0.45);
          image.data[pixelIndex + 2] = 112 + Math.round(occupancy * 1.15);
        } else {
          image.data[pixelIndex] = 120 + Math.round(occupancy * 1.3);
          image.data[pixelIndex + 1] = 54 + Math.round(occupancy * 0.45);
          image.data[pixelIndex + 2] = 34 + Math.round(occupancy * 0.2);
        }
        image.data[pixelIndex + 3] = 255;
      }
      rasterContext.putImageData(image, 0, 0);

      const padding = 14;
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
      context.translate(offsetX, offsetY + drawHeight);
      context.scale(drawWidth / previewWidth, -drawHeight / previewHeight);
      context.drawImage(raster, 0, 0);
      context.restore();
    };

    draw();
    const observer = new ResizeObserver(draw);
    observer.observe(canvas);
    return () => observer.disconnect();
  }, [grid, tone]);

  return (
    <article className={`costmap-preview costmap-preview--${tone}`}>
      <header>
        <div><strong>{title}</strong><span>{grid?.frame_id ?? "No frame"}</span></div>
        <i className={hasGrid ? "costmap-preview__live" : ""}>{hasGrid ? "LIVE" : "WAIT"}</i>
      </header>
      <div className="costmap-preview__canvas">
        <canvas ref={canvasRef} aria-label={`${title} occupancy grid`} />
        {!hasGrid && <span>Waiting for costmap...</span>}
      </div>
      <footer>
        <span>{grid?.width ?? 0} × {grid?.height ?? 0}</span>
        <span>{grid?.resolution?.toFixed(3) ?? "-"} m</span>
        <span>1:{grid?.sample_step ?? 1}</span>
      </footer>
    </article>
  );
}
