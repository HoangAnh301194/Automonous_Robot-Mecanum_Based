import { useEffect, useState } from "react";

import type {
  ConnectionState,
  DashboardState,
  TelemetryEnvelope,
} from "../types/telemetry";
import { demoState } from "./demoState";

function websocketUrl(): string {
  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${protocol}//${window.location.host}/ws/telemetry`;
}

export function useTelemetry() {
  const [connection, setConnection] = useState<ConnectionState>("connecting");
  const [state, setState] = useState<DashboardState | null>(
    import.meta.env.DEV ? demoState : null,
  );

  useEffect(() => {
    let socket: WebSocket | null = null;
    let reconnectTimer: number | null = null;
    let disposed = false;

    const connect = () => {
      if (disposed) return;
      setConnection("connecting");
      socket = new WebSocket(websocketUrl());

      socket.onopen = () => setConnection("online");
      socket.onmessage = (event) => {
        const envelope = JSON.parse(event.data) as TelemetryEnvelope;
        setState(envelope.data);
      };
      socket.onerror = () => socket?.close();
      socket.onclose = () => {
        setConnection("offline");
        if (!disposed) {
          reconnectTimer = window.setTimeout(connect, 2000);
        }
      };
    };

    connect();
    return () => {
      disposed = true;
      if (reconnectTimer !== null) window.clearTimeout(reconnectTimer);
      socket?.close();
    };
  }, []);

  return { connection, state };
}
