export type PageId =
  | "overview"
  | "navigation"
  | "hardware"
  | "vision"
  | "mission"
  | "ros-logs";

const primaryItems: Array<{ id: PageId; label: string; icon: string }> = [
  { id: "overview", label: "Overview", icon: "?" },
  { id: "navigation", label: "Navigation", icon: "?" },
  { id: "hardware", label: "Drive & Hardware", icon: "?" },
];

const systemItems: Array<{ id: PageId; label: string; icon: string }> = [
  { id: "vision", label: "Vision Pipeline", icon: "?" },
  { id: "mission", label: "Mission State", icon: "?" },
  { id: "ros-logs", label: "ROS Graph & Logs", icon: "?" },
];

interface SidebarProps {
  activePage: PageId;
  onChange: (page: PageId) => void;
}

export function Sidebar({ activePage, onChange }: SidebarProps) {
  return (
    <aside className="sidebar">
      <div className="window-dots"><i /><i /><i /></div>
      <div className="brand">
        <span className="brand__mark">?</span>
        <div>
          <strong>Robot Console</strong>
          <span>Developer UI v0.1.0</span>
        </div>
      </div>
      <nav className="sidebar__nav">
        {primaryItems.map((item) => (
          <button
            className={item.id === activePage ? "nav-item nav-item--active" : "nav-item"}
            key={item.id}
            onClick={() => onChange(item.id)}
            type="button"
          >
            <span className="nav-item__icon">{item.icon}</span>
            {item.label}
          </button>
        ))}
        <span className="nav-section-title">SYSTEM</span>
        {systemItems.map((item) => (
          <button
            className={item.id === activePage ? "nav-item nav-item--active" : "nav-item"}
            key={item.id}
            onClick={() => onChange(item.id)}
            type="button"
          >
            <span className="nav-item__icon">{item.icon}</span>
            {item.label}
          </button>
        ))}
      </nav>
      <div className="sidebar__footer">
        <span>READ-ONLY MODE</span>
        <small>Controls remain locked</small>
      </div>
    </aside>
  );
}
