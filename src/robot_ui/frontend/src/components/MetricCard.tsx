interface MetricCardProps {
  title: string;
  value: string;
  detail?: string;
}

export function MetricCard({ title, value, detail }: MetricCardProps) {
  return (
    <article className="metric-card">
      <span className="metric-card__title">{title}</span>
      <strong className="metric-card__value">{value}</strong>
      {detail && <span className="metric-card__detail">{detail}</span>}
    </article>
  );
}
