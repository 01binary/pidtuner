import { FC } from "react";
import styles from "./Plot.module.css";

type LegendItem = {
  key: string;
  color: string;
  label: string;
};

type LegendProps = {
  legend: LegendItem[];
  enabled: { [key: string]: boolean | undefined };
  onEnable: (key: string, enabled: boolean) => void;
};

export const Legend: FC<LegendProps> = ({
  legend,
  enabled,
  onEnable
}) => (
  <section className={styles.legend}>
    {legend.map(({ key, color, label }) => (
      <div
        key={key}
        className={[
          styles.legendItem,
          enabled[key] && styles.enabled
        ].filter(Boolean).join(' ')}
        onClick={(e) => {
          e.preventDefault();
          e.stopPropagation();
          onEnable(key, !enabled[key])
        }}
      >
        <div
          className={styles.legendColor}
          style={{ backgroundColor: color }}
        />
        <div className={styles.legendText}>
          {label}
        </div>
      </div>
    ))}
  </section>
);
