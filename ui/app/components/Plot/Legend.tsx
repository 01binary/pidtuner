import { FC } from "react";
import styles from "./Plot.module.css";

type LegendItem = {
  color: string,
  label: string,
  samples: number[],
};

type LegendProps = {
  legend: LegendItem[]
};

export const Legend: FC<LegendProps> = ({ legend }) => (
  <section className={styles.legend}>
    {legend.map(({ color, label }) => (
      <div key={label} className={styles.legendItem}>
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
