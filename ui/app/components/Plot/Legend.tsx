import { FC } from "react";
import styles from "./Plot.module.css";

type LegendItem = {
  key: string;
  color: string;
  label: string;
};

type LegendProps = {
  legend: LegendItem[];
};

export const Legend: FC<LegendProps> = ({
  legend
}) => (
  <section className={styles.legend}>
    {legend.map(({ key, color, label }) => (
      <div key={key} className={styles.legendItem}>
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
