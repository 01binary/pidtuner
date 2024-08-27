import { FC, useEffect, useRef, useState } from "react";
import styles from "./Timeline.module.css";

type HAxisProps = {
  length: number;
  grid: number;
};

export const HAxis: FC<HAxisProps> = ({
  length,
  grid
}) => {
  const hAxisRef = useRef<HTMLDivElement>(null);
  const [ticks, setTicks] = useState<number[]>([]);
  
  useEffect(() => {
    if (hAxisRef.current) {
      setTicks([...new Array(length + 1)].map((e, i) => i))
    }
  }, [length]);

  return (
    <div ref={hAxisRef} className={styles.haxis}>
      {ticks.map((value) => (
        <div key={value} className={styles.haxisTick}>
          <div className={styles.haxisTickLabel}>
            {value === 0 ? '' : (value * grid).toFixed(1)}
          </div>
        </div>
      ))}
    </div>
  );
};
