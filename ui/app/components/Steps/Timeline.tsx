import { FC } from "react";
import { VAxis } from "./VAxis";
import { Step } from "./Step";
import { Playhead } from "./Playhead";
import styles from "./Timeline.module.css";

const steps = [
  { from: 0, to: 0 },
  { from: -20, to: -20 },
  { from: 0, to: 0 },
  { from: 20, to: 20 },
  { from: 5, to: 5 },
  { from: 0, to: 0 },
  { from: 0, to: 0 },
  { from: 0, to: 0 },
  { from: 0, to: 0 },
  { from: 0, to: 0 }
];

type TimelineProps = {
  step: number;
  position: number;
  isPlaying: boolean;
};

export const Timeline: FC<TimelineProps> = ({
  step,
  position,
  isPlaying
}) => {
  return (
    <section className={styles.timeline}>
      <VAxis className={styles.vaxis} />
      <div className={styles.scroll}>
        <div className={styles.view}>
          {steps.map(({ from, to }, index) => (
            <Step
              key={index}
              from={from}
              to={to}
              isCurrentStep={index === step}
              isReadOnly={isPlaying}
            />
          ))}
          <Playhead
            position={position}
          />
        </div>
      </div>
    </section>
  );
};
