import { FC } from "react";
import { VAxis } from "./VAxis";
import { Step } from "./Step";
import { Playhead } from "./Playhead";
import styles from "./Performer.module.css";

const steps = [
  { from: 0, to: 0 },
  { from: -20, to: -20 },
  { from: 0, to: 0 },
  { from: 20, to: 20 },
  { from: 5, to: 5 }
];

type PerformerProps = {
  step: number;
  position: number;
  isPlaying: boolean;
};

export const Performer: FC<PerformerProps> = ({
  step,
  position,
  isPlaying
}) => {
  return (
    <section className={styles.performer}>
      <VAxis />
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
    </section>
  );
};
