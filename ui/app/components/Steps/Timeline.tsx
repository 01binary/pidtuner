import { FC } from "react";
import { VAxis } from "./VAxis";
import { Step } from "./Step";
import { Playhead } from "./Playhead";
import styles from "./Timeline.module.css";

type PerformanceStep = {
  from: number;
  to: number;
};

type TimelineProps = {
  steps: PerformanceStep[];
  currentStep: number;
  isPlaying: boolean;
  start: number;
  time: number;
  onSelect: (step: number) => void;
  onStepChange: (step: number, from: number, to: number) => void;
};

export const Timeline: FC<TimelineProps> = ({
  steps,
  start,
  time,
  currentStep,
  isPlaying,
  onStepChange,
  onSelect
}) => {
  return (
    <section className={styles.timeline}>
      <VAxis />
      <div className={styles.scroll}>
        <div className={styles.view}>
          {steps.map(({ from, to }, index) => (
            <Step
              key={index}
              from={from}
              to={to}
              isCurrentStep={index === currentStep}
              isReadOnly={isPlaying}
              onSelect={() => onSelect(index)}
              onChange={(from, to) => onStepChange(index, from, to)}
            />
          ))}
          <Playhead position={currentStep} />
        </div>
      </div>
    </section>
  );
};
