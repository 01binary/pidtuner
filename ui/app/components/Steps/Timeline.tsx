"use client";

import { FC, useCallback, useState, useEffect, useRef } from "react";
import { VAxis } from "./VAxis";
import { Step } from "./Step";
import { Playhead } from "./Playhead";
import styles from "./Timeline.module.css";

type PerformanceStep = {
  value: number;
};

type TimelineProps = {
  steps: PerformanceStep[];
  currentStep: number;
  isPlaying: boolean;
  start: number;
  time: number;
  grid: number;
  onSelect: (step: number) => void;
  onStepChange: (step: number, value: number) => void;
};

const getPosition = (
  start: number,
  time: number,
  grid: number
) => (
  (time - start) / grid
);

export const Timeline: FC<TimelineProps> = ({
  steps,
  start,
  time,
  grid,
  currentStep,
  isPlaying,
  onStepChange,
  onSelect
}) => {
  const timelineRef = useRef<HTMLDivElement>(null);
  const [timelineTop, setTimelineTop] = useState(0);

  useEffect(() => {
    if (timelineRef.current) {
      setTimelineTop(timelineRef.current?.getBoundingClientRect()?.top ?? 0);
    }
  }, []);

  const handleValuePreset = useCallback((value: number) => {
    if (isPlaying) return;
    onStepChange(currentStep, value);
  }, [onStepChange, currentStep, isPlaying]);

  return (
    <section className={styles.timeline}>
      <VAxis
        onSetValue={handleValuePreset}
      />

      <div className={styles.scroll}>
        <div
          ref={timelineRef}
          className={styles.view}
        >
          {steps.map(({ value }, index) => (
            <Step
              key={index}
              value={value}
              prev={index > 0 ? steps[index - 1].value : 0}
              next={index < steps.length - 1 ? steps[index + 1].value : 0}
              timelineTop={timelineTop}
              isCurrentStep={index === currentStep}
              isFirstStep={index === 0}
              isLastStep={index === steps.length - 1}
              isReadOnly={isPlaying}
              onSelect={() => onSelect(index)}
              onChange={(value) => onStepChange(index, value)}
            />
          ))}
          <Playhead
            position={getPosition(start, time, grid)}
          />
        </div>
      </div>
    </section>
  );
};
