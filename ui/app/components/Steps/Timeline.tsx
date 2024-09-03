"use client";

import { FC, useCallback, useState, useEffect, useRef } from "react";
import { VAxis } from "./VAxis";
import { HAxis } from "./HAxis";
import { Step } from "./Step";
import { Playhead } from "./Playhead";
import { StepType } from "./StepType";
import styles from "./Timeline.module.css";

type TimelineProps = {
  steps: StepType[];
  currentStep: number;
  isPlaying: boolean;
  start: number;
  time: number;
  grid: number;
  onSelect: (step: number) => void;
  onStepChange: (step: number, value: number) => void;
  onAddStep: () => void;
  onRemoveStep: () => void;
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
  onSelect,
  onAddStep,
  onRemoveStep
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

          <HAxis
            length={steps.length}
            grid={grid}
          />

          <div className={styles.manageButtons}>
            <button
              id="addButton"
              title="Add step"
              className={styles.addRemoveButton}
              onClick={onAddStep}
            >
              <svg width="48px" height="48px" viewBox="0 0 48 48">
                <polyline
                  fill="none"
                  stroke="#000000"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  points="30.9,12.5 19,12.5 19,24.4 5.8,24.4"
                />
                <path
                  fill="none"
                  stroke="white"
                  d="M34.2,39.4c0,2.6-2.1,4.7-4.7,4.7H10.5c-2.6,0-4.7-2.1-4.7-4.7V9.3
                    c0-2.6,2.1-4.7,4.7-4.7h19.1c2.6,0,4.7,2.1,4.7,4.7"
                />
                <line
                  stroke="#376BE8"
                  strokeWidth="1.5"
                  x1="25.6" y1="24.4" x2="42.8" y2="24.4"
                />
                <line
                  stroke="#376BE8"
                  strokeWidth="1.5"
                  x1="34.2"
                  y1="33"
                  x2="34.2"
                  y2="15.8"
                />
              </svg>
            </button>

            <button
              id="removeButton"
              title="Delete last step"
              className={styles.addRemoveButton}
              onClick={onRemoveStep}
            >
              <svg
                width="48px"
                height="48px"
                viewBox="0 0 48 48"
              >
                <polyline
                  fill="none"
                  stroke="#000000"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  points="31.9,12.5 20,12.5 20,24.4 6.8,24.4"
                />
                <path
                  fill="none"
                  stroke="white"
                  d="M35.2,39.4c0,2.6-2.1,4.7-4.7,4.7H11.5c-2.6,0-4.7-2.1-4.7-4.7V9.3
                    c0-2.6,2.1-4.7,4.7-4.7h19.1c2.6,0,4.7,2.1,4.7,4.7"
                />
                <line
                  stroke="#EC008C"
                  strokeWidth="1.5"
                  x1="29.1"
                  y1="18.3"
                  x2="41.3"
                  y2="30.5"
                />
                <line
                  stroke="#EC008C"
                  strokeWidth="1.5"
                  x1="29.1"
                  y1="30.5"
                  x2="41.3"
                  y2="18.3"
                />
              </svg>
            </button>
          </div>
        </div>
      </div>
    </section>
  );
};
