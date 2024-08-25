import { FC, useCallback, useEffect, useState, useRef } from "react";
import styles from "./Timeline.module.css";

const MIN = 6.5;
const TOP = 235;
const HEIGHT = 261;
const HALF = HEIGHT / 2;

type StepProps = {
  from: number;
  to: number;
  prev: number;
  next: number;
  isCurrentStep: boolean;
  isFirstStep: boolean;
  isLastStep: boolean;
  isReadOnly: boolean;
  onSelect: () => void;
  onChange: (from: number, to: number) => void;
};

export const Step: FC<StepProps> = ({
  from,
  to,
  prev,
  next,
  isCurrentStep,
  isFirstStep,
  isLastStep,
  isReadOnly,
  onSelect,
  onChange
}) => {
  const isDraggingRef = useRef(false);

  const handleMouseDown = useCallback((e) => {
    e.preventDefault();
    e.stopPropagation();

    isDraggingRef.current = true;

    onSelect();
  }, [onSelect]);

  const handleMouseMove = useCallback((e) => {
    e.preventDefault();
    e.stopPropagation();

    if (isDraggingRef.current) {
      const { clientY } = e;
      const norm = (clientY - TOP + MIN) / HEIGHT;

      let value = norm * 2 - 1;
      
      if (value < -1)
        value = -1;
      else if (value > 1)
        value = 1;

      onChange(-value, -value);
    }
  }, [onChange]);

  const handleMouseUp = useCallback((e) => {
    e.preventDefault();
    e.stopPropagation();
    isDraggingRef.current = false;
  }, []);

  return (
    <div
      className={[
        styles.step,
        isCurrentStep && styles.current
      ].filter(Boolean).join(' ')}
      onMouseDown={handleMouseDown}
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
    >
      <div className={styles.stepLabel}>
        {(from !== to)
          ? `${from.toFixed(1)} -> ${to.toFixed(1)}`
          : from.toFixed(1)}
      </div>

      <svg
        width="145.5px"
        height="261px"
        viewBox="0 0 145.5 261"
      >
        <path
          className={styles.stepBorder}
          d="M140.7,260.9H4.7c-2.6,0-4.7-2.1-4.7-4.7V4.7C0,2.1,2.1,0,4.7,0h136.1c2.6,0,4.7,2.1,4.7,4.7v251.6
            C145.4,258.8,143.3,260.9,140.7,260.9z"
        />

        <line
          className={styles.stepCenter}
          style={{
            visibility: from === 0 && to === 0 ? "hidden" : "visible"
          }}
          fill="none"
          strokeMiterlimit="10"
          strokeDasharray="4,6"
          x1="0"
          y1={HALF}
          x2="145.4"
          y2={HALF}
        />

        <line
          stroke="#DDD"
          fill="none"
          x1="145"
          y1={0}
          x2="145"
          y2={HEIGHT}
        />

        <g
          id="handleLeft"
          className={styles.controlPoint}
          style={{ visibility: isCurrentStep ? 'visible' : 'hidden' }}
        >
          <rect
            x="0"
            y={-from * HALF + HALF - 6.5}
            width="13"
            height="13"
            fill="white"
          />
        </g>

        <g
          id="handleRight"
          className={styles.controlPoint}
          style={{ visibility: isCurrentStep ? 'visible' : 'hidden' }}
        >
          <rect
            x="132.5"
            y={-to * HALF + HALF - 6.5}
            width="13"
            height="13"
            fill="white"
          />
        </g>

        <line
          id="stepValue"
          className={styles.value}
          fill="none"
          x1="0"
          y1={from * -HALF + HALF}
          x2="145.5"
          y2={to * -HALF + HALF}
        />

        {!isFirstStep && <line
          id="stepUp"
          className={styles.value}
          fill="none"
          x1="0"
          x2="0"
          y1={from * -HALF + HALF}
          y2={prev * -HALF + HALF}
        />}

        {!isLastStep && <line
          id="stepDown"
          className={styles.value}
          fill="none"
          x1="145.4"
          x2="145.4"
          y1={from * -HALF + HALF}
          y2={next * -HALF + HALF}
        />}
      </svg>
    </div>
  );
};
