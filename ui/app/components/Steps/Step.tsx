import { FC, useCallback, useEffect, useRef } from "react";
import styles from "./Timeline.module.css";

const MIN = 6.5;
const HEIGHT = 261;
const HALF = HEIGHT / 2;

const getValueFromMouse = (
  clientY: number,
  timelineTop: number
) => {
  const norm = (clientY - timelineTop - MIN) / HEIGHT;

  let value = norm * 2 - 1;

  if (value < -1)
    value = -1;
  else if (value > 1)
    value = 1;

  return -value;
};

type StepProps = {
  value: number;
  prev: number;
  next: number;
  isCurrentStep: boolean;
  isFirstStep: boolean;
  isLastStep: boolean;
  isReadOnly: boolean;
  timelineTop: number;
  onSelect: () => void;
  onChange: (value: number) => void;
};

export const Step: FC<StepProps> = ({
  value,
  prev,
  next,
  isCurrentStep,
  isFirstStep,
  isLastStep,
  isReadOnly,
  timelineTop,
  onSelect,
  onChange
}) => {
  const isDraggingRef = useRef(false);

  const handleMouseDown = useCallback((e) => {
    if (isReadOnly) return;

    e.preventDefault();
    e.stopPropagation();

    const { clientY } = e;
    const value = getValueFromMouse(clientY, timelineTop);

    onChange(value);
    isDraggingRef.current = true;

    onSelect();
  }, [onSelect, isReadOnly, timelineTop]);

  const handleMouseMove = useCallback((e) => {
    e.preventDefault();
    e.stopPropagation();

    if (isDraggingRef.current) {
      const { clientY } = e;
      const value = getValueFromMouse(clientY, timelineTop);
      onChange(value);
    }
  }, [onChange]);

  const handleMouseUp = useCallback((e) => {
    e.preventDefault();
    e.stopPropagation();
    isDraggingRef.current = false;
  }, []);

  useEffect(() => {
    document.addEventListener('mouseup', handleMouseUp, true);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp, true);
    }
  }, [handleMouseUp]);

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
        {value.toFixed(2)}
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
            visibility: value === 0 ? "hidden" : "visible"
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

        <line
          id="stepValue"
          className={styles.value}
          fill="none"
          x1="0"
          y1={value * -HALF + HALF}
          x2="145.5"
          y2={value * -HALF + HALF}
        />

        {!isFirstStep && <line
          id="stepUp"
          className={styles.value}
          fill="none"
          x1="0"
          x2="0"
          y1={value * -HALF + HALF}
          y2={prev * -HALF + HALF}
        />}

        {!isLastStep && <line
          id="stepDown"
          className={styles.value}
          fill="none"
          x1="145.4"
          x2="145.4"
          y1={value * -HALF + HALF}
          y2={next * -HALF + HALF}
        />}
      </svg>
    </div>
  );
};
