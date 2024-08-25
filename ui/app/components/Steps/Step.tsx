import { FC, useCallback, useRef } from "react";
import styles from "./Timeline.module.css";

const MIN = 6.5;

type StepProps = {
  from: number;
  to: number;
  prev: number;
  next: number;
  isCurrentStep: boolean;
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
  isReadOnly,
  onSelect,
  onChange
}) => {
  const stepRef = useRef<HTMLDivElement>(null);
  const isDraggingRef = useRef(false);

  const height = stepRef.current
    ?.getBoundingClientRect()?.height ?? 0;

  const half = height / 2;

  const handleMouseDown = useCallback((e) => {
    e.preventDefault();
    e.stopPropagation();

    isDraggingRef.current = true;

    onSelect();
  }, [onSelect]);

  const handleMouseMove = useCallback((e) => {
    e.preventDefault();
    e.stopPropagation();

    if (isDraggingRef.current && stepRef.current) {
      const { clientY } = e;
      const { top, height } = stepRef.current.getBoundingClientRect();
      const norm = (clientY - top + MIN) / height;

      let value = norm * 2 - 1;
      
      if (value < -1)
        value = -1;
      else if (value > 1)
        value = 1;

      onChange(value, value);
    }
  }, [onChange]);

  const handleMouseUp = useCallback((e) => {
    e.preventDefault();
    e.stopPropagation();
    isDraggingRef.current = false;
  }, []);

  return (
    <div
      ref={stepRef}
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
        width="145.4px"
        height="260.9px"
        viewBox="0 0 145.4 260.9"
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
          y1={half}
          x2="145.4"
          y2={half}
        />

        <line
          stroke="#DDD"
          fill="none"
          x1="145"
          y1={0}
          x2="145"
          y2={height}
        />

        <g
          id="handleLeft"
          className={styles.controlPoint}
          style={{ visibility: isCurrentStep ? 'visible' : 'hidden' }}
        >
          <rect
            x="0"
            y={from * half + half - 6.5}
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
            y={to * half + half - 6.5}
            width="13"
            height="13"
            fill="white"
          />
        </g>

        <line
          className={styles.value}
          fill="none"
          strokeMiterlimit="10"
          x1="0"
          y1={from * half + half}
          x2="145.4"
          y2={to * half + half}
        />

        <line
          className={styles.value}
          fill="none"
          x1="0"
          x2="0"
          y1={from * half + half}
          y2={prev * half + half}
        />

        <line
          className={styles.value}
          fill="none"
          x1="145.4"
          x2="145.4"
          y1={from * half + half}
          y2={next * half + half}
        />
      </svg>
    </div>
  );
};
