import { FC, useState, useEffect, useMemo, useRef } from "react";
import { inter } from "../../inter";
import styles from "./Gauge.module.css";

const DIV = 10
const MAX_STEP = 100000

const getStepSize = (range: number) => {
  for (let step = 1; step < MAX_STEP; step = step * DIV) {
    if ((range / step) <= DIV) {
      return step / DIV;
    }
  }

  return MAX_STEP;
};

const translation = (x: number, y: number) => (
  `matrix(1 0 0 1 ${x} ${y})`
);

const formatLabel = (value: number, step: number) => (
  step < 1 ? value.toFixed(1) : value.toString()
)

type GaugeType = {
  value: number;
  label: string;
}

export const Gauge: FC<GaugeType> = ({
  value,
  label
}) => {
  const [min, setMin] = useState(value);
  const [max, setMax] = useState(value);
  const [range, setRange] = useState(1);
  const [step, setStep] = useState(1);

  useEffect(() => {
    const newMin = Math.min(min, value);
    const newMax = Math.max(max, value);
    setRange(newMax - newMin);
    setStep(getStepSize(newMax - newMin))
    setMin(newMin);
    setMax(newMax);
  }, [value, min, max]);

  const stepCount = Math.ceil(range / step);
  const steps = [...Array(stepCount).keys()]
    .map(stepIndex => stepIndex * step);

  const offset = 18 - value / step * 18 - 18/2;

  return (
    <div className={styles.gauge}>
      <div className={styles.gaugeLabel}>
        {label}
      </div>
      <svg
        width="66px"
        height="144px"
        viewBox="0 0 66 144"
      >
        <path
          id="border"
          fill="#D3D3D3"
          d="M60.4,0H5.6C2.5,0,0,2.5,0,5.6v132.2c0,3.1,2.5,5.6,5.6,5.6h54.8c3.1,0,5.6-2.5,5.6-5.6V5.6
            C66.1,2.5,63.5,0,60.4,0z M62.5,134.5c0,3.1-2.5,5.6-5.6,5.6H9.2c-3.1,0-5.6-2.5-5.6-5.6V9c0-3.1,2.5-5.6,5.6-5.6h47.6
            c3.1,0,5.6,2.5,5.6,5.6V134.5z"
        />
        <line
          id="scale"
          fill="none"
          stroke="#424242"
          strokeMiterlimit="10"
          x1="49.9"
          y1="3.3"
          x2="49.9"
          y2="140.1"
        />
        <g style={{ transition: 'transform 0.3s ease-in-out' }} transform={translation(0, offset)}>
          {steps.map((stepValue, stepIndex) => (
            <text
              key={stepValue}
              x="12.9"
              y={22.7 * stepIndex}
              width="25.8"
              height="18"
              fontFamily={inter.style.fontFamily}
              fontSize="10.5px"
            >
              {formatLabel(stepValue, step)}
            </text>
          ))}
        </g>
        <polygon
          id="arrow"
          fill="#FFFFFF"
          stroke="#000000"
          strokeLinecap="round"
          strokeLinejoin="round"
          points="6.4,80 19.3,71.7 6.4,63.5 0.5,63.5 0.5,80"
        />
      </svg>
      <div className={styles.gaugeValue}>
        {value}
      </div>
    </div>
  )
};