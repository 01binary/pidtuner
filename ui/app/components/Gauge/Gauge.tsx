import { FC, useState, useEffect } from "react";
import { inter } from "../../inter";
import styles from "./Gauge.module.css";

const DIV = 10;
const STEP_COUNT = 100;
const MAX_STEP = 100000;
const STEP_HEIGHT = 22;
const STEP_FONT_HEIGHT = 11;
const GAUGE_HEIGHT = 143;

const getOffsetFromStep = (stepIndex: number) => (
  stepIndex * STEP_HEIGHT
)

const getOffsetFromValue = (value: number, stepSize: number) => (
  GAUGE_HEIGHT / 2 - value / stepSize * STEP_HEIGHT + STEP_HEIGHT / 2
)

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

const formatStepLabel = (value: number, stepSize: number) => (
  stepSize < 1 ? value.toFixed(1) : Math.ceil(value).toString()
)

const formatLabel = (value: number) => {
  const initial = value.toString()
  const point = initial.indexOf('.')

  if (point >= 0 && initial.length - point > 1) {
    return value.toFixed(1);
  } else {
    return initial;
  }
}

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
  const [step, setStep] = useState(1);

  useEffect(() => {
    const newMin = Math.min(min, value);
    const newMax = Math.max(max, value);
    setStep(getStepSize(Math.abs(newMax - newMin)))
    setMin(newMin);
    setMax(newMax);
  }, [value, min, max]);

  const steps = [...Array(STEP_COUNT).keys()]
    .map(stepIndex => stepIndex * step)
    .concat(max);

  const offset = getOffsetFromValue(Math.abs(value), step);

  return (
    <div className={styles.gauge}>
      <div className={styles.gaugeLabel}>
        {label}
      </div>
      <svg
        width="66px"
        height="143px"
        viewBox="0 0 66 143"
      >
        <line
          id="scale"
          fill="none"
          stroke="#999"
          x1="50"
          y1="3"
          x2="50"
          y2="140"
        />
        <g
          style={{ transition: 'transform 0.3s ease-in-out' }}
          transform={translation(0, offset)}
        >
          {steps.map((stepValue, stepIndex) => {
            const stepOffset = getOffsetFromStep(stepIndex)
            return (
              <g key={`step${stepValue}-${stepIndex}`}>
                <text
                  x="13"
                  y={stepOffset - STEP_HEIGHT / 2 + STEP_FONT_HEIGHT / 2 - 1}
                  width="26"
                  height={STEP_HEIGHT}
                  fontFamily={inter.style.fontFamily}
                  fontSize={`${STEP_FONT_HEIGHT}px`}
                >
                  {formatStepLabel(stepValue, step)}
                </text>
                <line
                  stroke="#999"
                  strokeWidth="1"
                  x1="39"
                  y1={stepOffset + STEP_HEIGHT / 2}
                  x2="50"
                  y2={stepOffset + STEP_HEIGHT / 2}
                />
                <line
                  stroke="#999"
                  strokeWidth="1"
                  x1="44"
                  y1={stepOffset + STEP_HEIGHT}
                  x2="50"
                  y2={stepOffset + STEP_HEIGHT}
                />
              </g>
            )
          })}
        </g>
        <path
          id="border"
          fill="#D3D3D3"
          d="M60.4,0H5.6C2.5,0,0,2.5,0,5.6v132.2c0,3.1,2.5,5.6,5.6,5.6h54.8c3.1,0,5.6-2.5,5.6-5.6V5.6
            C66.1,2.5,63.5,0,60.4,0z M62.5,134.5c0,3.1-2.5,5.6-5.6,5.6H9.2c-3.1,0-5.6-2.5-5.6-5.6V9c0-3.1,2.5-5.6,5.6-5.6h47.6
            c3.1,0,5.6,2.5,5.6,5.6V134.5z"
        />
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
        {formatLabel(value)}
      </div>
    </div>
  )
};