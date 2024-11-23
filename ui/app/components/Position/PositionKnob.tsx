import { FC, useEffect, useRef, useState, useCallback } from "react";
import { inter } from "../../inter";
import styles from "./PositionKnob.module.css";

const ERROR_RADIUS = 46.3
const OFFSET_RADIUS = 52.3
const RAD_TO_DEG = 57.2958;
const EPSILON = 0.0001;
const CROSSOVER = 3;
const CX = 101.89999389648438;
const CY = 95.19999694824219;

const getAngleFromValue = (value: number, min: number, max: number): number => {
  const midpoint = (min + max) / 2;
  const range = max - min;
  const half = range / 2;
  const norm = (value - midpoint) / half;

  // Map the normalized value to radians (-π to π)
  return norm * Math.PI;
}

const getValueFromAngle = (angle: number, min: number, max: number): number => {
  // Map the angle (-π to π) to normalized value (-1 to 1)
  const norm = angle / Math.PI;

  // Map the normalized value to the range [min, max]
  const range = max - min;
  const knobValue = ((norm + 1) / 2) * range + min;

  return knobValue;
}

const getAngleFromPoint = (x: number, y: number, cx: number, cy: number) => {
  const sin = y - cy;
  const cos = x - cx;
  const angle = Math.atan2(sin, cos);

  return angle;
};

const getSectorFromValue = (
  cx: number,
  cy: number,
  radius: number,
  value: number
) => {
  const angle = -value * Math.PI * 2 + Math.PI / 2;
  const x = cx + Math.cos(angle) * radius;
  const y = cy - Math.sin(angle) * radius;

  return { x, y };
}

const getCircumference = (radius: number) => (
  2 * Math.PI * radius
);

const getDasharray = (circumference: number, norm: number) => {
  const factor = Math.abs(norm);
  return `${circumference * factor},${circumference * (1 - factor)}`
};

const getDasharrayOffset = (circumference: number) => (
  circumference * 0.25
);

const getDasharrayTransform = (value: number) => (
  `rotate(${value < 0 ? value * Math.PI * 2 : 0}rad)`
);

const OFFSET_CIRCUMFERENCE = getCircumference(OFFSET_RADIUS);

type PositionKnobProps = {
  goal: number;
  position: number;
  min: number;
  max: number;
  handleChange: (goal: number) => void;
};

export const PositionKnob: FC<PositionKnobProps> = ({
  goal,
  position,
  min,
  max,
  handleChange
}) => {
  const offsetRef = useRef(0);
  const startRef = useRef(0);
  const isMouseDownRef = useRef(false);
  const crossOverRef = useRef(false);

  const [angle, setAngle] = useState(0);

  useEffect(() => {
    if (crossOverRef.current) return;

    const nextAngle = getAngleFromValue(goal, min, max);

    if (Math.abs(nextAngle - angle) > EPSILON) {
      setAngle(nextAngle);
    }
  }, [goal, angle, min, max]);

  const handleMouseDown = useCallback((e) => {
    e.preventDefault();
    isMouseDownRef.current = true;

    const { offsetX, offsetY } = e.nativeEvent;

    offsetRef.current = getAngleFromPoint(offsetX, offsetY, CX, CY);
    startRef.current = angle;
  }, [angle]);

  const handleMouseUp = useCallback((e) => {
    if (e.target.tagName === 'INPUT') return;
    e.preventDefault();
    isMouseDownRef.current = false;
  }, []);

  const handleMouseMove = useCallback((e) => {
    e.preventDefault();

    if (isMouseDownRef.current) {
      const { offsetX, offsetY } = e.nativeEvent;

      // Angle from mouse position
      const angleFromPoint = getAngleFromPoint(offsetX, offsetY, CX, CY);

      // Offset
      const angleWithOffset = angleFromPoint - offsetRef.current + startRef.current;

      // Wrap
      const angleWrapped = angleWithOffset > Math.PI
        ? angleWithOffset - Math.PI * 2
        : angleWithOffset;

      // Map
      const value = getValueFromAngle(angleWrapped, min, max);

      // Crossover
      const delta = angleWrapped - angle;
      crossOverRef.current = Math.abs(delta) >= CROSSOVER

      if (crossOverRef.current) {
        handleChange(delta > 0 ? min : max);
        setAngle(delta > 0 ? -Math.PI : Math.PI);
      } else {
        handleChange(value);
      }
    }
  }, [angle, handleChange, min, max]);

  useEffect(() => {
    document.addEventListener('mouseup', handleMouseUp, true);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp, true);
    }
  }, [handleMouseUp]);

  const {
    x: goalX,
    y: goalY
  } = getSectorFromValue(CX, CY, ERROR_RADIUS, goal);

  const {
    x: errorX,
    y: errorY
  } = getSectorFromValue(CX, CY, ERROR_RADIUS, position);

  return (
    <svg
      width="200.4px"
      height="193.2px"
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
      className={styles.positionKnob}
    >
      <circle
        id="range"
        fill="none"
        stroke="#7A7A7A"
        cx="101.9"
        cy="95.5"
        r="52.3"
      />
      <circle
        id="offset"
        fill="none"
        stroke="#424242"
        strokeWidth="4"
        strokeDasharray={getDasharray(OFFSET_CIRCUMFERENCE, goal)}
        strokeDashoffset={getDasharrayOffset(OFFSET_CIRCUMFERENCE)}
        style={{
          transformOrigin: '101.9px 95.5px',
          transform: getDasharrayTransform(goal),
        }}
        cx="101.9"
        cy="95.5"
        r="52.3"
      />
      <g id="spacer-left">
        <path
          fill="none"
          stroke="#D3D3D3"
          strokeWidth="2"
          d="M36.9,33.5l6.6,6.5C29.2,54.2,20.3,73.8,20.3,95.5
          c0,21.6,8.8,41.1,22.9,55.3l-6.4,6.4"
        />
      </g>
      <path
        id="spacer-right"
        fill="none"
        stroke="#D3D3D3"
        strokeWidth="2"
        d="M166.9,33.5l-6.6,6.5c14.3,14.2,23.1,33.8,23.1,55.5c0,21.6-8.8,41.1-22.9,55.3l6.4,6.4"
      />
      <text
        id="label-min"
        transform="matrix(1 0 0 1 96.7357 13.4993)"
        fontFamily={inter.style.fontFamily}
        fontSize="18px"
      >
        {0}
      </text>
      <text
        id="label-max"
        transform="matrix(1 0 0 1 92 188.6604)"
        fontFamily={inter.style.fontFamily}
        fontSize="18px"
      >
        {max * 100}
      </text>
      <g id="ticks" fill="none" stroke="#A5A5A5">
        <line x1="162.6" y1="95.6" x2="168.4" y2="95.6"/>
        <line x1="101.9" y1="33.5" x2="101.9" y2="22.2"/>
        <line x1="91.1" y1="34.4" x2="90.1" y2="28.9"/>
        <line x1="80.6" y1="37.2" x2="78.7" y2="31.9"/>
        <line x1="70.8" y1="41.8" x2="68" y2="36.9"/>
        <line x1="61.8" y1="47.8" x2="58.3" y2="43.6"/>
        <line x1="54.1" y1="55.5" x2="50" y2="52"/>
        <line x1="48.1" y1="64.5" x2="43.2" y2="61.7"/>
        <line x1="43.6" y1="74.3" x2="38" y2="72.3"/>
        <line x1="40.8" y1="84.7" x2="35.2" y2="83.8"/>
        <line x1="39.8" y1="95.5" x2="28.5" y2="95.5"/>
        <line x1="162.6" y1="95.6" x2="175.2" y2="95.5"/>
        <line x1="162.9" y1="84.7" x2="168.5" y2="83.8"/>
        <line x1="160.2" y1="74.3" x2="165.5" y2="72.3"/>
        <line x1="155.6" y1="64.5" x2="160.5" y2="61.7"/>
        <line x1="149.4" y1="55.6" x2="153.7" y2="52"/>
        <line x1="141.9" y1="47.8" x2="145.4" y2="43.6"/>
        <line x1="132.9" y1="41.8" x2="135.7" y2="36.9"/>
        <line x1="123.1" y1="37.2" x2="125.1" y2="31.6"/>
        <line x1="112.6" y1="34.4" x2="113.6" y2="28.9"/>
        <line x1="101.9" y1="157.5" x2="101.9" y2="168.8"/>
        <line x1="91.1" y1="156.6" x2="90.1" y2="162.2"/>
        <line x1="80.6" y1="153.8" x2="78.7" y2="159.1"/>
        <line x1="70.8" y1="149.2" x2="68" y2="154.1"/>
        <line x1="61.8" y1="143.2" x2="58.3" y2="147.4"/>
        <line x1="54.1" y1="135.5" x2="50" y2="139"/>
        <line x1="48.1" y1="126.5" x2="43.2" y2="129.3"/>
        <line x1="43.6" y1="116.7" x2="38" y2="118.8"/>
        <line x1="40.8" y1="106.3" x2="35.2" y2="107.3"/>
        <line x1="162.9" y1="106.3" x2="168.5" y2="107.3"/>
        <line x1="160.2" y1="116.7" x2="165.5" y2="118.7"/>
        <line x1="155.6" y1="126.5" x2="160.5" y2="129.3"/>
        <line x1="149.4" y1="135.4" x2="153.7" y2="139"/>
        <line x1="141.9" y1="143.2" x2="145.4" y2="147.4"/>
        <line x1="132.9" y1="149.2" x2="135.7" y2="154.1"/>
        <line x1="123.1" y1="153.8" x2="125.1" y2="159.4"/>
        <line x1="112.6" y1="156.6" x2="113.6" y2="162.2"/>
      </g>
      <g
        id="knob"
        onMouseDown={handleMouseDown}
        onMouseUp={handleMouseUp}
        onMouseMove={handleMouseMove}
        style={{
          cursor: "pointer",
          transformOrigin: '101.9px 95.2px',
          transform: `rotate(${angle * RAD_TO_DEG}deg)`,
        }}
      >
        <circle
          id="knob-shadow"
          cx="101.9"
          cy="95.2"
          r="43.6"
        />
        <circle
          id="knob-highlight"
          fill="#424242"
          cx="101.9"
          cy="95.2"
          r="31.2"
        />
        <g id="knob-head">
          <path
            id="knob-body"
            d="M121.7,136.5v-21.1c0-0.9-0.3-1.8-0.9-2.6c-1-1.3-2.2-3.4-2.3-4.4l-4.4-45.7c-0.2-2.3-0.7-4.5-1.4-6.6
            L106,35.4h-4.1h-4.1L91,56c-0.7,2.2-1.2,4.4-1.4,6.6l-4.4,45.7c-0.1,1-1.3,3-2.3,4.4c-0.6,0.8-0.9,1.7-0.9,2.6v21.1
            C82,145.6,121.7,145,121.7,136.5z"
          />
          <path
            id="knob-mark"
            fill="#FFFFFF"
            d="M101.9,65.4c-0.7,0-1.2-0.5-1.2-1.2V35.4c0-0.7,0.5-1.2,1.2-1.2s1.2,0.5,1.2,1.2v28.8
            C103.1,64.9,102.5,65.4,101.9,65.4z"
          />
        </g>
      </g>
      <line
        id="goalLine"
        stroke="blue"
        strokeWidth="1"
        x1={CX}
        y1={CY}
        x2={goalX}
        y2={goalY}
      />
      <line
        id="errorLine"
        stroke="red"
        strokeWidth="1"
        x1={CX}
        y1={CY}
        x2={errorX}
        y2={errorY}
      />
    </svg>
  );
}
