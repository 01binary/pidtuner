"use client";

import { useEffect, FC, useRef, useState, useLayoutEffect } from "react";
import { inter } from "../../inter";
import { RAD_TO_DEG, useKnob } from "../knobUtils";
import styles from "./PositionKnob.module.css";

const ERROR_RADIUS = 46.3
const OFFSET_RADIUS = 52.3

const getCircumference = (radius: number) => (
  2 * Math.PI * radius
)

const getGoalDasharray = (circumference: number, norm: number) => {
  const factor = Math.abs(norm) / 2;
  return `${circumference * factor},${circumference * (1 - factor)}`
}

const getErrorDasharray = (circumference: number, norm: number) => {
  const factor = Math.abs(norm);
  return `${circumference * factor},${circumference * (1 - factor)}`
}

const getErrorRotationFull = (position: number, error: number): number => (
  error < 0
    ? position * Math.PI - Math.PI / 2
    : (position - error) * Math.PI - Math.PI / 2
);

const ERROR_CIRCUMFERENCE = getCircumference(ERROR_RADIUS)
const OFFSET_CIRCUMFERENCE = getCircumference(OFFSET_RADIUS)

type PositionKnobProps = {
  goal: number;
  position: number;
  error: number;
  isFullRange: boolean;
  handleChange: (goal: number) => void;
};

export const PositionKnob: FC<PositionKnobProps> = ({
  goal,
  position,
  error,
  isFullRange,
  handleChange
}) => {
  const centerRef = useRef<SVGElement>(null);
  const [centerX, setCenterX] = useState(0);
  const [centerY, setCenterY] = useState(0);

  const {
    svgRef,
    knobRef,
    handleMouseDown,
    handleMouseUp,
    handleMouseMove,
    originX,
    originY,
    angle
  } = useKnob({
    value: goal,
    wrap: true,
    centerX,
    centerY,
    handleChange,
    isFullRange
  })

  useLayoutEffect(() => {
    if (!centerRef.current) {
      return
    }

    const {
      left,
      top,
      width,
      height
    } = centerRef.current.getBoundingClientRect()

    setCenterX(left + width / 2);
    setCenterY(top + height / 2);
  }, [])

  useEffect(() => {
    document.addEventListener('mouseup', handleMouseUp, true);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp, true);
    }
  }, [handleMouseUp]);

  const errorRotation = getErrorRotationFull(position, error)

  return (
    <svg
      ref={svgRef}
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
        id="error"
        fill="none"
        stroke="#EC008C"
        strokeWidth="6"
        strokeDasharray={getErrorDasharray(ERROR_CIRCUMFERENCE, Math.abs(error) / 2)}
        strokeDashoffset={0}
        style={{
          transformOrigin: '101.9px 95.5px',
          transform: `rotate(${errorRotation}rad)`,
        }}
        cx="101.9"
        cy="95.5"
        r="46.3"
      />
      <circle
        id="offset"
        fill="none"
        stroke="#424242"
        strokeWidth="4"
        strokeDasharray={getGoalDasharray(OFFSET_CIRCUMFERENCE, goal)}
        strokeDashoffset={OFFSET_CIRCUMFERENCE * 0.25}
        style={{
          transformOrigin: '101.9px 95.5px',
          transform: `rotate(${goal < 0 ? goal * Math.PI : 0}rad)`,
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
      <g id="ticks">
        <line fill="none" stroke="#A5A5A5" x1="162.6" y1="95.6" x2="168.4" y2="95.6"/>
        <line fill="none" stroke="#A5A5A5" x1="101.9" y1="33.5" x2="101.9" y2="22.2"/>
        <line fill="none" stroke="#A5A5A5" x1="91.1" y1="34.4" x2="90.1" y2="28.9"/>
        <line fill="none" stroke="#A5A5A5" x1="80.6" y1="37.2" x2="78.7" y2="31.9"/>
        <line fill="none" stroke="#A5A5A5" x1="70.8" y1="41.8" x2="68" y2="36.9"/>
        <line fill="none" stroke="#A5A5A5" x1="61.8" y1="47.8" x2="58.3" y2="43.6"/>
        <line fill="none" stroke="#A5A5A5" x1="54.1" y1="55.5" x2="50" y2="52"/>
        <line fill="none" stroke="#A5A5A5" x1="48.1" y1="64.5" x2="43.2" y2="61.7"/>
        <line fill="none" stroke="#A5A5A5" x1="43.6" y1="74.3" x2="38" y2="72.3"/>
        <line fill="none" stroke="#A5A5A5" x1="40.8" y1="84.7" x2="35.2" y2="83.8"/>
        <line fill="none" stroke="#A5A5A5" x1="39.8" y1="95.5" x2="28.5" y2="95.5"/>
        <line fill="none" stroke="#A5A5A5" x1="162.6" y1="95.6" x2="175.2" y2="95.5"/>
        <line fill="none" stroke="#A5A5A5" x1="162.9" y1="84.7" x2="168.5" y2="83.8"/>
        <line fill="none" stroke="#A5A5A5" x1="160.2" y1="74.3" x2="165.5" y2="72.3"/>
        <line fill="none" stroke="#A5A5A5" x1="155.6" y1="64.5" x2="160.5" y2="61.7"/>
        <line fill="none" stroke="#A5A5A5" x1="149.4" y1="55.6" x2="153.7" y2="52"/>
        <line fill="none" stroke="#A5A5A5" x1="141.9" y1="47.8" x2="145.4" y2="43.6"/>
        <line fill="none" stroke="#A5A5A5" x1="132.9" y1="41.8" x2="135.7" y2="36.9"/>
        <line fill="none" stroke="#A5A5A5" x1="123.1" y1="37.2" x2="125.1" y2="31.6"/>
        <line fill="none" stroke="#A5A5A5" x1="112.6" y1="34.4" x2="113.6" y2="28.9"/>
        <line fill="none" stroke="#A5A5A5" x1="101.9" y1="157.5" x2="101.9" y2="168.8"/>
        <line fill="none" stroke="#A5A5A5" x1="91.1" y1="156.6" x2="90.1" y2="162.2"/>
        <line fill="none" stroke="#A5A5A5" x1="80.6" y1="153.8" x2="78.7" y2="159.1"/>
        <line fill="none" stroke="#A5A5A5" x1="70.8" y1="149.2" x2="68" y2="154.1"/>
        <line fill="none" stroke="#A5A5A5" x1="61.8" y1="143.2" x2="58.3" y2="147.4"/>
        <line fill="none" stroke="#A5A5A5" x1="54.1" y1="135.5" x2="50" y2="139"/>
        <line fill="none" stroke="#A5A5A5" x1="48.1" y1="126.5" x2="43.2" y2="129.3"/>
        <line fill="none" stroke="#A5A5A5" x1="43.6" y1="116.7" x2="38" y2="118.8"/>
        <line fill="none" stroke="#A5A5A5" x1="40.8" y1="106.3" x2="35.2" y2="107.3"/>
        <line fill="none" stroke="#A5A5A5" x1="162.9" y1="106.3" x2="168.5" y2="107.3"/>
        <line fill="none" stroke="#A5A5A5" x1="160.2" y1="116.7" x2="165.5" y2="118.7"/>
        <line fill="none" stroke="#A5A5A5" x1="155.6" y1="126.5" x2="160.5" y2="129.3"/>
        <line fill="none" stroke="#A5A5A5" x1="149.4" y1="135.4" x2="153.7" y2="139"/>
        <line fill="none" stroke="#A5A5A5" x1="141.9" y1="143.2" x2="145.4" y2="147.4"/>
        <line fill="none" stroke="#A5A5A5" x1="132.9" y1="149.2" x2="135.7" y2="154.1"/>
        <line fill="none" stroke="#A5A5A5" x1="123.1" y1="153.8" x2="125.1" y2="159.4"/>
        <line fill="none" stroke="#A5A5A5" x1="112.6" y1="156.6" x2="113.6" y2="162.2"/>
      </g>
      <g
        id="knob"
        ref={knobRef}
        onMouseDown={handleMouseDown}
        onMouseUp={handleMouseUp}
        onMouseMove={handleMouseMove}
        style={{
          cursor: "pointer",
          transformOrigin: `${centerX - originX}px ${
            centerY - originY
          }px`,
          transform: `rotate(${angle * RAD_TO_DEG}deg)`,
        }}
      >
        <circle
          id="knob-shadow"
          ref={centerRef}
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
    </svg>
  );
}
