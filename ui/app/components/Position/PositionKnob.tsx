"use client";

import { useCallback, useEffect, FC } from "react";
import { inter } from "../../inter";
import { RAD_TO_DEG, useKnob } from "../knobUtils";
import styles from "./PositionKnob.module.css";

type PositionKnobProps = {
  position: number;
  handleChange: (position: number) => void;
};

export const PositionKnob: FC<PositionKnobProps> = ({
  position,
  handleChange
}) => {
  const {
    svgRef,
    knobRef,
    handleMouseDown,
    handleMouseUp,
    handleMouseMove,
    knobCenterX,
    knobCenterY,
    originX,
    originY,
    angle
  } = useKnob({
    value: position,
    range: 'full',
    wrap: true,
    handleChange
  })

  useEffect(() => {
    document.addEventListener('mouseup', handleMouseUp, true);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp, true);
    }
  }, [handleMouseUp]);

  return (
    <svg
      ref={svgRef}
      width="200.4px"
      height="193.2px"
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
      className={styles.positionKnob}
    >
      <path
        id="error"
        fill="#EC008C"
        d="M102.3,95.5c0,0,42.8,28.8,42.8,28.8c8.6-9.9,9-28.8,9-28.8H102.3z"
      />
      <path
        id="offset"
        fill="none"
        stroke="#424242"
        strokeWidth="6"
        strokeLinejoin="round"
        d="M101.9,147.8c18.2,0,34.3-9.3,43.6-23.4"
      />
      <circle
        id="range"
        fill="none"
        stroke="#7A7A7A"
        cx="101.9"
        cy="95.5"
        r="52.3"
      />
      <g id="spacer-left">
        <line
          fill="none"
          stroke="#D3D3D3"
          strokeWidth="2"
          x1="20.3"
          y1="95.5"
          x2="0"
          y2="95.5"
        />
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
        stroke-miterlimit="10"
        d="M166.9,33.5l-6.6,6.5c14.3,14.2,23.1,33.8,23.1,55.5c0,21.6-8.8,41.1-22.9,55.3l6.4,6.4"
      />
      <g id="arrow-ccw">
        <path
          fill="none"
          stroke="#EC008C"
          strokeLinecap="round"
          strokeLinejoin="round"
          d="M188.8,49.8c4.2,6.3,7.5,13.1,10,20.3"
        />
        <path
          fill="#EC008C"
          d="M190.3,52l6.3-0.6l0-0.2l-8.1-5c-2.4-2.1-4.7-4.1-7.1-6.2c1.3,2.9,2.6,5.7,3.9,8.6l2.4,9.2l0.2,0.1
          L190.3,52z"
        />
      </g>
      <g id="arrow-cw">
        <path
          fill="none"
          stroke="#376BE8"
          strokeLinecap="round"
          strokeLinejoin="round"
          d="M188.8,141.3c4.2-6.3,7.5-13.1,10-20.3"
        />
        <path
          fill="#376BE8"
          d="M190.3,139l-2.4-5.9l-0.2,0.1l-2.4,9.2c-1.3,2.9-2.6,5.7-3.9,8.6c2.4-2.1,4.7-4.1,7.1-6.2l8.1-5l0-0.2
          L190.3,139z"
        />
      </g>
      <text
        id="label-min"
        transform="matrix(1 0 0 1 96.7357 13.4993)"
        fontFamily={inter.style.fontFamily}
        fontSize="18px"
      >
        0
      </text>
      <text
        id="label-max"
        transform="matrix(1 0 0 1 86.5052 188.6604)"
        fontFamily={inter.style.fontFamily}
        fontSize="18px"
      >
        100
      </text>
      <g id="ticks">
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="162.6" y1="95.6" x2="168.4" y2="95.6"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="101.9" y1="33.5" x2="101.9" y2="22.2"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="91.1" y1="34.4" x2="90.1" y2="28.9"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="80.6" y1="37.2" x2="78.7" y2="31.9"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="70.8" y1="41.8" x2="68" y2="36.9"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="61.8" y1="47.8" x2="58.3" y2="43.6"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="54.1" y1="55.5" x2="50" y2="52"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="48.1" y1="64.5" x2="43.2" y2="61.7"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="43.6" y1="74.3" x2="38" y2="72.3"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="40.8" y1="84.7" x2="35.2" y2="83.8"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="39.8" y1="95.5" x2="28.5" y2="95.5"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="162.6" y1="95.6" x2="175.2" y2="95.5"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="162.9" y1="84.7" x2="168.5" y2="83.8"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="160.2" y1="74.3" x2="165.5" y2="72.3"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="155.6" y1="64.5" x2="160.5" y2="61.7"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="149.4" y1="55.6" x2="153.7" y2="52"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="141.9" y1="47.8" x2="145.4" y2="43.6"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="132.9" y1="41.8" x2="135.7" y2="36.9"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="123.1" y1="37.2" x2="125.1" y2="31.6"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="112.6" y1="34.4" x2="113.6" y2="28.9"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="101.9" y1="157.5" x2="101.9" y2="168.8"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="91.1" y1="156.6" x2="90.1" y2="162.2"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="80.6" y1="153.8" x2="78.7" y2="159.1"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="70.8" y1="149.2" x2="68" y2="154.1"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="61.8" y1="143.2" x2="58.3" y2="147.4"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="54.1" y1="135.5" x2="50" y2="139"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="48.1" y1="126.5" x2="43.2" y2="129.3"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="43.6" y1="116.7" x2="38" y2="118.8"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="40.8" y1="106.3" x2="35.2" y2="107.3"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="162.9" y1="106.3" x2="168.5" y2="107.3"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="160.2" y1="116.7" x2="165.5" y2="118.7"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="155.6" y1="126.5" x2="160.5" y2="129.3"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="149.4" y1="135.4" x2="153.7" y2="139"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="141.9" y1="143.2" x2="145.4" y2="147.4"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="132.9" y1="149.2" x2="135.7" y2="154.1"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="123.1" y1="153.8" x2="125.1" y2="159.4"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="112.6" y1="156.6" x2="113.6" y2="162.2"/>
      </g>
      <g
        id="knob"
        ref={knobRef}
        onMouseDown={handleMouseDown}
        onMouseUp={handleMouseUp}
        onMouseMove={handleMouseMove}
        style={{
          cursor: "pointer",
          transformOrigin: `${knobCenterX - originX}px ${
            knobCenterY - originY
          }px`,
          transform: `rotate(${angle * RAD_TO_DEG}deg)`,
        }}
      >
        <circle id="knob-shadow" cx="101.9" cy="95.2" r="43.6"/>
        <circle id="knob-highlight" fill="#424242" cx="101.9" cy="95.2" r="31.2"/>
        <g id="knob-head">
          <path id="knob-body" d="M121.7,136.5v-21.1c0-0.9-0.3-1.8-0.9-2.6c-1-1.3-2.2-3.4-2.3-4.4l-4.4-45.7c-0.2-2.3-0.7-4.5-1.4-6.6
            L106,35.4h-4.1h-4.1L91,56c-0.7,2.2-1.2,4.4-1.4,6.6l-4.4,45.7c-0.1,1-1.3,3-2.3,4.4c-0.6,0.8-0.9,1.7-0.9,2.6v21.1
            C82,145.6,121.7,145,121.7,136.5z"/>
          <path id="knob-mark" fill="#FFFFFF" d="M101.9,65.4c-0.7,0-1.2-0.5-1.2-1.2V35.4c0-0.7,0.5-1.2,1.2-1.2s1.2,0.5,1.2,1.2v28.8
            C103.1,64.9,102.5,65.4,101.9,65.4z"/>
        </g>
      </g>
    </svg>
  );
}
