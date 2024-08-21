"use client";

import { useCallback } from "react";
import { RAD_TO_DEG, useKnob } from "../knobUtils";
import styles from "./VelocityKnob.module.css";

const INCREMENTS = [
  0.08, 0.13, 0.18, 0.25, 0.27, 0.33, 0.4, 0.46, 0.50, 0.56, 0.62, 0.68, 0.75, 0.78, 0.85, 0.9
];

type VelocityKnobProps = {
  velocity: number;
  handleChange: (velocity: number) => void;
  invert?: boolean;
};

export const VelocityKnob = ({
  velocity,
  handleChange,
  invert
}: VelocityKnobProps) => {
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
    value: velocity,
    invert,
    range: 'half',
    wrap: true,
    handleChange
  })

  const invertMultiplier = invert ? -1 : 1;

  const handleMarkClick = useCallback((index: number, forward: boolean = true) => {
    const shouldInvert = forward ? invert : !invert;
    const increment = shouldInvert ? -INCREMENTS[index] : INCREMENTS[index];
    handleChange(increment);
  }, [invert, handleChange]);

  const handleJump = useCallback((increment: number) => {
    const nextVelocity = velocity + increment;
    handleChange(Math.max(Math.min(nextVelocity, 1), -1));
  }, [velocity, handleChange])

  return (
    <svg
      ref={svgRef}
      className={styles.velocityKnob}
      width="220px"
      height="123px"
      viewBox="0 0 219.9 122.7"
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
    >
      <g
        id="knobInteractive"
        ref={knobRef}
        onMouseDown={handleMouseDown}
        onMouseUp={handleMouseUp}
        style={{
          cursor: 'pointer',
          transformOrigin: `${knobCenterX - originX}px ${knobCenterY - originY}px`,
          transform: `rotate(${(angle) * RAD_TO_DEG}deg)`
        }}
      >
        <path id="knob" d="M107.4,30.5c-0.5,0-0.9,0-1.4,0c-2.3,0-4.2,0.9-5.8,2.7c-2.6,3-6,4.8-9.9,5.4c-1.9,0.3-3.5,1.2-4.6,2.9
          c-0.8,1.3-1.6,2.6-2.4,3.9c-1.1,1.9-1.2,3.8-0.5,5.9c0.7,2,1,4,1,6.1c0,2-0.3,4-1,6.1c-0.7,2.1-0.6,4,0.5,5.9
          c0.8,1.3,1.6,2.6,2.4,3.9c1.1,1.7,2.6,2.6,4.6,2.9c3.9,0.7,7.3,2.5,9.9,5.4c1.6,1.8,3.5,2.7,5.8,2.7c1.2,0,2.5,0,3.7,0
          c2-0.1,3.8-0.9,5.2-2.5c2.5-3,5.7-4.8,9.6-5.7c5.4-1.3,9-7.8,7.4-13c-0.6-1.9-0.8-3.7-0.8-5.6c0-1.9,0.2-3.7,0.8-5.6
          c1.6-5.3-2-11.8-7.4-13c-3.8-0.9-7-2.7-9.6-5.7c-1.4-1.6-3.1-2.4-5.2-2.5C109,30.5,108.2,30.5,107.4,30.5"/>
        <circle id="knobCenter" fill="#FFFFFF" cx="107.4" cy="57.4" r="19.6"/>
        <circle id="dot" fill="#FFFFFF" cx="107.4" cy="33.9" r="1.3"/>
      </g>

      <g id="gradationMarksInteractive" fill="#D3D3D3">
        <path onClick={() => handleMarkClick(0, true)} className={styles.mark} d="M114.2,19.7c1.8,0.3,3.6,0.8,5.3,1.4l0.5-1.4c-1.8-0.5-3.7-1-5.6-1.2L114.2,19.7z"/>
        <path onClick={() => handleMarkClick(1, true)} className={styles.mark} d="M125.2,23.7l0.9-1.6c-1.7-0.8-3.5-1.6-5.3-2.2l-0.5,1.4C121.9,22,123.6,22.8,125.2,23.7z"/>
        <path onClick={() => handleMarkClick(2, true)} className={styles.mark} d="M130.4,27.2l1.5-1.7c-1.6-1.1-3.3-2.1-5-3l-0.9,1.6C127.5,25.1,129,26.1,130.4,27.2z"/>
        <path onClick={() => handleMarkClick(3, true)} className={styles.mark} d="M135,31.6l2.2-1.8c-1.4-1.4-2.9-2.6-4.5-3.8l-1.5,1.8C132.5,29,133.8,30.3,135,31.6z"/>
        <path onClick={() => handleMarkClick(4, true)} className={styles.mark} d="M138.7,36.7l3-1.7c-1.2-1.6-2.5-3.1-3.9-4.6l-2.2,1.9C136.7,33.7,137.7,35.2,138.7,36.7z"/>
        <path onClick={() => handleMarkClick(5, true)} className={styles.mark} d="M141.4,42.4l3.9-1.4c-0.9-1.8-2-3.6-3.2-5.3l-3.1,1.8C140,39,140.8,40.7,141.4,42.4z"/>
        <path onClick={() => handleMarkClick(6, true)} className={styles.mark} d="M143.1,48.5l4.9-0.9c-0.6-2-1.4-4-2.3-5.9l-4,1.5C142.3,44.9,142.8,46.7,143.1,48.5z"/>
        <path onClick={() => handleMarkClick(7, true)} className={styles.mark} d="M143.8,54.8h5.7c-0.2-2.2-0.6-4.2-1.2-6.3l-5,0.9C143.6,51.1,143.7,52.9,143.8,54.8z"/>
        <path onClick={() => handleMarkClick(8, true)} className={styles.mark} d="M149.6,55.6C149.6,55.6,149.5,55.6,149.6,55.6l-5.8,0c0,1.8-0.2,3.7-0.5,5.4l6.4,1.1C149.8,60,149.8,57.8,149.6,55.6z"/>
        <path onClick={() => handleMarkClick(9, true)} className={styles.mark} d="M143.1,61.9c-0.3,1.8-0.8,3.6-1.4,5.2l6.8,2.4c0.5-2.1,0.9-4.3,1.1-6.6L143.1,61.9z"/>
        <path onClick={() => handleMarkClick(10, true)} className={styles.mark} d="M139.1,72.9l6.8,3.9c1-2,1.7-4.2,2.4-6.3l-6.8-2.5C140.8,69.7,140,71.3,139.1,72.9z"/>
        <path onClick={() => handleMarkClick(11, true)} className={styles.mark} d="M138.7,73.7c-0.9,1.6-2,3-3.2,4.4l6.4,5.4c1.3-1.9,2.5-3.8,3.5-5.9L138.7,73.7z"/>
        <path onClick={() => handleMarkClick(12, true)} className={styles.mark} d="M135,78.8c-1.2,1.4-2.5,2.6-3.9,3.8l5.7,6.8c1.7-1.6,3.2-3.3,4.6-5.2L135,78.8z"/>
        <path onClick={() => handleMarkClick(13, true)} className={styles.mark} d="M130.4,83.1c-1.4,1.1-2.9,2.2-4.5,3.1l4.7,8c2-1.3,3.8-2.7,5.6-4.3L130.4,83.1z"/>
        <path onClick={() => handleMarkClick(14, true)} className={styles.mark} d="M125.2,86.7c-1.6,0.9-3.2,1.7-4.9,2.3l3.3,9c2.2-0.9,4.3-2,6.3-3.2L125.2,86.7z"/>
        <path onClick={() => handleMarkClick(15, true)} className={styles.mark} d="M122.7,98.3l-3.3-9c-1.7,0.6-3.5,1.1-5.3,1.4l1.7,9.6C118.2,99.8,120.5,99.2,122.7,98.3z"/>

        <path onClick={() => handleMarkClick(0, false)} className={styles.mark} d="M100.7,19.7c-1.8,0.3-3.6,0.8-5.3,1.4l-0.5-1.4c1.8-0.5,3.7-1,5.6-1.2L100.7,19.7z"/>
        <path onClick={() => handleMarkClick(1, false)} className={styles.mark} d="M89.6,23.7l-0.9-1.6c1.7-0.8,3.5-1.6,5.3-2.2l0.5,1.4C92.9,22,91.2,22.8,89.6,23.7z"/>
        <path onClick={() => handleMarkClick(2, false)} className={styles.mark} d="M84.4,27.2l-1.5-1.7c1.6-1.1,3.3-2.1,5-3l0.9,1.6C87.3,25.1,85.8,26.1,84.4,27.2z"/>
        <path onClick={() => handleMarkClick(3, false)} className={styles.mark} d="M79.9,31.6l-2.2-1.8c1.4-1.4,2.9-2.6,4.5-3.8l1.5,1.8C82.4,29,81.1,30.3,79.9,31.6z"/>
        <path onClick={() => handleMarkClick(4, false)} className={styles.mark} d="M76.1,36.7l-3-1.7c1.2-1.6,2.5-3.1,3.9-4.6l2.2,1.9C78.1,33.7,77.1,35.2,76.1,36.7z"/>
        <path onClick={() => handleMarkClick(5, false)} className={styles.mark} d="M73.4,42.4L69.4,41c0.9-1.8,2-3.6,3.2-5.3l3.1,1.8C74.8,39,74,40.7,73.4,42.4z"/>
        <path onClick={() => handleMarkClick(6, false)} className={styles.mark} d="M71.7,48.5l-4.9-0.9c0.6-2,1.4-4,2.3-5.9l4,1.5C72.5,44.9,72,46.7,71.7,48.5z"/>
        <path onClick={() => handleMarkClick(7, false)} className={styles.mark} d="M71.1,54.8h-5.7c0.2-2.2,0.6-4.2,1.2-6.3l5,0.9C71.2,51.1,71.1,52.9,71.1,54.8z"/>
        <path onClick={() => handleMarkClick(8, false)} className={styles.mark} d="M65.3,55.6C65.3,55.6,65.3,55.6,65.3,55.6l5.8,0c0,1.8,0.2,3.7,0.5,5.4l-6.4,1.1C65,60,65,57.8,65.3,55.6z"/>
        <path onClick={() => handleMarkClick(9, false)} className={styles.mark} d="M71.7,61.9c0.3,1.8,0.8,3.6,1.4,5.2l-6.8,2.4c-0.5-2.1-0.9-4.3-1.1-6.6L71.7,61.9z"/>
        <path onClick={() => handleMarkClick(11, false)} className={styles.mark} d="M76.1,73.7c0.9,1.6,2,3,3.2,4.4l-6.4,5.4c-1.3-1.9-2.5-3.8-3.5-5.9L76.1,73.7z"/>
        <path onClick={() => handleMarkClick(12, false)} className={styles.mark} d="M79.9,78.8c1.2,1.4,2.5,2.6,3.9,3.8L78,89.4c-1.7-1.6-3.2-3.3-4.6-5.2L79.9,78.8z"/>
        <path onClick={() => handleMarkClick(13, false)} className={styles.mark} d="M84.4,83.1c1.4,1.1,2.9,2.2,4.5,3.1l-4.7,8c-2-1.3-3.8-2.7-5.6-4.3L84.4,83.1z"/>
        <path onClick={() => handleMarkClick(14, false)} className={styles.mark} d="M89.6,86.7c1.6,0.9,3.2,1.7,4.9,2.3l-3.3,9c-2.2-0.9-4.3-2-6.3-3.2L89.6,86.7z"/>
        <path onClick={() => handleMarkClick(10, false)} className={styles.mark} d="M75.7,72.9l-6.8,3.9c-1-2-1.7-4.2-2.4-6.3l6.8-2.5C74,69.7,74.8,71.3,75.7,72.9z"/>
        <path onClick={() => handleMarkClick(15, false)} className={styles.mark} d="M92.1,98.3l3.3-9c1.7,0.6,3.5,1.1,5.3,1.4l-1.7,9.6C96.6,99.8,94.3,99.2,92.1,98.3z"/>
      </g>

      <g id="textLabelsStatic" style={{ pointerEvents: 'none' }}>
        <text transform="matrix(1 0 0 1 177.2629 16.6725)" fill="#5B5B5B" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          {invert ? 'RPWM' : 'LPWM'}
        </text>
        <text transform="matrix(1 0 0 1 5.1185 16.6725)" fill="#5B5B5B" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          {invert ? 'LPWM' : 'RPWM'}
        </text>
      </g>
      
      <g id="textLabelsInteractive">
        <text onClick={() => handleChange(0.75 * invertMultiplier)} className={styles.interactiveLabel} transform="matrix(1 0 0 1 149.6492 105.817)" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          {invert ? -75 : 75}
        </text>
        <text onClick={() => handleChange(0.25 * invertMultiplier)} className={styles.interactiveLabel} transform="matrix(1 0 0 1 149.6492 16.6725)" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          {invert ? -25 : 25}
        </text>
        <text onClick={() => handleChange(0.5 * invertMultiplier)} className={styles.interactiveLabel} transform="matrix(1 0 0 1 169.9804 62.0267)" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          {invert ? -50 : 50}
        </text>
        <text onClick={() => handleChange(-0.25 * invertMultiplier)} className={styles.interactiveLabel} transform="matrix(1 0 0 1 49.5592 16.6725)" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          {invert ? 25 : -25}
        </text>
        <text onClick={() => handleChange(-0.5 * invertMultiplier)} className={styles.interactiveLabel} transform="matrix(1 0 0 1 29.2288 62.0267)" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          {invert ? 50 : -50}
        </text>
        <text onClick={() => handleChange(-0.75 * invertMultiplier)} className={styles.interactiveLabel} transform="matrix(1 0 0 1 49.5592 105.817)" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          {invert ? 75 : -75}
        </text>
        <text onClick={() => handleChange(0)} className={styles.interactiveLabel} transform="matrix(1 0 0 1 103.9989 8.9245)" fontFamily="'Roboto-Medium', sans-serif, sans-serif" fontSize="12px">
          0
        </text>
        <text onClick={() => handleChange(1)} className={styles.interactiveLabel} transform="matrix(1 0 0 1 97.1786 119.0267)" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          100
        </text>
      </g>

      <g id="jumpButtons">
        <g
          id="plusRightButton"
          style={invert ? { visibility: 'hidden' } : { visibility: 'visible' }}
          className={styles.jumpButton}
          onClick={() => handleJump(0.1)}
        >
          <polygon
            className={styles.jumpBorder}
            fill="white"
            stroke="#A5A5A5"
            strokeMiterlimit="10"
            points="134.9,100.2 124.4,100.2 119.1,105.2 124.4,110.2 134.9,110.2 140.1,105.2"
          />
          <polygon
            id="plusRight"
            className={styles.jumpLabel}
            fillRule="evenodd"
            clipRule="evenodd"
            fill="#EC008C"
            points="133.3,104.7 130.1,104.7 130.1,101.5 129.1,101.5 129.1,104.7 125.9,104.7 125.9,105.7 129.1,105.7 129.1,108.9 130.1,108.9 130.1,105.7 133.3,105.7 "
          />
        </g>

        <g
          id="plusLeftButton"
          style={invert ? { visibility: 'visible' } : { visibility: 'hidden' }}
          className={styles.jumpButton}
          onClick={() => handleJump(0.1)}
        >
          <polygon
            className={styles.jumpBorder}
            fill="white"
            stroke="#A5A5A5"
            strokeMiterlimit="10"
            points="89.6,100.2 79.1,100.2 73.9,105.2 79.1,110.2 89.6,110.2 94.9,105.2 "
          />
          <polygon
            id="plusLeft"
            className={styles.jumpLabel}
            fillRule="evenodd"
            clipRule="evenodd"
            fill="#EC008C"
            points="88.1,104.7 84.9,104.7 84.9,101.5 83.9,101.5 83.9,104.7 80.7,104.7 80.7,105.7 83.9,105.7 83.9,108.9 84.9,108.9 84.9,105.7 88.1,105.7 "
          />
        </g>

        <g
          id="minusRightButton"
          style={invert ? { visibility: 'visible' } : { visibility: 'hidden' }}
          className={styles.jumpButton}
          onClick={() => handleJump(-0.1)}
        >
          <polygon
            className={styles.jumpBorder}
            fill="white"
            stroke="#A5A5A5"
            strokeMiterlimit="10"
            points="134.9,100.2 124.4,100.2 119.1,105.2 124.4,110.2 134.9,110.2 140.1,105.2"
          />
          <rect
            id="minusRight"
            className={styles.jumpLabel}
            x="125.9"
            y="104.7"
            fillRule="evenodd"
            clipRule="evenodd"
            fill="#376BE8"
            width="7.4"
            height="1"
          />
        </g>

        <g
          id="minusLeftButton"
          style={invert ? { visibility: 'hidden' } : { visibility: 'visible' }}
          className={styles.jumpButton}
          onClick={() => handleJump(-0.1)}
        >
          <polygon
            className={styles.jumpBorder}
            fill="white"
            stroke="#A5A5A5"
            strokeMiterlimit="10"
            points="89.6,100.2 79.1,100.2 73.9,105.2 79.1,110.2 89.6,110.2 94.9,105.2 "
          />
          <rect
            id="minusLeft"
            className={styles.jumpLabel}
            x="80.7"
            y="104.7"
            fillRule="evenodd"
            clipRule="evenodd"
            fill="#376BE8"
            width="7.4"
            height="1"
          />
        </g>
      </g>

      <g id="ticks" style={{ pointerEvents: 'none' }}>
        <line fill="none" stroke="#A5A5A5" strokeMiterlimit="10" x1="157.7" y1="58.4" x2="162.8" y2="58.4"/>
        <path fill="none" stroke="#A5A5A5" strokeMiterlimit="10" d="M148,21.6l-3.9,3.9c8.4,8.4,13.7,20,13.7,32.9
          c0,12.8-5.2,24.4-13.6,32.7l3.8,3.8"/>
        <line fill="none" stroke="#A5A5A5" strokeMiterlimit="10" x1="154.2" y1="40.6" x2="157.7" y2="39.2"/>
        <line fill="none" stroke="#A5A5A5" strokeMiterlimit="10" x1="154.5" y1="75.5" x2="158" y2="77"/>
        <line fill="none" stroke="#A5A5A5" strokeMiterlimit="10" x1="57.1" y1="58.4" x2="52" y2="58.4"/>
        <path fill="none" stroke="#A5A5A5" strokeMiterlimit="10" d="M66.9,21.6l3.9,3.9c-8.4,8.4-13.7,20-13.7,32.9
          c0,12.8,5.2,24.4,13.6,32.7l-3.8,3.8"/>
        <line fill="none" stroke="#A5A5A5" strokeMiterlimit="10" x1="60.6" y1="40.6" x2="57.1" y2="39.2"/>
        <line fill="none" stroke="#A5A5A5" strokeMiterlimit="10" x1="60.4" y1="75.5" x2="56.8" y2="77"/>
      </g>
    </svg>
  );
};
