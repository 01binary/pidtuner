"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import styles from "./VelocityKnob.module.css";

const MAX_ANGLE = Math.PI;
const BIAS_ANGLE = Math.PI / 2;
const RAD_TO_DEG = 57.2958;
const DEG_TO_RAD = 0.0174533;

type VelocityKnobProps = {
  value: number,
  min: number,
  max: number
};

const getValueFromAngle = (angle: number, invert: boolean) => {
  const norm = angle / MAX_ANGLE;
  return invert ? norm : norm;
};

const getAngleFromPoint = (x: number, y: number, cx: number, cy: number, ox: number, oy: number) => {
  const sin = y - (cy - oy);
  const cos = x - (cx - ox);
  return Math.atan2(sin, cos) + BIAS_ANGLE;
};

export const VelocityKnob = () => {
  const svgRef = useRef<SVGSVGElement>(null);
  const knobRef = useRef<SVGPathElement>(null);
  const [knobCenterX, setKnobCenterX] = useState(0);
  const [knobCenterY, setKnobCenterY] = useState(0);
  const [originX, setOriginX] = useState(0);
  const [originY, setOriginY] = useState(0);
  const [angle, setAngle] = useState(0);
  const prevAngleRef = useRef(0);
  const isMouseDownRef = useRef(false);

  const [value, setValue] = useState(0);
  const invert = false;

  useEffect(() => {
    const { x, y, width, height } = knobRef.current
      ?.getBoundingClientRect();

    const { x: ox, y: oy } = svgRef.current
      ?.getBoundingClientRect();

    setKnobCenterX(x + width / 2);
    setKnobCenterY(y + height / 2);
    setOriginX(ox);
    setOriginY(oy);
  }, []);

  const handleMouseDown = useCallback((e) => {
    e.preventDefault();
    e.stopPropagation();
    isMouseDownRef.current = true;
  }, []);

  const handleMouseUp = useCallback((e) => {
    e.preventDefault();
    e.stopPropagation();
    isMouseDownRef.current = false;
  }, []);

  const handleMouseMove = useCallback((e) => {
    e.preventDefault();
    e.stopPropagation();

    if (isMouseDownRef.current) {
      const { offsetX, offsetY } = e.nativeEvent;

      const currentAngle = getAngleFromPoint(
        offsetX, offsetY, knobCenterX, knobCenterY, originX, originY);

      const lastAngle = prevAngleRef.current;
      const delta = currentAngle - lastAngle;

      let nextAngle = lastAngle + delta;

      if (nextAngle > Math.PI) nextAngle -= Math.PI * 2;

      setAngle(nextAngle);
      setValue(getValueFromAngle(nextAngle, false));

      prevAngleRef.current = currentAngle;
    }
  }, [originX, originY, knobCenterX, knobCenterY]);

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
        ref={knobRef}
        onMouseDown={handleMouseDown}
        onMouseUp={handleMouseUp}
        style={{
          cursor: 'pointer',
          transformOrigin: `${knobCenterX - originX}px ${knobCenterY - originY}px`,
          transform: `rotate(${(angle) * RAD_TO_DEG}deg)`
        }}
      >
        <path
          d="M107.4,30.5c-0.5,0-0.9,0-1.4,0c-2.3,0-4.2,0.9-5.8,2.7c-2.6,3-6,4.8-9.9,5.4c-1.9,0.3-3.5,1.2-4.6,2.9
          c-0.8,1.3-1.6,2.6-2.4,3.9c-1.1,1.9-1.2,3.8-0.5,5.9c0.7,2,1,4,1,6.1c0,2-0.3,4-1,6.1c-0.7,2.1-0.6,4,0.5,5.9
          c0.8,1.3,1.6,2.6,2.4,3.9c1.1,1.7,2.6,2.6,4.6,2.9c3.9,0.7,7.3,2.5,9.9,5.4c1.6,1.8,3.5,2.7,5.8,2.7c1.2,0,2.5,0,3.7,0
          c2-0.1,3.8-0.9,5.2-2.5c2.5-3,5.7-4.8,9.6-5.7c5.4-1.3,9-7.8,7.4-13c-0.6-1.9-0.8-3.7-0.8-5.6c0-1.9,0.2-3.7,0.8-5.6
          c1.6-5.3-2-11.8-7.4-13c-3.8-0.9-7-2.7-9.6-5.7c-1.4-1.6-3.1-2.4-5.2-2.5C109,30.5,108.2,30.5,107.4,30.5 M127.1,57.4
          c0,10.8-8.8,19.6-19.6,19.6c-10.8,0-19.6-8.9-19.6-19.6c0-10.8,8.8-19.6,19.6-19.6c10.8,0,19.6,8.8,19.6,19.6
          C127.1,57.3,127.1,57.4,127.1,57.4C127.1,57.4,127.1,57.4,127.1,57.4z"
        />
        <circle fill="#FFFFFF" cx="107.4" cy="33.9" r="1.3"/>
      </g>
      <text
        fontFamily="'Roboto-Medium', sans-serif, sans-serif"
        fontSize="12px"
        fill="red"
        x={knobCenterX - originX - 11}
        y={knobCenterY - originY + 4}
      >
        {Math.round(value * 100) / 100}
      </text>

      <g style={{ pointerEvents: 'none' }}>
        <text transform="matrix(1 0 0 1 177.2629 16.6725)" fill="#5B5B5B" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          {invert ? 'RPWM' : 'LPWM'}
        </text>
        <text transform="matrix(1 0 0 1 5.1185 16.6725)" fill="#5B5B5B" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          {invert ? 'LPWM' : 'RPWM'}
        </text>
        <text transform="matrix(1 0 0 1 149.6492 105.817)" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          {invert ? -75 : 75}
        </text>
        <text transform="matrix(1 0 0 1 149.6492 16.6725)" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          {invert ? -25 : 25}
        </text>
        <text transform="matrix(1 0 0 1 169.9804 62.0267)" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          {invert ? -50 : 50}
        </text>
        <text transform="matrix(1 0 0 1 49.5592 16.6725)" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          {invert ? 25 : -25}
        </text>
        <text transform="matrix(1 0 0 1 29.2288 62.0267)" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          {invert ? 50 : -50}
        </text>
        <text transform="matrix(1 0 0 1 49.5592 105.817)" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          {invert ? 75 : -75}
        </text>
        <text transform="matrix(1 0 0 1 103.9989 8.9245)" fontFamily="'Roboto-Medium', sans-serif, sans-serif" fontSize="12px">
          0
        </text>
        <text transform="matrix(1 0 0 1 97.1786 119.0267)" fontFamily="'Roboto-Medium', sans-serif" fontSize="12px">
          100
        </text>

        <rect x="73.4" y="110" fill="none" width="68" height="12"/>
        <rect x="73.4" y="-0.1" fill="none" width="68" height="12"/>
        <line fill="none" stroke="#A5A5A5" strokeMiterlimit="10" x1="157.7" y1="58.4" x2="162.8" y2="58.4"/>
        <path fill="none" stroke="#A5A5A5" strokeMiterlimit="10" d="M148,21.6l-3.9,3.9c8.4,8.4,13.7,20,13.7,32.9
          c0,12.8-5.2,24.4-13.6,32.7l3.8,3.8"/>
        <line fill="none" stroke="#A5A5A5" strokeMiterlimit="10" x1="154.2" y1="40.6" x2="157.7" y2="39.2"/>
        <line fill="none" stroke="#A5A5A5" strokeMiterlimit="10" x1="154.5" y1="75.5" x2="158" y2="77"/>
        <rect x="140" y="96.8" fill="none" width="36.9" height="12"/>
        <rect x="140" y="7.7" fill="none" width="36.9" height="12"/>
        <rect x="170.9" y="7.7" fill="none" width="49" height="12"/>
        <rect x="160.3" y="53" fill="none" width="36.9" height="12"/>
        <line fill="none" stroke="#A5A5A5" strokeMiterlimit="10" x1="57.1" y1="58.4" x2="52" y2="58.4"/>
        <path fill="none" stroke="#A5A5A5" strokeMiterlimit="10" d="M66.9,21.6l3.9,3.9c-8.4,8.4-13.7,20-13.7,32.9
          c0,12.8,5.2,24.4,13.6,32.7l-3.8,3.8"/>
        <line fill="none" stroke="#A5A5A5" strokeMiterlimit="10" x1="60.6" y1="40.6" x2="57.1" y2="39.2"/>
        <line fill="none" stroke="#A5A5A5" strokeMiterlimit="10" x1="60.4" y1="75.5" x2="56.8" y2="77"/>
        <rect x="37.9" y="7.7" fill="none" width="36.9" height="12"/>
        <rect x="17.6" y="53" fill="none" width="36.9" height="12"/>
        <rect x="37.9" y="96.8" fill="none" width="36.9" height="12"/>
        <rect y="7.7" fill="none" width="45.5" height="12"/>
        <path fill="#D3D3D3" d="M125.2,23.7l0.9-1.6c-1.7-0.8-3.5-1.6-5.3-2.2l-0.5,1.4C121.9,22,123.6,22.8,125.2,23.7z"/>
        <path fill="#D3D3D3" d="M130.4,27.2l1.5-1.7c-1.6-1.1-3.3-2.1-5-3l-0.9,1.6C127.5,25.1,129,26.1,130.4,27.2z"/>
        <path fill="#D3D3D3" d="M125.2,86.7c-1.6,0.9-3.2,1.7-4.9,2.3l3.3,9c2.2-0.9,4.3-2,6.3-3.2L125.2,86.7z"/>
        <path fill="#D3D3D3" d="M135,31.6l2.2-1.8c-1.4-1.4-2.9-2.6-4.5-3.8l-1.5,1.8C132.5,29,133.8,30.3,135,31.6z"/>
        <path fill="#D3D3D3" d="M135,78.8c-1.2,1.4-2.5,2.6-3.9,3.8l5.7,6.8c1.7-1.6,3.2-3.3,4.6-5.2L135,78.8z"/>
        <path fill="#D3D3D3" d="M130.4,83.1c-1.4,1.1-2.9,2.2-4.5,3.1l4.7,8c2-1.3,3.8-2.7,5.6-4.3L130.4,83.1z"/>
        <path fill="#D3D3D3" d="M138.7,73.7c-0.9,1.6-2,3-3.2,4.4l6.4,5.4c1.3-1.9,2.5-3.8,3.5-5.9L138.7,73.7z"/>
        <path fill="#D3D3D3" d="M139.1,72.9l6.8,3.9c1-2,1.7-4.2,2.4-6.3l-6.8-2.5C140.8,69.7,140,71.3,139.1,72.9z"/>
        <path fill="#D3D3D3" d="M138.7,36.7l3-1.7c-1.2-1.6-2.5-3.1-3.9-4.6l-2.2,1.9C136.7,33.7,137.7,35.2,138.7,36.7z"/>
        <path fill="#D3D3D3" d="M122.7,98.3l-3.3-9c-1.7,0.6-3.5,1.1-5.3,1.4l1.7,9.6C118.2,99.8,120.5,99.2,122.7,98.3z"/>
        <path fill="#D3D3D3" d="M141.4,42.4l3.9-1.4c-0.9-1.8-2-3.6-3.2-5.3l-3.1,1.8C140,39,140.8,40.7,141.4,42.4z"/>
        <path fill="#D3D3D3" d="M143.1,61.9c-0.3,1.8-0.8,3.6-1.4,5.2l6.8,2.4c0.5-2.1,0.9-4.3,1.1-6.6L143.1,61.9z"/>
        <path fill="#D3D3D3" d="M114.2,19.7c1.8,0.3,3.6,0.8,5.3,1.4l0.5-1.4c-1.8-0.5-3.7-1-5.6-1.2L114.2,19.7z"/>
        <path fill="#D3D3D3" d="M149.6,55.6C149.6,55.6,149.5,55.6,149.6,55.6l-5.8,0c0,1.8-0.2,3.7-0.5,5.4l6.4,1.1
          C149.8,60,149.8,57.8,149.6,55.6z"/>
        <path fill="#D3D3D3" d="M143.1,48.5l4.9-0.9c-0.6-2-1.4-4-2.3-5.9l-4,1.5C142.3,44.9,142.8,46.7,143.1,48.5z"/>
        <path fill="#D3D3D3" d="M143.8,54.8h5.7c-0.2-2.2-0.6-4.2-1.2-6.3l-5,0.9C143.6,51.1,143.7,52.9,143.8,54.8z"/>
        <path fill="#D3D3D3" d="M89.6,23.7l-0.9-1.6c1.7-0.8,3.5-1.6,5.3-2.2l0.5,1.4C92.9,22,91.2,22.8,89.6,23.7z"/>
        <path fill="#D3D3D3" d="M84.4,27.2l-1.5-1.7c1.6-1.1,3.3-2.1,5-3l0.9,1.6C87.3,25.1,85.8,26.1,84.4,27.2z"/>
        <path fill="#D3D3D3" d="M89.6,86.7c1.6,0.9,3.2,1.7,4.9,2.3l-3.3,9c-2.2-0.9-4.3-2-6.3-3.2L89.6,86.7z"/>
        <path fill="#D3D3D3" d="M79.9,31.6l-2.2-1.8c1.4-1.4,2.9-2.6,4.5-3.8l1.5,1.8C82.4,29,81.1,30.3,79.9,31.6z"/>
        <path fill="#D3D3D3" d="M79.9,78.8c1.2,1.4,2.5,2.6,3.9,3.8L78,89.4c-1.7-1.6-3.2-3.3-4.6-5.2L79.9,78.8z"/>
        <path fill="#D3D3D3" d="M84.4,83.1c1.4,1.1,2.9,2.2,4.5,3.1l-4.7,8c-2-1.3-3.8-2.7-5.6-4.3L84.4,83.1z"/>
        <path fill="#D3D3D3" d="M76.1,73.7c0.9,1.6,2,3,3.2,4.4l-6.4,5.4c-1.3-1.9-2.5-3.8-3.5-5.9L76.1,73.7z"/>
        <path fill="#D3D3D3" d="M75.7,72.9l-6.8,3.9c-1-2-1.7-4.2-2.4-6.3l6.8-2.5C74,69.7,74.8,71.3,75.7,72.9z"/>
        <path fill="#D3D3D3" d="M76.1,36.7l-3-1.7c1.2-1.6,2.5-3.1,3.9-4.6l2.2,1.9C78.1,33.7,77.1,35.2,76.1,36.7z"/>
        <path fill="#D3D3D3" d="M92.1,98.3l3.3-9c1.7,0.6,3.5,1.1,5.3,1.4l-1.7,9.6C96.6,99.8,94.3,99.2,92.1,98.3z"/>
        <path fill="#D3D3D3" d="M73.4,42.4L69.4,41c0.9-1.8,2-3.6,3.2-5.3l3.1,1.8C74.8,39,74,40.7,73.4,42.4z"/>
        <path fill="#D3D3D3" d="M71.7,61.9c0.3,1.8,0.8,3.6,1.4,5.2l-6.8,2.4c-0.5-2.1-0.9-4.3-1.1-6.6L71.7,61.9z"/>
        <path fill="#D3D3D3" d="M100.7,19.7c-1.8,0.3-3.6,0.8-5.3,1.4l-0.5-1.4c1.8-0.5,3.7-1,5.6-1.2L100.7,19.7z"/>
        <path fill="#D3D3D3" d="M65.3,55.6C65.3,55.6,65.3,55.6,65.3,55.6l5.8,0c0,1.8,0.2,3.7,0.5,5.4l-6.4,1.1C65,60,65,57.8,65.3,55.6z"
          />
        <path fill="#D3D3D3" d="M71.7,48.5l-4.9-0.9c0.6-2,1.4-4,2.3-5.9l4,1.5C72.5,44.9,72,46.7,71.7,48.5z"/>
        <path fill="#D3D3D3" d="M71.1,54.8h-5.7c0.2-2.2,0.6-4.2,1.2-6.3l5,0.9C71.2,51.1,71.1,52.9,71.1,54.8z"/>
        <rect x="125.9" y="104.7" fillRule="evenodd" clipRule="evenodd" fill="#376BE8" width="7.4" height="1"/>
        <polygon fillRule="evenodd" clipRule="evenodd" fill="#EC008C" points="88.1,104.7 84.9,104.7 84.9,101.5 83.9,101.5 83.9,104.7 
          80.7,104.7 80.7,105.7 83.9,105.7 83.9,108.9 84.9,108.9 84.9,105.7 88.1,105.7 "/>
      </g>
    </svg>
  );
};
