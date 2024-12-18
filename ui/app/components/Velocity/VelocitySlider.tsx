import { FC, useCallback, useRef, useEffect } from "react";
import { inter } from "../../inter";
import styles from "./VelocitySlider.module.css";

const MIN = 1;
const MAX = -1;
const HALF = 55;
const INCREMENTS = [1, 0.75, 0.5, 0.25];

type VelocitySliderProps = {
  velocity: number;
  handleChange: (velocity: number) => void;
  invert?: boolean;
}

export const VelocitySlider: FC<VelocitySliderProps> = ({
  velocity,
  handleChange,
  invert
}) => {
  const borderRef = useRef<SVGRectElement>(null);
  const sliderRef = useRef<SVGGElement>(null);
  const isDraggingRef = useRef<boolean>(false);
  const dragOffsetRef = useRef<number>(0);

  const handleMarkClick = useCallback((
    index: number,
    forward: boolean = true
  ) => {
    const increment = forward ? INCREMENTS[index] : -INCREMENTS[index];
    handleChange(increment);
  }, [handleChange]);

  const handleJump = useCallback((increment: number) => {
    const nextVelocity = velocity + increment;
    handleChange(Math.max(Math.min(nextVelocity, 1), -1));
  }, [velocity, handleChange]);

  const handleMouseDown = useCallback((e) => {
    if (!sliderRef.current || !borderRef.current)
      return;

    e.preventDefault();

    const { clientY: mousePosition } = e;
    const { y: sliderTop, height: sliderHeight } = sliderRef.current
      .getBoundingClientRect();
    const sliderCenter = sliderTop + sliderHeight / 2;

    isDraggingRef.current = true;
    dragOffsetRef.current = mousePosition - sliderCenter;
  }, []);

  const handleMouseMove = useCallback((e) => {
    if (!isDraggingRef.current || !borderRef.current)
      return;

    e.preventDefault();

    const { clientY: mousePosition } = e;
    const { top: min, bottom: max } = borderRef.current
      .getBoundingClientRect();

    const norm = ((mousePosition - dragOffsetRef.current) - min) / (max - min);
    const value = norm * (MAX - MIN) + MIN;

    handleChange(Math.max(Math.min(value, MIN), MAX));
  }, []);

  const handleMouseUp = useCallback((e) => {
    if (e.target.tagName === 'INPUT') {
      return;
    }

    e.preventDefault();

    isDraggingRef.current = false;
  }, []);

  useEffect(() => {
    document.addEventListener('mouseup', handleMouseUp, true);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp, true);
    }
  }, [handleMouseUp]);

  return (
    <svg
      className={styles.velocitySlider}
      width="175px"
      height="150px"
      viewBox="0 0 175 150"
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
    >
      <g id="interactiveMarks">
        <polygon
          id="t3"
          onClick={() => handleMarkClick(3, true)}
          className={styles.mark}
          fillRule="evenodd"
          clipRule="evenodd"
          fill="#D3D3D3"
          points="74.2,65.8 74.2,56.4 77,56.4 74.5,65.8 	"
        />
        <polygon
          id="t2"
          onClick={() => handleMarkClick(2, true)}
          className={styles.mark}
          fillRule="evenodd"
          clipRule="evenodd"
          fill="#D3D3D3"
          points="74.2,54.3 74.2,44.9 80.1,44.9 77.6,54.3 	"
        />
        <polygon
          id="t1"
          onClick={() => handleMarkClick(1, true)}
          className={styles.mark}
          fillRule="evenodd"
          clipRule="evenodd"
          fill="#D3D3D3"
          points="74.2,42.8 74.2,33.4 83.2,33.4 80.7,42.8 	"
        />
        <polygon
          id="t0"
          onClick={() => handleMarkClick(0, true)}
          className={styles.mark}
          fillRule="evenodd"
          clipRule="evenodd"
          fill="#D3D3D3"
          points="86.3,22 83.7,31.3 74.2,31.3 74.2,22 	"
        />
        <polygon
          id="b3"
          onClick={() => handleMarkClick(0, false)}
          className={styles.mark}
          fillRule="evenodd"
          clipRule="evenodd"
          fill="#D3D3D3"
          points="86.3,126.4 83.7,117 74.2,117 74.2,126.4 	"
        />
        <polygon
          id="b2"
          onClick={() => handleMarkClick(1, false)}
          className={styles.mark}
          fillRule="evenodd"
          clipRule="evenodd"
          fill="#D3D3D3"
          points="74.2,105.5 74.2,114.9 83.2,114.9 80.7,105.5"
        />
        <polygon
          id="b1"
          onClick={() => handleMarkClick(2, false)}
          className={styles.mark}
          fillRule="evenodd"
          clipRule="evenodd"
          fill="#D3D3D3"
          points="74.2,94 74.2,103.4 80.1,103.4 77.6,94 	"
        />
        <polygon
          id="b0"
          onClick={() => handleMarkClick(3, false)}
          className={styles.mark}
          fillRule="evenodd"
          clipRule="evenodd"
          fill="#D3D3D3"
          points="74.2,82.5 74.2,91.9 77,91.9 74.5,82.5 	"
        />
      </g>

      <g id="interactiveLabels">
        <text
          className={styles.label}
          transform="matrix(1 0 0 1 15.6066 24.0709)"
          fontFamily={inter.style.fontFamily}
          fontSize="14px"
          onClick={() => handleChange(1)}
        >
          100
        </text>
        <text
          className={styles.label}
          transform="matrix(1 0 0 1 23.5636 51.2281)"
          fontFamily={inter.style.fontFamily}
          fontSize="14px"
          onClick={() => handleChange(0.5)}
        >
          50
        </text>
        <text
          className={styles.label}
          transform="matrix(1 0 0 1 31.5206 78.8717)"
          fontFamily={inter.style.fontFamily}
          fontSize="14px"
          onClick={() => handleChange(0)}
        >
          0
        </text>
        <text
          className={styles.label}
          transform="matrix(1 0 0 1 18.9699 106.9201)"
          fontFamily={inter.style.fontFamily}
          fontSize="14px"
          onClick={() => handleChange(-0.5)}
        >
          -50
        </text>
        <text
          className={styles.label}
          transform="matrix(1 0 0 1 11.0128 133.6735)"
          fontFamily={inter.style.fontFamily}
          fontSize="14px"
          onClick={() => handleChange(-1)}
        >
          -100
        </text>
      </g>

      <g
        id="jumpButtonTop"
        className={styles.jumpButton}
        onClick={() => handleJump(0.1)}
      >
        <polygon
          id="jumpTopBorder"
          className={styles.jumpBorder}
          stroke="#A5A5A5"
          fill="none"
          strokeMiterlimit="10"
          points="92.6,0.5 76.8,0.5 68.9,8 76.8,15.5 92.6,15.5 100.4,8"
        />
        <polygon
          id="plus"
          fillRule="evenodd"
          clipRule="evenodd"
          fill="#EC008C"
          points="90.2,7.3 85.4,7.3 85.4,2.5 83.9,2.5 83.9,7.3 79.1,7.3 79.1,8.8 83.9,8.8 83.9,13.6 85.4,13.6 85.4,8.8 90.2,8.8"
        />
      </g>

      <g
        id="jumpButtonBottom"
        className={styles.jumpButton}
        onClick={() => handleJump(-0.1)}
      >
        <polygon
          id="jumpBottomBorder"
          className={styles.jumpBorder}
          fill="white"
          stroke="#A5A5A5"
          strokeMiterlimit="10"
          points="92.6,133 76.8,133 68.9,140.5 76.8,148 92.6,148 100.4,140.5"
        />
        <rect
          id="minus"
          x="79.1"
          y="139.7"
          fillRule="evenodd"
          clipRule="evenodd"
          fill="#376BE8"
          width="11.1"
          height="1.5"
        />
      </g>

      <rect
        ref={borderRef}
        id="border"
        x="70.9"
        y="19.4"
        fill="none"
        stroke="#A5A5A5"
        strokeMiterlimit="10"
        width="27.5"
        height="109.8"
      />

      <g id="ticks">
        <line
          fill="none"
          stroke="#A5A5A5"
          strokeMiterlimit="10"
          x1="60.3"
          y1="74.2"
          x2="45.5"
          y2="74.2"
        />
        <line
          fill="none"
          stroke="#A5A5A5"
          strokeMiterlimit="10"
          x1="60.3"
          y1="101.6"
          x2="52.9"
          y2="101.6"
        />
        <line
          fill="none"
          stroke="#A5A5A5"
          strokeMiterlimit="10"
          x1="60.3"
          y1="46.7"
          x2="52.9"
          y2="46.7"
        />
        <polyline
          fill="none"
          stroke="#A5A5A5"
          strokeMiterlimit="10"
          points="45.5,129.1 60.3,129.1 60.3,129 60.3,19.2 45.5,19.2 	"
        />
      </g>

      <g id="staticLabels">
        <text
          transform="matrix(1 0 0 1 109.4626 145.6881)"
          fill="#5B5B5B"
          fontFamily={inter.style.fontFamily}
          fontSize="18px"
        >
          {invert ? 'LPWM' : 'RPWM'}
        </text>
        <text
          transform="matrix(1 0 0 1 110.2097 12.8253)"
          fill="#5B5B5B"
          fontFamily={inter.style.fontFamily}
          fontSize="18px"
        >
          {invert ? 'RPWM' : 'LPWM'}
        </text>
      </g>

      <g
        id="sliderHead"
        ref={sliderRef}
        className={styles.sliderHead}
        style={{
          transform: `translate(0, ${-velocity * HALF}px)`,
        }}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
      >
        <polygon
          fill="#FFFFFF"
          stroke="#000000"
          strokeLinecap="round"
          strokeLinejoin="round"
          strokeMiterlimit="10"
          points="93.2,66 80.2,74.2 93.2,82.5 104.8,82.5 104.8,66"
        />
      </g>
    </svg>
  );
};
