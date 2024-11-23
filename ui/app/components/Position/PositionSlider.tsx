import { FC, useCallback, useRef, useEffect } from "react";
import { inter } from "../../inter";
import styles from "./PositionSlider.module.css";

const RANGE = 163;

const getSliderTransform = (position: number) => (
  `translate(${position * RANGE}px, 0px)`
)

const clamp = (value: number, min: number, max: number) => (
  Math.max(Math.min(value, max), min)
)

type PositionSliderProps = {
  goal: number;
  position: number;
  handleChange: (position: number) => void;
  min: number;
  max: number;
}

export const PositionSlider: FC<PositionSliderProps> = ({
  goal,
  position,
  handleChange,
  min,
  max
}) => {
  const borderRef = useRef<SVGRectElement>(null);
  const isDraggingRef = useRef<boolean>(false);
  const offsetRef = useRef<number>(0);

  const handleMouseDown = useCallback((e) => {
    if (!borderRef.current)
      return;

    e.preventDefault();

    const { clientX: mousePosition } = e;
    const { left: minPosition } = borderRef.current.getBoundingClientRect();
    const center = goal * RANGE;

    offsetRef.current = mousePosition - minPosition - center;
    isDraggingRef.current = true;
  }, [goal]);

  const handleMouseMove = useCallback((e) => {
    if (!isDraggingRef.current || !borderRef.current)
      return;

    e.preventDefault();

    const { clientX: mousePosition } = e;
    const { left: minPosition } = borderRef.current.getBoundingClientRect();

    const norm = (mousePosition - minPosition - offsetRef.current) / RANGE;
    const value = norm * (max - min) + min;
  
    handleChange(clamp(value, 0, 1));
  }, [min, max, handleChange]);

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
    <div className={styles.positionSlider}>
      <svg
        width="200.4px"
        height="96px"
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
      >
        <line
          id="error"
          y1="31.75"
          y2="31.75"
          x1={17.7 + position * RANGE}
          x2={17.7 + goal * RANGE}
          stroke="#EC008C"
          strokeWidth="29.5"
        />

        <rect
          id="track"
          ref={borderRef}
          x="17.7"
          y="17"
          fill="none"
          stroke="#A5A5A5"
          width="163"
          height="29.5"
        />

        <g id="scale">
          <line stroke="#A5A5A5" x1="99.1" y1="57.3" x2="99.1" y2="72.1"/>
          <line stroke="#A5A5A5" x1="139.9" y1="57.3" x2="139.9" y2="69.3"/>
          <line stroke="#A5A5A5" x1="160.3" y1="57.3" x2="160.3" y2="64.7"/>
          <line stroke="#A5A5A5" x1="119.5" y1="57.3" x2="119.5" y2="64.7"/>
          <line stroke="#A5A5A5" x1="58.3" y1="57.3" x2="58.3" y2="69.3"/>
          <line stroke="#A5A5A5" x1="78.7" y1="57.3" x2="78.7" y2="64.7"/>
          <line stroke="#A5A5A5" x1="38" y1="57.3" x2="38" y2="64.7"/>

          <polyline
            fill="none"
            stroke="#A5A5A5"
            points="180.7,72.1 180.7,57.3 180.5,57.3 17.6,57.3 17.6,72.1 	"
          />

          <text
            className={styles.interactive}
            transform="matrix(1 0 0 1 12.4579 93.6534)"
            fontFamily={inter.style.fontFamily}
            fontSize="18px"
            onClick={() => handleChange(min)}
          >
            0
          </text>

          <text
            className={styles.interactive}
            transform="matrix(1 0 0 1 88 93.6534)"
            fontFamily={inter.style.fontFamily}
            fontSize="18px"
            onClick={() => handleChange((max - min) / 2)}
          >
            50
          </text>

          <text
            className={styles.interactive}
            transform="matrix(1 0 0 1 165.4023 93.6534)"
            fontFamily={inter.style.fontFamily}
            fontSize="18px"
            onClick={() => handleChange(max)}
          >
            100
          </text>
        </g>

        <line
          id="current"
          stroke="#424242"
          strokeWidth="4"
          strokeLinejoin="round"
          x1="17.7"
          y1="46.7"
          x2={17.7 + goal * RANGE}
          y2="46.7"
        />

        <polygon
          id="head"
          className={styles.interactive}
          style={{ transform: getSliderTransform(goal) }}
          fill="#FFFFFF"
          stroke="#000000"
          strokeLinecap="round"
          strokeLinejoin="round"
          points="9.5,16.9 17.7,29.8 26,16.9 26,5.3 9.5,5.3"
          onMouseDown={handleMouseDown}
          onMouseMove={handleMouseMove}
          onMouseUp={handleMouseUp}
        />
      </svg>
    </div>
  )
};
