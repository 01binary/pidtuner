import { FC } from "react";
import styles from "./Timeline.module.css";

const STEP = 145.5;

type PlayheadProps = {
  position: number;
};

export const Playhead: FC<PlayheadProps> = ({
  position
}) => {
  const left = position * STEP;
  const isHalf = left < 0.1;

  return (
    <svg
      width="18px"
      height="295px"
      viewBox="0 0 18 295"
      className={styles.playhead}
      style={{ left }}
    >
      <polygon
        id="playheadFull"
        fill="#FFFFFF"
        stroke="#000000"
        strokeLinecap="round"
        strokeLinejoin="round"
        points="0.5,12.1 9,25.1 17.5,12.1 17.5,0.5 0.5,0.5"
        style={{
          visibility: isHalf ? 'hidden' : 'visible'
        }}
      />

      <polyline
        id="playheadHalf"
        fill="#FFFFFF"
        stroke="#000000"
        strokeLinecap="round"
        strokeLinejoin="round"
        points="9.5,24.3 17.5,12.1 17.5,0.5 9.5,0.5 9.5,24.3"
        style={{
          visibility: isHalf ? 'visible' : 'hidden'
        }}
      />
    
      <line
        fill="none"
        stroke="#898989"
        strokeLinejoin="round"
        x1="9"
        y1="25"
        x2="9"
        y2="295"
      />
    </svg>
  );
};
