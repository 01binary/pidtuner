import { FC } from "react";
import styles from "./Timeline.module.css";

type PlayheadProps = {
  position: number;
};

export const Playhead: FC<PlayheadProps> = ({
  position
}) => (
  <svg
    width="18px"
    height="295px"
    viewBox="0 0 18 295"
    className={styles.playhead}
    style={{ left: position * 146.3 }}
  >
    <polygon
      fill="#FFFFFF"
      stroke="#000000"
      strokeLinecap="round"
      strokeLinejoin="round"
      points="0.5,12.1 9,25.1 17.5,12.1 17.5,0.5 0.5,0.5"
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
