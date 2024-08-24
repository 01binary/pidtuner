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
    style={{ left: position }}
  >
    <polygon
      fill="#FFFFFF"
      stroke="#000000"
      stroke-linecap="round"
      stroke-linejoin="round"
      stroke-miterlimit="10"
      points="0.5,12.1 9,25.1 17.5,12.1 17.5,0.5 0.5,0.5"
    />
  
    <line
      fill="none"
      stroke="#898989"
      stroke-linejoin="round"
      stroke-miterlimit="10"
      x1="9"
      y1="25"
      x2="9"
      y2="295"
    />
  </svg>
);
