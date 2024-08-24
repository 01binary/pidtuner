import { FC } from "react";
import styles from "./Performer.module.css";

type PlayheadProps = {
  position: number;
};

export const Playhead: FC<PlayheadProps> = ({
  position
}) => (
  <svg
    width="16.5px"
    height="294.7px"
    viewBox="0 0 16.5 294.7"
    className={styles.playhead}
    style={{
      left: position
    }}
  >
    <polygon
      fill="#FFFFFF"
      stroke="#000000"
      stroke-linecap="round"
      stroke-linejoin="round"
      stroke-miterlimit="10"
      points="0,11.6 8.3,24.6 16.5,11.6 16.5,0 0,0 "
    />
  
    <line
      fill="none"
      stroke="#424242"
      stroke-linejoin="round"
      stroke-miterlimit="10"
      x1="8.3"
      y1="24.6"
      x2="8.3"
      y2="294.7"
    />
  </svg>
);
