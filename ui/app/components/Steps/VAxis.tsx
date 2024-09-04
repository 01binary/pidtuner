import { FC } from "react";
import { inter } from "../../inter";
import styles from "./Timeline.module.css";

type VAxisProps = {
  invert?: boolean;
  onSetValue: (value: number) => void;
};

export const VAxis: FC<VAxisProps> = ({
  invert,
  onSetValue
}) => (
  <svg
    width="135px"
    height="334.8px"
    viewBox="0 0 135 334.8"
    className={styles.vaxis}
  >
    <g id="staticLabels">
      <text
        transform="matrix(1 0 0 1 0.8954 16.132)"
        fill="#5B5B5B"
        fontFamily={inter.style.fontFamily}
        fontSize="18px"
      >
        {invert ? 'RPWM' : 'LPWM'}
      </text>
      <text
        transform="matrix(1 0 0 1 -1.849316e-04 276.6776)"
        fill="#5B5B5B"
        fontFamily={inter.style.fontFamily}
        fontSize="18px"
      >
        {invert ? 'LPWM' : 'RPWM'}
      </text>
    </g>

    <g id="clickableLabels">
      <text
        transform="matrix(1 0 0 1 86.7763 145.8334)"
        fontFamily={inter.style.fontFamily}
        fontSize="18px"
        className={styles.label}
        onClick={() => onSetValue(0)}
      >
        0
      </text>
      <text
        transform="matrix(1 0 0 1 66.3153 16.1102)"
        fontFamily={inter.style.fontFamily}
        fontSize="18px"
        onClick={() => onSetValue(1)}
      >
        100
      </text>
      <text
        transform="matrix(1 0 0 1 76.5448 80.5704)"
        fontFamily={inter.style.fontFamily}
        fontSize="18px"
        className={styles.label}
        onClick={() => onSetValue(0.5)}
      >
        50
      </text>
      <text
        transform="matrix(1 0 0 1 76.5448 47.9391)"
        fontFamily={inter.style.fontFamily}
        fontSize="18px"
        className={styles.label}
        onClick={() => onSetValue(0.75)}
      >
        75
      </text>
      <text
        transform="matrix(1 0 0 1 76.5448 113.2018)"
        fontFamily={inter.style.fontFamily}
        fontSize="18px"
        className={styles.label}
        onClick={() => onSetValue(0.25)}
      >
        25
      </text>
      <text
        transform="matrix(1 0 0 1 70.6395 178.4055)"
        fontFamily={inter.style.fontFamily}
        fontSize="18px"
        className={styles.label}
        onClick={() => onSetValue(-0.25)}
      >
        -25
      </text>
      <text
        transform="matrix(1 0 0 1 70.6386 211.308)"
        fontFamily={inter.style.fontFamily}
        fontSize="18px"
        className={styles.label}
        onClick={() => onSetValue(-0.5)}
      >
        -50
      </text>
      <text
        transform="matrix(1 0 0 1 70.6386 242.775)"
        fontFamily={inter.style.fontFamily}
        fontSize="18px"
        onClick={() => onSetValue(-0.75)}
      >
        -75
      </text>
      <text
        transform="matrix(1 0 0 1 60.4091 276.2621)"
        fontFamily={inter.style.fontFamily}
        fontSize="18px"
        className={styles.label}
        onClick={() => onSetValue(-1)}
      >
        -100
      </text>
    </g>

    <g id="ticks" strokeMiterlimit="10">
      <line fill="none" stroke="#A5A5A5" x1="117.3" y1="139.3" x2="102.5" y2="139.3"/>
      <line fill="none" stroke="#A5A5A5" x1="117.3" y1="204.5" x2="105.3" y2="204.5"/>
      <line fill="none" stroke="#A5A5A5" x1="117.3" y1="237.1" x2="109.9" y2="237.1"/>
      <line fill="none" stroke="#A5A5A5" x1="117.3" y1="171.9" x2="109.9" y2="171.9"/>
      <line fill="none" stroke="#A5A5A5" x1="117.3" y1="74.1" x2="105.3" y2="74.1"/>
      <line fill="none" stroke="#A5A5A5" x1="117.3" y1="106.7" x2="109.9" y2="106.7"/>
      <line fill="none" stroke="#A5A5A5" x1="117.3" y1="41.5" x2="109.9" y2="41.5"/>
      <polyline fill="none" stroke="#A5A5A5" points="102.5,269.8 117.3,269.8 117.3,269.4 117.3,8.9 102.5,8.9 	"/>
    </g>
  </svg>
);
