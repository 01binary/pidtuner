import { FC, useCallback } from "react";
import styles from "./VelocitySlider.module.css";

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
  const invertMultiplier = invert ? -1 : 1;

  const handleMarkClick = useCallback((
    index: number,
    forward: boolean = true
  ) => {
    const shouldInvert = forward ? invert : !invert;
    const increment = shouldInvert ? -INCREMENTS[index] : INCREMENTS[index];
    handleChange(increment);
  }, [invert, handleChange]);

  const handleJump = useCallback((increment: number) => {
    const nextVelocity = velocity + increment;
    handleChange(Math.max(Math.min(nextVelocity, 1), -1));
  }, [velocity, handleChange]);

  return (
    <svg
      className={styles.velocitySlider}
      width="175px"
      height="150px"
      viewBox="0 0 175 150"
    >
      <g id="interactiveMarks">
        <polygon id="t3" onClick={() => handleMarkClick(3, true)} className={styles.mark} fillRule="evenodd" clipRule="evenodd" fill="#D3D3D3" points="74.2,65.8 74.2,56.4 77,56.4 74.5,65.8 	"/>
        <polygon id="t2" onClick={() => handleMarkClick(2, true)} className={styles.mark} fillRule="evenodd" clipRule="evenodd" fill="#D3D3D3" points="74.2,54.3 74.2,44.9 80.1,44.9 77.6,54.3 	"/>
        <polygon id="t1" onClick={() => handleMarkClick(1, true)} className={styles.mark} fillRule="evenodd" clipRule="evenodd" fill="#D3D3D3" points="74.2,42.8 74.2,33.4 83.2,33.4 80.7,42.8 	"/>
        <polygon id="t0" onClick={() => handleMarkClick(0, true)} className={styles.mark} fillRule="evenodd" clipRule="evenodd" fill="#D3D3D3" points="86.3,22 83.7,31.3 74.2,31.3 74.2,22 	"/>
        <polygon id="b3" onClick={() => handleMarkClick(0, false)} className={styles.mark} fillRule="evenodd" clipRule="evenodd" fill="#D3D3D3" points="86.3,126.4 83.7,117 74.2,117 74.2,126.4 	"/>
        <polygon id="b2" onClick={() => handleMarkClick(1, false)} className={styles.mark} fillRule="evenodd" clipRule="evenodd" fill="#D3D3D3" points="74.2,105.5 74.2,114.9 83.2,114.9 80.7,105.5"/>
        <polygon id="b1" onClick={() => handleMarkClick(2, false)} className={styles.mark} fillRule="evenodd" clipRule="evenodd" fill="#D3D3D3" points="74.2,94 74.2,103.4 80.1,103.4 77.6,94 	"/>
        <polygon id="b0" onClick={() => handleMarkClick(3, false)} className={styles.mark} fillRule="evenodd" clipRule="evenodd" fill="#D3D3D3" points="74.2,82.5 74.2,91.9 77,91.9 74.5,82.5 	"/>
      </g>

      <g id="interactiveLabels">
        <text
          className={styles.label}
          transform="matrix(1 0 0 1 17.7052 79.8073)"
          fontFamily="'Roboto-Medium'"
          fontSize="18px"
          onClick={() => handleChange(0)}
        >
          0
        </text>
        <text
          className={styles.label}
          transform="matrix(1 0 0 1 4.5216 134.6168)"
          fontFamily="'Roboto-Medium'"
          fontSize="18px"
          onClick={() => handleChange(-1)}
        >
          -100
        </text>
        <text
          className={styles.label}
          transform="matrix(1 0 0 1 7.4747 24.9982)"
          fontFamily="'Roboto-Medium'"
          fontSize="18px"
          onClick={() => handleChange(1)}
        >
          100
        </text>
      </g>

      <g
        id="jumpButtonTop"
        className={styles.jumpButton}
      >
        <polygon
          id="jumpTopBorder"
          className={styles.jumpBorder}
          fill="none"
          stroke="#A5A5A5"
          strokeMiterlimit="10"
          points="92.6,0.5 76.8,0.5 68.9,8 76.8,15.5 92.6,15.5 100.4,8"
        />
        <polygon
          id="plusTop"
          fillRule="evenodd"
          clipRule="evenodd"
          fill="#EC008C"
          points="90.2,7.3 85.4,7.3 85.4,2.5 83.9,2.5 83.9,7.3 79.1,7.3 79.1,8.8 83.9,8.8 83.9,13.6 85.4,13.6 85.4,8.8 90.2,8.8"
          style={{ visibility: invert ? 'hidden' : 'visible' }}
        />
        <rect
          id="minusTop"
          x="79.1"
          y="7.3"
          fillRule="evenodd"
          clipRule="evenodd"
          fill="#376BE8"
          width="11.1"
          height="1.5"
          style={{ visibility: invert ? 'visible' : 'hidden' }}
        />
      </g>

      <g
        id="jumpButtonBottom"
        className={styles.jumpButton}
      >
        <polygon
          id="jumpBottomBorder"
          className={styles.jumpBorder}
          fill="white"
          stroke="#A5A5A5"
          strokeMiterlimit="10"
          points="92.6,133 76.8,133 68.9,140.5 76.8,148 92.6,148 100.4,140.5"
        />

        <polygon
          id="plusBottom"
          fillRule="evenodd"
          clipRule="evenodd"
          fill="#EC008C"
          points="90.2,139.7 85.4,139.7 85.4,134.9 83.9,134.9 83.9,139.7 79.1,139.7 79.1,141.2 83.9,141.2 83.9,146 85.4,146 85.4,141.2 90.2,141.2"
          style={{ visibility: invert ? 'visible' : 'hidden' }}
        />
      
        <rect
          id="minusBottom"
          x="79.1"
          y="139.7"
          fillRule="evenodd"
          clipRule="evenodd"
          fill="#376BE8"
          width="11.1"
          height="1.5"
          style={{ visibility: invert ? 'hidden' : 'visible' }}
        />
      </g>

      <rect
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
        <line fill="none" stroke="#A5A5A5" strokeMiterlimit="10" x1="60.3" y1="74.2" x2="45.5" y2="74.2"/>
        <line fill="none" stroke="#A5A5A5" strokeMiterlimit="10" x1="60.3" y1="101.6" x2="52.9" y2="101.6"/>
        <line fill="none" stroke="#A5A5A5" strokeMiterlimit="10" x1="60.3" y1="46.7" x2="52.9" y2="46.7"/>
        <polyline fill="none" stroke="#A5A5A5" strokeMiterlimit="10" points="45.5,129.1 60.3,129.1 60.3,129 60.3,19.2 45.5,19.2 	"/>
      </g>

      <g id="staticLabels">
        <text
          transform="matrix(1 0 0 1 109.4626 145.6881)"
          fill="#5B5B5B"
          fontFamily="'Roboto-Medium'"
          fontSize="18px"
        >
          RPWM
        </text>
        <text
          transform="matrix(1 0 0 1 110.2097 12.8253)"
          fill="#5B5B5B"
          fontFamily="'Roboto-Medium'"
          fontSize="18px"
        >
          LPWM
        </text>
      </g>

      <polygon
        id="sliderHead"
        className={styles.sliderHead}
        fill="#FFFFFF"
        stroke="#000000"
        strokeLinecap="round"
        strokeLinejoin="round"
        strokeMiterlimit="10"
        points="93.2,66 80.2,74.2 93.2,82.5 104.8,82.5 104.8,66"
      />
    </svg>
  );
};
