import { FC } from "react";
import styles from "./Performer.module.css";

type StepProps = {
  from: number;
  to: number;
  isCurrentStep: boolean;
};

export const Step: FC<StepProps> = ({
  from,
  to,
  isCurrentStep
}) => {
  return (
    <div
      className={[
        styles.step,
        isCurrentStep && styles.current
      ].filter(Boolean).join(' ')}
    >
      <div className={styles.stepLabel}>
        {(to !== undefined && from !== to)
          ? `${from} -> ${to}`
          : from}
      </div>

      <svg
        width="145.4px"
        height="260.9px"
        viewBox="0 0 145.4 260.9"
      >
        <path
          className={styles.stepBorder}
          d="M140.7,260.9H4.7c-2.6,0-4.7-2.1-4.7-4.7V4.7C0,2.1,2.1,0,4.7,0h136.1c2.6,0,4.7,2.1,4.7,4.7v251.6
            C145.4,258.8,143.3,260.9,140.7,260.9z"
        />

        <line
          className={styles.stepCenter}
          fill="none"
          strokeMiterlimit="10"
          strokeDasharray="4,6"
          x1="0"
          y1="128.9"
          x2="145.4"
          y2="128.9"
        />

        <g
          id="handleLeft"
          className={styles.controlPoint}
          style={{ visibility: isCurrentStep ? 'visible' : 'hidden' }}
        >
          <rect x="0" y="123" width="13" height="13" fill="white" />
          <path
            fill="#FFFFFF"
            d="M12.1,123.3v11.1H1v-11.1H12.1 M13.1,122.3H0v13.1h13.1V122.3L13.1,122.3z"
          />
        </g>

        <g
          id="handleRight"
          className={styles.controlPoint}
          style={{ visibility: isCurrentStep ? 'visible' : 'hidden' }}
        >
          <rect x="132" y="123" width="13" height="13" fill="white" />
          <path
            fill="#FFFFFF"
            d="M144.4,123.3v11.1h-11.1v-11.1H144.4 M145.4,122.3h-13.1v13.1h13.1V122.3L145.4,122.3z"
          />
        </g>

        <line
          id="valueLine"
          display="none"
          fill="none"
          stroke="#707070"
          strokeMiterlimit="10"
          x1="6.6"
          y1="128.9"
          x2="138.8"
          y2="128.9"
        />
      </svg>
    </div>
  );
};
