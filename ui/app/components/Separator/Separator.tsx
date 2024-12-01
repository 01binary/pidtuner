import { FC } from "react";
import styles from "./Separator.module.css";

type SeparatorProps = {
  invert?: boolean;
  spacing?: string;
  spacingLeft?: string;
  spacingRight?: string;
}

export const Separator: FC<SeparatorProps> = ({
  invert,
  spacing,
  spacingLeft,
  spacingRight
}) => (
  <svg
    width="45"
    height="77"
    viewBox="0 0 45 77"
    className={[
      styles.separator,
      invert && styles['separator--invert']
    ].filter(Boolean).join(' ')}
    style={{
      marginLeft: spacing ?? spacingLeft,
      marginRight: spacing ?? spacingRight
    }}
  >
    <polyline
      points="43.9,1.3 27.4,17.8 0.9,17.8"
      fill="none" stroke="#d3d3d3"
      strokeLinecap="round"
      strokeLinejoin="round"
      strokeWidth="2"
    />
    <polyline
      points="43.9,75.6 27.4,59.1 0.9,59.1"
      fill="none"
      stroke="#d3d3d3"
      strokeLinecap="round"
      strokeLinejoin="round"
      strokeWidth="2"
    />
  </svg>
);
