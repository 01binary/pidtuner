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
    height="57"
    viewBox="0 0 45 57"
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
      points="43.9 1.4 27.4 17.9 .9 17.9"
      fill="none" stroke="#d3d3d3"
      strokeLinecap="round"
      strokeLinejoin="round"
      strokeWidth="2"
    />
    <polyline
      points="43.9 55.7 27.4 39.2 .9 39.2"
      fill="none"
      stroke="#d3d3d3"
      strokeLinecap="round"
      strokeLinejoin="round"
      strokeWidth="2"
    />
  </svg>
);
