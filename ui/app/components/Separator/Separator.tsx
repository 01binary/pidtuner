/* eslint-disable @next/next/no-img-element */

import styles from "./Separator.module.css";

type SeparatorProps = {
  invert?: boolean;
}

export const Separator = ({ invert }: SeparatorProps) => (
  <img
    className={[
      styles.separator,
      invert && styles['separator--invert']
    ].filter(Boolean).join(' ')}
    src="./separator.svg"
    width="45"
    height="57"
  />
);
