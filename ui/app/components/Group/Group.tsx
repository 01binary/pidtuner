import { FC, ReactElement } from "react";
import styles from "./Group.module.css";

type GroupProps = {
  children: ReactElement | ReactElement[];
  vertical?: boolean;
  center?: boolean;
  alignTop?: boolean;
  marginTop?: boolean;
  tight?: boolean;
};

export const Group: FC<GroupProps> = ({
  children,
  vertical,
  center,
  alignTop,
  marginTop,
  tight
}) => (
  <section
    className={[
      styles.group,
      vertical && styles.vertical,
      center && styles.center,
      alignTop && styles.top,
      marginTop && styles.marginTop,
      tight ? styles.tight : styles.relaxed
    ].filter(Boolean).join(' ')}
  >
    {children}
  </section>
);
