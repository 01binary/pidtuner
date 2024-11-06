import { FC, ReactElement } from "react";
import styles from "./Group.module.css";

type GroupProps = {
  children: ReactElement | ReactElement[];
  vertical?: boolean;
  center?: boolean;
  alignTop?: boolean;
  marginTop?: boolean;
};

export const Group: FC<GroupProps> = ({
  children,
  vertical,
  center,
  alignTop,
  marginTop
}) => (
  <section
    className={[
      styles.group,
      vertical && styles.vertical,
      center && styles.center,
      alignTop && styles.top,
      marginTop && styles.marginTop
    ].filter(Boolean).join(' ')}
  >
    {children}
  </section>
);
