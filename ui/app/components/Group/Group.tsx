import { FC, ReactElement } from "react";
import styles from "./Group.module.css";

type GroupProps = {
  children: ReactElement | ReactElement[];
  vertical?: boolean;
  center?: boolean;
  alignTop?: boolean;
};

export const Group: FC<GroupProps> = ({
  children,
  vertical,
  center,
  alignTop
}) => (
  <section
    className={[
      styles.group,
      vertical && styles.vertical,
      center && styles.center,
      alignTop && styles.top
    ].filter(Boolean).join(' ')}
  >
    {children}
  </section>
);
