import { FC, ReactElement } from "react";
import styles from "./Group.module.css";

type GroupProps = {
  children: ReactElement | ReactElement[];
  vertical?: boolean;
  center?: boolean;
};

export const Group: FC<GroupProps> = ({
  children,
  vertical,
  center
}) => (
  <section
    className={[
      styles.group,
      vertical && styles.vertical,
      center && styles.center
    ].filter(Boolean).join(' ')}
  >
    {children}
  </section>
);
