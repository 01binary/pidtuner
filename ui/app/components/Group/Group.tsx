import { FC, ReactElement } from "react";
import styles from "./Group.module.css";

type GroupProps = {
  children: ReactElement | ReactElement[];
  vertical?: boolean;
};

export const Group: FC<GroupProps> = ({ children, vertical }) => (
  <section
    className={[styles.group, vertical && styles.vertical].join(' ')}
  >
    {children}
  </section>
);
