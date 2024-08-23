import { FC, ReactElement } from "react";
import styles from "./Group.module.css";

type GroupProps = {
  children: ReactElement | ReactElement[];
};

export const Group: FC<GroupProps> = ({ children }) => (
  <section className={styles.group}>
    {children}
  </section>
);
