import { ReactNode, FC } from "react";
import styles from "./Module.module.css";

type ModuleProps = {
  title: string;
  image: ReactNode;
  children?: ReactNode
};

export const Module: FC<ModuleProps> = ({
  title,
  image,
  children
}) => (
  <section className={styles.module}>
    <section className={styles["module__header"]}>
      {image}
      <h2>{title}</h2>
    </section>

    <section className={styles["module__controls"]}>
      {children}
    </section>
  </section>
);
