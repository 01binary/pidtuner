import { FC } from "react";
import { VAxis } from "./VAxis";
import styles from "./Performer.module.css";

export const Performer = () => {
  return (
    <section className={styles.performer}>
      <VAxis />
    </section>
  );
};
