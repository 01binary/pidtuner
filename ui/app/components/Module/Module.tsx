import { ReactNode, FC, useState, useCallback } from "react";
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
}) => {
  const [expanded, setExpanded] = useState(true);

  const handleToggleHeader = useCallback(() => {
    setExpanded(e => !e);
  }, []);

  return (
    <section className={styles.module}>
      <section className={styles.header}>
        {image}
        <h2 onClick={handleToggleHeader}>
          {title}
        </h2>
      </section>

      {expanded && <section className={styles.controls}>
        {children}
      </section>}
    </section>
  );
}
