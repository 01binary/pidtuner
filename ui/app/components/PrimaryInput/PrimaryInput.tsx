import { FC } from "react";
import styles from "./PrimaryInput.module.css";

type PrimaryInputProps = {
  id?: string;
  label?: string;
  value?: any;
};

export const PrimaryInput: FC<PrimaryInputProps> = ({
  id,
  label,
  ...props
}) => (
  <>
    {label &&
      <label
        className={styles.label}
        htmlFor={id}
      >
        {label}
      </label>
    }
    <input
      id={id}
      className={styles.primaryInput}
      {...props}
    />
  </>
);
