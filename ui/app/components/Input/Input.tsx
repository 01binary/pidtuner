import { ChangeEventHandler, FC } from "react";
import styles from "./Input.module.css";

type InputProps = {
  id?: string;
  type?: string;
  label?: string;
  value?: string;
  onChange?: ChangeEventHandler;
};

export const Input: FC<InputProps> = ({
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
          className={styles.input}
          {...props}
        />
  </>
);
