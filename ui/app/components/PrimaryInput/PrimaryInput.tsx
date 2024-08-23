import { FC } from "react";
import styles from "./PrimaryInput.module.css";

type PrimaryInputProps = {
  id?: string;
  type?: string;
  label?: string;
  units?: string;
  value?: any;
  autoSize?: boolean;
};

export const PrimaryInput: FC<PrimaryInputProps> = ({
  id,
  label,
  units,
  autoSize,
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
    {units
      ? (
          <div>
            <input
              id={id}
              className={[
                styles.primaryInput,
                autoSize && styles.autoSize
              ].filter(Boolean).join(' ')}
              {...props}
            />
            <span className={styles.units}>
              {units}
            </span>
          </div>
      )
      : (
        <input
          id={id}
          className={[
            styles.primaryInput,
            autoSize && styles.autoSize
          ].filter(Boolean).join(' ')}
          {...props}
        />
      )
    }
  </>
);
