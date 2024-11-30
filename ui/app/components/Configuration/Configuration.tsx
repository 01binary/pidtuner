import { FC } from "react";
import { Module } from "../Module";
import styles from "./Configuration.module.css";

const SETTINGS = [
  {
    key: 'LPWMpin',
    label: 'LPWM Pin',
    type: 'number',
    min: 0,
    max: 100,
    step: 1
  },
  {
    key: 'RPWMpin',
    label: 'RPWM Pin',
    type: 'number',
    min: 0,
    max: 100,
    step: 1
  },
  {
    key: 'ADCpin',
    label: 'ADC Pin',
    type: 'number',
    min: 0,
    max: 15,
    step: 1
  },
  {
    key: 'csPin',
    label: 'CS Pin',
    type: 'number',
    min: 0,
    max: 100,
    step: 1
  },
  {
    key: 'Apin',
    label: 'A Pin',
    type: 'number',
    min: 0,
    max: 100,
    step: 1
  },
  {
    key: 'Bpin',
    label: 'B Pin',
    type: 'number',
    min: 0,
    max: 100,
    step: 1
  },
  {
    key: 'pwmMin',
    label: 'PWM Min',
    type: 'number',
    min: -1000,
    max: 1000,
    step: 1
  },
  {
    key: 'pwmMax',
    label: 'PWM Max',
    type: 'number',
    min: -1000,
    max: 1000,
    step: 1
  },
  {
    key: 'absoluteMin',
    label: 'Absolute Min',
    type: 'number',
    min: -1000,
    max: 1000,
    step: 0.001
  },
  {
    key: 'absoluteMax',
    label: 'Absolute Max',
    type: 'number',
    min: -1000,
    max: 1000,
    step: 0.001
  },
  {
    key: 'pwmInvert',
    label: 'PWM Invert',
    type: 'boolean'
  },
  {
    key: 'absoluteInvert',
    label: 'Absolute Invert',
    type: 'boolean'
  },
  {
    key: 'quadratureInvert',
    label: 'Quadrature Invert',
    type: 'boolean'
  },
  {
    key: 'Kp',
    label: 'Proportional gain',
    type: 'number',
    min: -10000,
    max: 10000,
    step: 0.001
  },
  {
    key: 'Ki',
    label: 'Integral gain',
    type: 'number',
    min: -10000,
    max: 10000,
    step: 0.001
  },
  {
    key: 'Kd',
    label: 'Derivative gain',
    type: 'number',
    min: -10000,
    max: 10000,
    step: 0.001
  },
  {
    key: 'iMin',
    label: 'Integral Min',
    type: 'number',
    min: -10000,
    max: 10000,
    step: 0.001
  },
  {
    key: 'iMax',
    label: 'Integral Max',
    type: 'number',
    min: -10000,
    max: 10000,
    step: 0.001
  }
];

type RowProps = {
  key: string;
  label: string;
  type: string;
  value: number | boolean;
  min?: number;
  max?: number;
  step?: number;
};

const Row: FC<RowProps> = ({
  key,
  label,
  type,
  value,
  ...options
}) => (
  <tr className={styles.setting}>
    <td className={styles.setting}>
      {label}
    </td>
    <td className={styles.value}>
      {type === "number"
        ? (
            <input type="number" {...options} value={value as number} />
          )
        : (
            <input type="checkbox" checked={value as boolean} />
          )
      }
    </td>
  </tr>
)

type ConfigurationProps = {
  values: {
    [key: string]: number | boolean;
  }
};

export const Configuration: FC<ConfigurationProps> = ({
  values
}) => (
  <Module
    title="Configuration"
    image={<img src="/configuration.svg" width="32" height="32" />}
  >
    <table className={styles.settings}>
    {SETTINGS.map(({ key, ...setting}) => (
      <Row
        key={key}
        {...setting}
        value={values[key]}
      />
    ))}
    </table>
  </Module>
);
