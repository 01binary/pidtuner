import { FC, useCallback } from "react";
import { ConfigurationCommand } from "@/app/useMotorControl";
import { Module } from "../Module";
import { SETTINGS } from "./settings";
import styles from "./Configuration.module.css";

type RowProps = {
  name: string;
  label: string;
  type: string;
  value: any;
  onChange: (name: string, value: any) => void;
  min?: number;
  max?: number;
  step?: number;
};

const Row: FC<RowProps> = ({
  name,
  label,
  type,
  value,
  onChange,
  ...options
}) => (
  <tr className={styles.setting}>
    <td className={styles.key}>
      {label}
    </td>
    <td className={styles.value}>
      {type === 'number'
        ? (
            <input
              type='number'
              {...options}
              value={value}
              onChange={(e) => onChange(name, e.target.valueAsNumber)}
            />
          )
        : (
            <div>boolean</div>
          )
      }
    </td>
  </tr>
)

type ConfigurationProps = {
  publishConfiguration: (configuration: ConfigurationCommand) => void;
  configuration: ConfigurationCommand
};

export const Configuration: FC<ConfigurationProps> = ({
  configuration,
  publishConfiguration
}) => {
  const handleChangeValue = useCallback((name: string, value: any) => {
    publishConfiguration({
      ...configuration,
      [name]: value
    } as ConfigurationCommand)
  }, [configuration, publishConfiguration]);

  return (
    <Module
      title="Configuration"
      image={<img src="/configuration.svg" width="32" height="32" />}
    >
      <table cellSpacing="0" className={styles.settings}>
        <tbody>
          {SETTINGS.map(({ name, ...setting}) => (
            <Row
              key={name}
              name={name}
              value={configuration[name]}
              onChange={handleChangeValue}
              {...setting}
            />
          ))}
        </tbody>
      </table>
    </Module>
  );
};
