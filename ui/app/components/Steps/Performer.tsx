import { FC } from "react";
import { VAxis } from "./VAxis";
import { Step } from "./Step";
import styles from "./Performer.module.css";

const steps = [
  { from: 0, to: 0 },
  { from: -20, to: -20 },
  { from: 0, to: 0 },
  { from: 20, to: 20 },
  { from: 5, to: 5 }
];

type PerformerProps = {
  step: number;
};

export const Performer: FC<PerformerProps> = ({ step }) => {
  return (
    <section className={styles.performer}>
      <VAxis />
      {steps.map(({ from, to }, index) => (
        <Step
          key={index}
          from={from}
          to={to}
          isCurrentStep={index === step}
        />
      ))}
    </section>
  );
};
