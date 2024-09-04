import { StepType } from "./StepType";

export const formatSteps = (steps: StepType[], grid: number) => {
  return steps.map(({ value }) => ({
    command: value,
    duration: grid
  }))
};
