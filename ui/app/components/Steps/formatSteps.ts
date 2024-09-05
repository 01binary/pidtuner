import { StepType } from "./StepType";

export const formatSteps = (
  steps: StepType[],
  grid: number,
  loop: boolean
) => ({
  loop,
  steps: steps.map(({ value }) => ({
    command: value,
    duration: grid
  }))
});
