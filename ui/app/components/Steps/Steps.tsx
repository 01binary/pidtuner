"use client";

import { FC, useCallback, useState } from "react";
import { Module } from "../Module";
import { Controls } from "./Controls";
import { Timeline } from "./Timeline";
import { formatSteps } from "./formatSteps";
import { StepCommand } from "@/app/useMotorControl";

const defaultSteps = [
  { value: 0 },
  { value: -0.5 },
  { value: 0 },
  { value: 0.5 },
  { value: 0 },
  { value: 0 }
];

type StepsProps = {
  time: number;
  step: number;
  isPlaying: boolean;
  setStep: React.Dispatch<React.SetStateAction<number>>;
  publishSteps: (command: StepCommand) => void;
};

export const Steps: FC<StepsProps> = ({
  time,
  step,
  isPlaying,
  setStep,
  publishSteps
}) => {
  const [steps, setSteps] = useState(defaultSteps);
  const [grid, setGrid] = useState(1);
  const [isLooping, setLooping] = useState(false);

  const handlePlay = useCallback(() => {
    const message = formatSteps(steps, grid, isLooping);
    publishSteps(message);
  }, [steps, grid, isLooping, publishSteps]);

  const handleStepSelect = useCallback((selectStep: number) => {
    setStep(selectStep);
  }, []);

  const handleStepChange = useCallback((
    stepIndex: number,
    value: number
  ) => {
    setSteps(steps => steps.map(
      (step, index) => index === stepIndex
        ? { value }
        : step
      ));
  }, []);

  const handleAddStep = useCallback(() => {
    setSteps(steps => [...steps, { value: 0 }])
  }, []);

  const handleRemoveStep = useCallback(() => {
    setSteps(steps => steps.slice(0, steps.length - 1));
  }, []);

  return (
    <Module
      title="Step"
      image={
        <img
          src="/step.svg"
          width="32"
          height="32"
        />
      }
    >
      <Controls
        step={step}
        grid={grid}
        isPlaying={isPlaying}
        onStepChange={setStep}
        onGridChange={setGrid}
        onPlay={handlePlay}
      />

      <Timeline
        time={time}
        start={0}
        grid={grid}
        steps={steps}
        currentStep={step}
        isPlaying={isPlaying}
        onSelect={handleStepSelect}
        onStepChange={handleStepChange}
        onAddStep={handleAddStep}
        onRemoveStep={handleRemoveStep}
      />
    </Module>
  );
};
