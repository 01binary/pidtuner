"use client";

import { useCallback, useState } from "react";
import { Module } from "../Module";
import { Controls } from "./Controls";
import { Timeline } from "./Timeline";
import { formatSteps } from "./formatSteps";

const defaultSteps = [
  { value: 0 },
  { value: -0.2 },
  { value: 0 },
  { value: 0.2 },
  { value: 0.5 }
];

export const Steps = () => {
  const [steps, setSteps] = useState(defaultSteps);
  const [step, setStep] = useState(0);
  const [grid, setGrid] = useState(1);
  const [isPlaying, setPlaying] = useState(false);
  const [time, setTime] = useState(0);

  const handlePlay = useCallback(() => {
    setPlaying(true);
    const message = formatSteps(steps, grid);
    console.log(message);
  }, [steps, grid]);

  const handleStop = useCallback(() => {
    setPlaying(false);
  }, []);

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
        onStop={handleStop}
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
