"use client";

import { FC, useCallback, useState } from "react";
import { Group } from "../Group";
import { Module } from "../Module";
import { PrimaryInput } from "../PrimaryInput";
import { Separator } from "../Separator";
import { Timeline } from "./Timeline";

type ControlsProps = {
  step: number;
  grid: number;
  isPlaying: boolean;
  onStepChange: (step: number) => void;
  onGridChange: (grid: number) => void;
  onPlay: () => void;
  onStop: () => void;
};

const Controls: FC<ControlsProps> = ({
  step,
  grid,
  onStepChange,
  onGridChange,
  onPlay,
  onStop
}) => (
  <Group center>
    <Group vertical>
      <PrimaryInput
        type="number"
        value={step}
        onChange={(e) => onStepChange(e.target.value)}
        autoSize
      />

      <PrimaryInput
        type="number"
        label="Grid"
        units="s"
        value={grid}
        min={0.1}
        max={100}
        onChange={(e) => onGridChange(e.target.value)}
        autoSize
      />
    </Group>

    <Group vertical>
      <button title="Play" onClick={onPlay}>
        <svg
          width="24px"
          height="24px"
          viewBox="0 0 24 24"
        >
          <polygon
            fill="#376be8"
            points="22.4,12 1.6,0 1.6,23.9 "
          />
        </svg>
      </button>

      <button title="Stop" onClick={onStop}>
        <svg
          width="24px"
          height="24px"
          viewBox="0 0 24 24"
        >
          <rect
            x="1.2"
            y="1.2"
            fill="#EC008C"
            width="21.6"
            height="21.6"
          />
        </svg>
      </button>
    </Group>
  </Group>
);

export const Steps = () => {
  const [step, setStep] = useState(0);
  const [position, setPosition] = useState(0);
  const [grid, setGrid] = useState(1);
  const [isPlaying, setPlaying] = useState(false);

  const handlePlay = useCallback(() => {
    setPlaying(true);
  }, []);

  const handleStop = useCallback(() => {
    setPlaying(false);
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
        step={step}
        position={position}
        isPlaying={isPlaying}
      />
    </Module>
  );
};
