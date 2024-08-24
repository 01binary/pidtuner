"use client";

import { FC, useState } from "react";
import { Group } from "../Group";
import { Module } from "../Module";
import { PrimaryInput } from "../PrimaryInput";
import { Separator } from "../Separator";
import { Performer } from "./Performer";

type ControlsProps = {
  step: number;
  grid: number;
  onGridChange: (grid: number) => void;
};

const Controls: FC<ControlsProps> = ({
  step,
  grid,
  onGridChange,
}) => (
  <Group center>
    <Group vertical>
      <PrimaryInput
        type="number"
        value={step}
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

    <Separator spacing={'16px'} />

    <Group vertical>
      <button>
        <svg
          width="24px"
          height="24px"
          viewBox="0 0 24 24"
        >
          <polygon
            fill="white"
            points="22.4,12 1.6,0 1.6,23.9 "
          />
        </svg>
      </button>

      <button>
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

    <Separator />
  </Group>
);

export const Steps = () => {
  const [step, setStep] = useState(0);
  const [grid, setGrid] = useState(1);

  return (
    <Module
      title="Step"
      image={<img src="/step.svg" width="32" height="32" />}
    >
      <Controls
        step={step}
        grid={grid}
        onGridChange={setGrid}
      />

      <Performer />
    </Module>
  );
};
