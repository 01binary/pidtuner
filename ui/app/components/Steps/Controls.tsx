import { FC } from "react";
import { Group } from "../Group";
import { PrimaryInput } from "../PrimaryInput";

type ControlsProps = {
  step: number;
  grid: number;
  isPlaying: boolean;
  onStepChange: (step: number) => void;
  onGridChange: (grid: number) => void;
  onPlay: () => void;
};

export const Controls: FC<ControlsProps> = ({
  step,
  grid,
  onStepChange,
  onGridChange,
  onPlay
}) => (
  <Group vertical>
    <PrimaryInput
      type="number"
      value={step}
      onChange={(e) => onStepChange(Number(e.target.value))}
      min={0}
      autoSize
    />

    <PrimaryInput
      type="number"
      label="Grid"
      value={grid}
      units="s"
      onChange={(e) => onGridChange(Number(e.target.value))}
      min={0}
      autoSize
    />

    <button
      title="Play"
      onClick={onPlay}
      style={{ maxWidth: '48px'}}
    >
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
  </Group>
);
