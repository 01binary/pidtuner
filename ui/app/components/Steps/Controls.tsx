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
  <Group vertical alignTop marginTop>
    <PrimaryInput
      type="number"
      value={step}
      onChange={(e) => onStepChange(Number(e.target.value))}
      min={0}
      autoSize
    />

    <PrimaryInput
      id="grid"
      type="number"
      label="Grid (s)"
      value={grid}
      onChange={(e) => onGridChange(Number(e.target.value))}
      min={0}
    />

    <button
      title="Play"
      onClick={onPlay}
      style={{ maxWidth: '48px', marginTop: '24px' }}
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
