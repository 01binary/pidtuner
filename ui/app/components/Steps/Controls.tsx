import { FC } from "react";
import { Group } from "../Group";
import { PrimaryInput } from "../PrimaryInput";
import { Separator } from "../Separator";

type ControlsProps = {
  step: number;
  grid: number;
  isPlaying: boolean;
  onStepChange: (step: number) => void;
  onGridChange: (grid: number) => void;
  onPlay: () => void;
  onStop: () => void;
};

export const Controls: FC<ControlsProps> = ({
  step,
  grid,
  onStepChange,
  onGridChange,
  onPlay,
  onStop
}) => (
  <Group center>
    <PrimaryInput
      type="number"
      value={step}
      onChange={(e) => onStepChange(Number(e.target.value))}
      min={0}
      autoSize
    />

    <Separator />

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
