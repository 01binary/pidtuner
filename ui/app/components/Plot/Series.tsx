import * as d3 from "d3";
import { FC } from "react";
import {
  SPACING_HALF,
  MARGIN_RIGHT,
  MARGIN_BOTTOM,
  MARGIN_TOP
} from "./constants";

type SeriesProps = {
  samples: number[],
  min?: number;
  max?: number;
  width: number | undefined,
  height: number,
  strokeWidth?: number,
  color: string
};

export const Series: FC<SeriesProps> = ({
  samples,
  min,
  max,
  width,
  height,
  strokeWidth = 1,
  color
}) => {
  if (!width) return null;

  const x = d3.scaleLinear(
    [0, samples.length - 1],
    [SPACING_HALF, width - MARGIN_RIGHT]);

  const y = d3.scaleLinear(
    min !== undefined && max !== undefined
      ? [min, max]
      : d3.extent(samples),
    [height - MARGIN_BOTTOM, MARGIN_TOP]
  );

  const line = d3.line((point: unknown, index: number) => x(index), y);

  return (
    <path
      fill="none"
      stroke={color}
      strokeWidth={strokeWidth}
      d={line(samples)}
    />
  )
};
