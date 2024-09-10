import * as d3 from "d3";
import { FC } from "react";
import {
  SPACING_HALF,
  MARGIN_RIGHT,
  MARGIN_BOTTOM,
  MARGIN_TOP
} from "./constants";

type Sample = {
  [property: string]: number;
};

type SeriesProps = {
  property: string;
  samples: Sample[];
  min?: number;
  max?: number;
  width: number | undefined,
  height: number,
  strokeWidth?: number,
  color: string
};

export const Series: FC<SeriesProps> = ({
  samples,
  property,
  min,
  max,
  width,
  height,
  strokeWidth = 1,
  color
}) => {
  if (!width) return null;

  const series = samples.map(s => s[property]);

  const x = d3.scaleLinear(
    [0, samples.length - 1],
    [SPACING_HALF, width - MARGIN_RIGHT]);

  const y = d3.scaleLinear(
    min !== undefined && max !== undefined
      ? [min, max]
      : d3.extent(series),
    [height - MARGIN_BOTTOM, MARGIN_TOP]
  );

  const line = d3.line((point: unknown, index: number) => x(index), y);

  return (
    <path
      fill="none"
      stroke={color}
      strokeWidth={strokeWidth}
      d={line(series)}
    />
  )
};
