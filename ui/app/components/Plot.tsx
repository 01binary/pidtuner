"use client";

import * as d3 from "d3";
import { useEffect, useMemo, useRef, useState } from "react";
import styles from "./Plot.module.css";
import data from "./sample.json";

const MARGIN_LEFT = 0;
const MARGIN_TOP = 0;
const MARGIN_RIGHT = 0;
const MARGIN_BOTTOM = 0;

type SeriesProps = {
  samples: number[],
  width: number,
  height: number,
  strokeWidth?: number,
  color: string
};

const Series = ({
  samples,
  width,
  height,
  strokeWidth = 1,
  color
}: SeriesProps) => {
  const x = d3.scaleLinear(
    [0, samples.length - 1],
    [MARGIN_LEFT, width - MARGIN_RIGHT]);

  const y = d3.scaleLinear(
    d3.extent(samples),
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
}

export const Plot = () => {
  const plotRef = useRef<SVGSVGElement>(null);
  const [plotSize, setPlotSize] = useState<{ width: number; height: number }>({});

  useEffect(() => {
    setPlotSize({
      width: plotRef.current?.clientWidth ?? 0,
      height: plotRef.current?.clientHeight ?? 0
    })
  }, []);

  const { width, height } = plotSize;
  const absolute = useMemo(() => data.map(({ absolute }) => absolute), []);
  const command = useMemo(() => data.map(({ command }) => command), []);

  return (
    <svg ref={plotRef} width={width} height={height}>
      <Series samples={absolute} width={width} height={height} color="red" />
      <Series samples={command} width={width} height={height} color="blue" />
    </svg>
  );
};
