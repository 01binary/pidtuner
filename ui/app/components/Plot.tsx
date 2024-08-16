"use client";

import * as d3 from "d3";
import { useEffect, useMemo, useRef, useState } from "react";
import styles from "./Plot.module.css";
import data from "./sample.json";

const HEIGHT = 300;
const MARGIN_LEFT = 48;
const MARGIN_TOP = 8;
const MARGIN_RIGHT = 8;
const MARGIN_BOTTOM = 32;
const TICK_SIZE = 8;
const SPACING = 8;

type SeriesProps = {
  samples: number[],
  width: number | undefined,
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
  if (!width) return null;

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
  const axisLeftRef = useRef<SVGGElement>(null);
  const axisBottomRef = useRef<SVGGElement>(null);
  const [width, setWidth] = useState<number | undefined>();

  const x = d3.scaleLinear(
    [0, data[data.length - 1].time],
    [MARGIN_LEFT, width ?? 0 - MARGIN_RIGHT]);

  const y = d3.scaleLinear(
    [-1, 1],
    [HEIGHT - MARGIN_BOTTOM, MARGIN_TOP]
  );

  useEffect(() => {
    setWidth(plotRef.current?.clientWidth ?? 0)
  }, []);

  useEffect(() => {
    d3
      .select(axisBottomRef.current)
      .call(d3.axisBottom(x).tickSize(TICK_SIZE))
      .call(g => g.select(".domain").remove())
  }, [x]);

  useEffect(() => {
    d3
      .select(axisLeftRef.current)
      .call(d3.axisLeft(y)
      .tickSize(TICK_SIZE))
  }, [y]);

  const absolute = useMemo(() => data.map(({ absolute }) => absolute), []);
  const command = useMemo(() => data.map(({ command }) => command), []);

  const legend = [
    { samples: absolute, color: '#ec008c', label: 'absolute' },
    { samples: command, color: '#376be8', label: 'command' }
    // #795da3
  ]

  const layoutProps = {
    width,
    height: HEIGHT
  }

  return (
    <svg
      ref={plotRef}
      width={width}
      height={HEIGHT}
      className={styles.plot}
    >
      <g
        ref={axisBottomRef}
        className={styles.axisBottom}
        transform={`translate(0,${HEIGHT - MARGIN_BOTTOM + SPACING})`}
      />
      <line
        className={styles.axisBottom__domain}
        x1={MARGIN_LEFT}
        y1={HEIGHT - MARGIN_BOTTOM + SPACING}
        x2={width ?? 0 - MARGIN_RIGHT}
        y2={HEIGHT - MARGIN_BOTTOM + SPACING}
        strokeWidth="1"
      />
      <g
        ref={axisLeftRef}
        className={styles.axisLeft}
        transform={`translate(${MARGIN_LEFT - MARGIN_LEFT / 4},0)`}
      />
      {legend.map(({ samples, color, label }) => (
        <Series
          key={label}
          samples={samples}
          color={color}
          {...layoutProps}
        />
      ))}
    </svg>
  );
};
