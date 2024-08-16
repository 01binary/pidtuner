"use client";

import * as d3 from "d3";
import { useEffect, useMemo, useRef, useState } from "react";
import styles from "./Plot.module.css";
import data from "./sample.json";

const HEIGHT = 300;
const MARGIN_LEFT = 56;
const MARGIN_TOP = 8;
const MARGIN_RIGHT = 8;
const MARGIN_BOTTOM = 32;
const AXIS_LEFT_WIDTH = 64;
const AXIS_LEFT_OFFSET = 48;
const TICK_SIZE = 8;
const SPACING = 8;
const SPACING_HALF = 4;
const SAMPLE_WIDTH = 3;

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
    [SPACING_HALF, width - MARGIN_RIGHT]);

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
  const axisLeftRef = useRef<SVGGElement>(null);
  const axisBottomRef = useRef<SVGGElement>(null);

  const width = (data.length * SAMPLE_WIDTH) - MARGIN_LEFT - MARGIN_RIGHT;

  const x = d3.scaleLinear(
    [0, data[data.length - 1].time],
    [SPACING_HALF, width - SPACING_HALF]);

  const y = d3.scaleLinear(
    [-1, 1],
    [HEIGHT - MARGIN_BOTTOM, MARGIN_TOP]
  );

  useEffect(() => {
    d3
      .select(axisBottomRef.current)
      .call(d3.axisBottom(x).tickSize(TICK_SIZE))
      .call(g => g.select(".domain").remove());
  }, [x]);

  useEffect(() => {
    d3
      .select(axisLeftRef.current)
      .call(d3.axisLeft(y)
      .tickSize(TICK_SIZE));
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
    <div className={styles.plotArea}>
      <svg
        width={AXIS_LEFT_WIDTH}
        height={HEIGHT}
        className={styles.axisLeft}
      >
        <g
          ref={axisLeftRef}
          transform={`translate(${AXIS_LEFT_OFFSET}, 0)`}
        />
      </svg>

      <div className={styles.plotPane}>
        <svg
          width={width}
          height={HEIGHT}
          className={styles.plot}
        >
          <defs>
            <pattern
              id="dotFill"
              patternUnits="userSpaceOnUse"
              width="19.925"
              height="25.8"
            >
              <circle
                cx="5"
                cy="9"
                r="1.5"
                fill="#dddddd"
                stroke="none"
              />
            </pattern>
          </defs>

          <rect
            x={0}
            y={0}
            width={width ?? 0 - MARGIN_LEFT}
            height={HEIGHT - MARGIN_BOTTOM - MARGIN_TOP}
            fill="url(#dotFill)"
          />

          <g
            ref={axisBottomRef}
            className={styles.axisBottom}
            transform={`translate(0,${HEIGHT - MARGIN_BOTTOM + SPACING})`}
          />

          <line
            className={styles.axisBottom__domain}
            x1={SPACING_HALF}
            y1={HEIGHT - MARGIN_BOTTOM + SPACING}
            x2={width ?? 0 - MARGIN_RIGHT}
            y2={HEIGHT - MARGIN_BOTTOM + SPACING}
            strokeWidth="1"
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
      </div>
    </div>
  );
};
