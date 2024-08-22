"use client";

import * as d3 from "d3";
import { FC, useCallback, useEffect, useMemo, useRef } from "react";
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

const Series: FC<SeriesProps> = ({
  samples,
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
};

type LegendItem = {
  color: string,
  label: string,
  samples: number[],
};

type LegendProps = {
  legend: LegendItem[]
};

const Legend: FC<LegendProps> = ({ legend }) => (
  <section className={styles.legend}>
    {legend.map(({ color, label }) => (
      <div key={label} className={styles.legendItem}>
        <div
          className={styles.legendColor}
          style={{ backgroundColor: color }}
        />
        <div className={styles.legendText}>
          {label}
        </div>
      </div>
    ))}
  </section>
);

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

  const handleExport = useCallback(() => {
    const text = data
      .reduce((lines, { time, command, absolute }) => ([
        ...lines,
        `${time},${command},${absolute}`
      ]), ['time,command,absolute'])
      .join('\r\n');

      const element = document.createElement('a');
      element.setAttribute('href', 'data:text/plain;charset=utf-8,' + encodeURIComponent(text));
      element.setAttribute('download', 'output.csv');
      element.style.display = 'none';
      document.body.appendChild(element);

      element.click();

      document.body.removeChild(element);
  }, []);

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
    <>
      <section className={styles.plotHeader}>
        <img src="icon.svg" width="48" height="48" />

        <h1>Motor Control</h1>

        <div className={styles.plotToolbar}>
          <button
            onClick={handleExport}
            title="Export measurements"
          >
            Export
          </button>
          <button
            className="warning estop"
            title="Emergency stop"
          >
            <img src="/estop.svg" width="24" height="24" />
          </button>
        </div>
      </section>

      <section className={styles.plotStrip}>
        {/* Left Axis that doesn't scroll */}
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

        {/* Scrolling plot */}
        <div className={styles.plotScroll}>
          <svg
            width={width}
            height={HEIGHT}
            className={styles.plot}
          >
            {/* Dot pattern */}
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

            {/* Rectangle that fills the plot with dot pattern */}
            <rect
              width={width ?? 0 - MARGIN_LEFT}
              height={HEIGHT - MARGIN_BOTTOM - MARGIN_TOP}
              fill="url(#dotFill)"
            />

            {/* Bottom axis tick marks */}
            <g
              ref={axisBottomRef}
              className={styles.axisBottom}
              transform={`translate(0,${HEIGHT - MARGIN_BOTTOM + SPACING})`}
            />

            {/* Bottom axis domain */}
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
      </section>

      <Legend legend={legend} />
    </>
  );
};
