"use client";

import * as d3 from "d3";
import React, {
  FC,
  useCallback,
  useEffect,
  useMemo,
  useRef,
  ChangeEventHandler
} from "react";
import { Input } from "../Input";
import { ControlMode } from "@/app/useMotorControl";
import { PlotType } from "./PlotType";
import styles from "./Plot.module.css";
import { exportSamples } from "./exportSamples";
import { formatSamples } from "./formatSamples";
import { Series } from "./Series";
import { Legend } from "./Legend";
import {
  SAMPLE_WIDTH,
  MARGIN_LEFT,
  MARGIN_RIGHT,
  TICK_SIZE,
  HEIGHT,
  MARGIN_TOP,
  MARGIN_BOTTOM,
  AXIS_LEFT_WIDTH,
  AXIS_LEFT_OFFSET,
  SPACING_HALF,
  SPACING
} from "./constants";

const MODE = ['velocity', 'position', 'step'];

const LEGEND = [
  { key: 'absolute', color: '#ec008c', label: 'absolute', min: 0, max: 1 },
  { key: 'command', color: '#376be8', label: 'command', min: -1, max: 1 }
  // #795da3
];

type PlotProps = {
  server: string;
  onServerChange: ChangeEventHandler<HTMLInputElement>;
  data: PlotType[];
  mode: ControlMode;
  isConnected: boolean;
  isCapturing: boolean;
  isEmergencyStop: boolean;
  setCapturing: React.Dispatch<React.SetStateAction<boolean>>;
  onEStop: () => void;
};

export const Plot: FC<PlotProps> = ({
  server,
  onServerChange,
  isConnected,
  isCapturing,
  setCapturing,
  isEmergencyStop,
  mode,
  data,
  onEStop
}) => {
  const scrollRef = useRef<HTMLDivElement>(null);
  const axisLeftRef = useRef<SVGGElement>(null);
  const axisBottomRef = useRef<SVGGElement>(null);

  const width = useMemo(() => (
    (data.length * SAMPLE_WIDTH) - MARGIN_LEFT - MARGIN_RIGHT
  ), [data]);

  const x = useMemo(() => d3.scaleLinear(
    [0, data[data.length - 1]?.time ?? 0],
    [SPACING_HALF, width - SPACING_HALF]
  ), [data, width]);

  const y = useMemo(() => d3.scaleLinear(
    [-1, 1],
    [HEIGHT - MARGIN_BOTTOM, MARGIN_TOP]
  ), []);

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

  useEffect(() => {
    if (scrollRef.current) {
      scrollRef.current.scrollTo(width, 0);
    }
  }, [width]);

  const handleExport = useCallback(() => {
    const contents = formatSamples(data);
    exportSamples(contents);
  }, [data]);

  const handleToggleCapture = useCallback(() => {
    setCapturing(value => !value);
  }, []);

  const layoutProps = {
    width,
    height: HEIGHT
  }

  return (
    <>
      <section className={styles.plotHeader}>
        <img src="icon.svg" width="48" height="48" />

        <h1>
          Motor Control
          <span className={styles.mode}>
            {' '} / {MODE[mode]}
          </span>
        </h1>

        <div className={styles.plotToolbar}>
          <Input
            id="server"
            label="server"
            type="text"
            value={server}
            onChange={onServerChange}
          />

          {isConnected
            ? <img width="32" height="32" src="/bridge-on.svg" />
            : <img width="32" height="32" src="/bridge-off.svg" />
          }

          <div className={styles.toolbarGroup}>
            <Legend legend={LEGEND} />
          </div>

          <button
            title="Enable capture"
            onClick={handleToggleCapture}
          >
            {isCapturing
              ? <img src="/pause.svg" width="24" height="24" />
              : <img src="/record.svg" width="24" height="24" />
            }
          </button>

          <button
            onClick={handleExport}
            title="Export measurements"
          >
            <img src="/export.svg" width="24" height="24" />
          </button>

          <button
            className="warning estop"
            title="Emergency stop"
            onClick={onEStop}
          >
            {isEmergencyStop
              ? <img src="/estop-on.svg" width="24" height="24" />
              : <img src="/estop.svg" width="24" height="24" />
            }
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
        <div
          ref={scrollRef}
          className={styles.plotScroll}
        >
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

            {LEGEND.map(({ key, color, min, max }) => (
              <Series
                key={key}
                property={key}
                samples={data}
                color={color}
                min={min}
                max={max}
                {...layoutProps}
              />
            ))}
          </svg>
        </div>
      </section>
    </>
  );
};
