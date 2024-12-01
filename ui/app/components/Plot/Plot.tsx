"use client";

import * as d3 from "d3";
import React, {
  FC,
  useState,
  useCallback,
  useEffect,
  useMemo,
  useRef,
  ChangeEventHandler
} from "react";
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
  SPACING,
  MIN_WIDTH
} from "./constants";

const MODE = ["Velocity", "Position", "Step"];

const LEGEND = [
  { key: "command", color: "#376be8", label: "command", min: -1, max: 1 },
  { key: "absolute", color: "#ec008c", label: "absolute", min: 0, max: 1 },
  { key: "quadrature", color: "#795da3", label: "quadrature" },
  { key: "goal", color: "#00ccbe", label: "goal" }
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
  const [enabled, setEnabled] = useState<{ [key: string]: boolean | undefined }>({
    command: true,
    absolute: true,
    quadrature: true,
    goal: true
  });

  const width = useMemo(() => (
    Math.max(data.length * SAMPLE_WIDTH, MIN_WIDTH) - MARGIN_LEFT - MARGIN_RIGHT
  ), [data]);

  const x = useMemo(() => d3.scaleLinear(
    [0, data[data.length - 1]?.time ?? 0],
    [SPACING_HALF, width - SPACING_HALF]
  ), [data, width]);

  const y = useMemo(() => d3.scaleLinear(
    [-1, 1],
    [HEIGHT - MARGIN_BOTTOM, MARGIN_TOP]
  ), []);

  const quadratureToAbsoluteRef = useRef(0);
  const lengthRef = useRef(0);

  useEffect(() => {
    if (data.length < 2) return;

    let lastAbsolute = data[lengthRef.current].absolute;
    let lastQuadrature = data[lengthRef.current].quadrature;
    let avgDa = 0;
    let avgDq = 0;

    for (let n = lengthRef.current + 1; n < data.length; n++) {
      const { absolute, quadrature } = data[n];

      const da = Math.abs(lastAbsolute - absolute);
      const dq = Math.abs(lastQuadrature - quadrature);

      if (da < 0.000001 || dq < 0.000001) continue;

      avgDa = (da + avgDa) / 2;
      avgDq = (dq + avgDq) / 2;

      lastAbsolute = absolute;
      lastQuadrature = quadrature;
    }

    if (avgDq === 0) return;

    const dadq = avgDa / avgDq;

    quadratureToAbsoluteRef.current =
      (quadratureToAbsoluteRef.current + dadq) / 2;

    lengthRef.current = data.length;
  }, [data]);

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

  const handleToggleLegendSeries = useCallback((key: string, enabled: boolean) => {
    setEnabled(current => ({
      ...current,
      [key]: enabled
    }))
  }, []);

  const layoutProps = {
    width,
    height: HEIGHT
  }

  return (
    <>
      <section className={styles.plotHeader}>
        <img src="icon.svg" width="48" height="48" />

        <h1 className={styles.plotTitle}>
          {MODE[mode]} <span className={styles.mode}>control</span>
        </h1>

        <div className={styles.plotToolbar}>
          <div className={styles.static}>
            server: {' '}{server}
          </div>

          {isConnected
            ? <img width="32" height="32" src="/bridge-on.svg" />
            : <img width="32" height="32" src="/bridge-off.svg" />
          }

          <Legend
            legend={LEGEND}
            enabled={enabled}
            onEnable={handleToggleLegendSeries}
          />

          <div className={styles.toolbarGroup}>
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
        </div>
      </section>

      <section className={styles.plotStrip}>
        {/* Quadrature to Absolute Mapping Indicator */ }
        <div className={styles.absoluteToQuadratureRatio}>
          <span className={styles.da}>δa</span>
          {' / '}
          <span className={styles.dq}>δq</span>
          {' '}
          {Math.round(quadratureToAbsoluteRef.current * 1e6) / 1e6}
        </div>

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
            {data.length > 0 && <g
              ref={axisBottomRef}
              className={styles.axisBottom}
              transform={`translate(0,${HEIGHT - MARGIN_BOTTOM + SPACING})`}
            />}

            {/* Bottom axis domain */}
            {data.length > 0 && <line
              className={styles.axisBottom__domain}
              x1={SPACING_HALF}
              y1={HEIGHT - MARGIN_BOTTOM + SPACING}
              x2={width ?? 0 - MARGIN_RIGHT}
              y2={HEIGHT - MARGIN_BOTTOM + SPACING}
              strokeWidth="1"
            />}

            {LEGEND.map(({ key, color, min, max }) => (
              enabled[key]
              ? <Series
                  key={key}
                  property={key}
                  samples={data}
                  color={color}
                  min={min}
                  max={max}
                  {...layoutProps}
                />
              : null
            ))}
          </svg>
        </div>
      </section>
    </>
  );
};
