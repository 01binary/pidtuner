"use client";

import * as d3 from "d3";
import {
  FC,
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState
} from "react";
import { PlotType } from "./PlotType";
import styles from "./Plot.module.css";
import initialData from "./sample.json";
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

export const Plot: FC = () => {
  const scrollRef = useRef<HTMLDivElement>(null);
  const axisLeftRef = useRef<SVGGElement>(null);
  const axisBottomRef = useRef<SVGGElement>(null);
  const [data, setData] = useState<PlotType[]>(initialData);
  const captureRef = useRef<boolean>(true);
  const allDataRef = useRef<PlotType[]>(initialData);
  const currentTimeRef = useRef<number>(initialData[initialData.length - 1].time + 0.02);

  const width = useMemo(() => (
    (data.length * SAMPLE_WIDTH) - MARGIN_LEFT - MARGIN_RIGHT
  ), [data]);

  useEffect(() => {
    const timer = window.setInterval(() => {
      if (!captureRef.current) return;

      const newSamples: PlotType[] = [
        {
          time: currentTimeRef.current + 0.02 * 0,
          command: Math.random(),
          absolute: Math.random()
        },
        {
          time: currentTimeRef.current + 0.02 * 1,
          command: Math.random(),
          absolute: Math.random()
        },
        {
          time: currentTimeRef.current + 0.02 * 2,
          command: Math.random(),
          absolute: Math.random()
        },
        {
          time: currentTimeRef.current + 0.02 * 3,
          command: Math.random(),
          absolute: Math.random()
        },
      ];

      currentTimeRef.current = currentTimeRef.current + 0.02 * 4;

      allDataRef.current.push(...newSamples);
      setData(d => d/*.slice(newSamples.length)*/.concat(newSamples))
    }, 100);

    () => window.clearInterval(timer);
  }, []);

  const x = useMemo(() => d3.scaleLinear(
    [0, data[data.length - 1].time],
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
    captureRef.current = !captureRef.current;
  }, []);

  const absolute = useMemo(() => data.map(({ absolute }) => absolute), [data]);
  const command = useMemo(() => data.map(({ command }) => command), [data]);

  const legend = useMemo(() => ([
    { samples: absolute, color: '#ec008c', label: 'absolute' },
    { samples: command, color: '#376be8', label: 'command' }
    // #795da3
  ]), [absolute, command]);

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
            title="Enable capture"
            onClick={handleToggleCapture}
          >
            <img src="/record.svg" width="24" height="24" />
          </button>

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
