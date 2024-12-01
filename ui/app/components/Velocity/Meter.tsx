import { FC, useRef, useMemo } from "react";
import { inter } from "../../inter";
import styles from "./Meter.module.css";

const MIN_OUTPUT = -40;
const MAX_OUTPUT = 40;
const DIVISIONS = [0, 5, 10, 15, 20];

type MeterProps = {
  value: number;
  max: number;
  label: string;
  color: string;
  divisions: number[];
};

const valueToDeg = (value: number, max: number) => {
  const norm = Math.abs(value / max);
  return norm * (MAX_OUTPUT - MIN_OUTPUT) + MIN_OUTPUT;
};

const translation = (x: number, y: number) => (
  `matrix(1 0 0 1 ${x} ${y})`
);

const useMovingAverageFilter = (value: number, size: number) => {
  const bufferRef = useRef([...new Array(size)]);
  const nextRef = useRef(0);

  const output = useMemo(() => {
    bufferRef.current[nextRef.current] = value;
    nextRef.current++;

    if (nextRef.current === bufferRef.current.length)
      nextRef.current = 0;

    const sum = bufferRef.current.reduce((acc, v) => acc + (v ?? 0), 0);
    const length = bufferRef.current.length;

    return sum / length;
  }, [value]);

  return output;
};

export const Meter: FC<MeterProps> = ({
  value: rawValue,
  max,
  label,
  color,
  divisions = DIVISIONS
}) => {
  const value = useMovingAverageFilter(rawValue, 128);
  const [div1, div2, div3, div4, div5] = divisions;

  return (
    <svg
      width="144px"
      height="143.2px"
      viewBox="0 0 144 143.2"
      className={styles.meter}
    >
      <text
        transform="matrix(1 0 0 1 15.3742 73.905)"
        fill={color}
        fontFamily={inter.style.fontFamily}
        fontSize="24px"
      >
        {label}
      </text>

      <text
        transform="matrix(1 0 0 1 95 73.905)"
        fontFamily={inter.style.fontFamily}
        fontSize="24px"
      >
        {Math.round(Math.abs(value))}
      </text>

      <g
        id="indicator"
        className={styles.indicator}
        style={{
          transformOrigin: `72px 99.8px`,
          transform: `rotate(${valueToDeg(value, max)}deg)`,
          transition: 'transform 1s ease-out'
        }}
      >
        <circle id="center" fill="none" cx="72" cy="99.8" r="5.9"/>
        <line fill="none" stroke="#27303B" strokeMiterlimit="10" x1="72" y1="40.1" x2="72" y2="99.8"/>
        <path fill="#27303B" d="M72.3,41.5c1.9-1.1,2.9-4.4,1.6-5.8c-0.6-0.7-1.1-2.2-1.4-3.6c-0.1-0.6-0.9-0.6-1,0
          c-0.3,1.4-0.8,2.9-1.4,3.6c-1.2,1.4-0.2,4.7,1.6,5.8H72.3z"/>
      </g>

      <g id="border">
        <line id="bolt" fill="#FFFFFF" stroke="#898989" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="72" y1="114.7" x2="74.8" y2="125.4"/>
        <path id="box" fill="#D3D3D3" d="M138.9,0H5.1C2.3,0,0,2.3,0,5.1v133c0,2.8,2.3,5.1,5.1,5.1h133.9c2.8,0,5.1-2.3,5.1-5.1V5.1
          C144,2.3,141.7,0,138.9,0z M73.4,129.6c-5.3,0-9.6-4.3-9.6-9.6s4.3-9.6,9.6-9.6s9.6,4.3,9.6,9.6S78.7,129.6,73.4,129.6z
          M135.8,76.1c0,3.2-2.6,5.8-5.8,5.8H13.2c-3.2,0-5.8-2.6-5.8-5.8V13.5c0-3.2,2.6-5.8,5.8-5.8H130c3.2,0,5.8,2.6,5.8,5.8V76.1z"/>
      </g>

      <g id="ticks">
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="72" y1="32.6" x2="72" y2="23.6"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="67.3" y1="32.7" x2="67" y2="28.2"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="62.6" y1="33.3" x2="62" y2="28.7"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="58" y1="34" x2="57.1" y2="29.6"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="53.5" y1="35.2" x2="52.2" y2="30.8"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="49" y1="36.6" x2="45.9" y2="28.2"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="44.7" y1="38.4" x2="42.8" y2="34.2"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="40.4" y1="40.4" x2="38.3" y2="36.4"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="36.4" y1="42.8" x2="34" y2="39"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="32.5" y1="45.4" x2="29.8" y2="41.8"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="28.8" y1="48.3" x2="23" y2="41.4"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="76.7" y1="32.7" x2="77" y2="28.2"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="81.4" y1="33.3" x2="82" y2="28.7"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="86" y1="34" x2="86.9" y2="29.6"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="90.5" y1="35.2" x2="91.8" y2="30.8"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="95" y1="36.6" x2="98.1" y2="28.2"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="99.3" y1="38.4" x2="101.2" y2="34.2"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="103.6" y1="40.4" x2="105.7" y2="36.4"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="107.6" y1="42.8" x2="110" y2="39"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="111.5" y1="45.4" x2="114.2" y2="41.8"/>
        <line fill="#FFFFFF" stroke="#898989" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="115.2" y1="48.3" x2="121" y2="41.4"/>
      </g>

      <g id="labels">
        <text
          transform={translation(15, 39)}
          fontFamily={inter.style.fontFamily}
          fontSize="10px"
        >
          {div1}
        </text>
        <text
          transform={translation(40.3, 24.5)}
          fontFamily={inter.style.fontFamily}
          fontSize="10px"
        >
          {div2}
        </text>
        <text
          transform={translation(
            div3.toString().length < 2 ? 68.5 : 65.5,
            18.5
          )}
          fontFamily={inter.style.fontFamily}
          fontSize="10px"
        >
          {div3}
        </text>
        <text
          transform={translation(
            div4.toString().length < 2 ? 98 : 96,
            23
          )}
          fontFamily={inter.style.fontFamily}
          fontSize="10px"
        >
          {div4}
        </text>
        <text
          transform={translation(
            div5.toString().length < 2 ? 122 : 120.7,
            37
          )}
          fontFamily={inter.style.fontFamily}
          fontSize="10px"
        >
          {div5}
        </text>
      </g>
    </svg>
  );
};
