"use client";

import { useCallback } from "react";
import { RAD_TO_DEG, useKnob } from "../knobUtils";
import styles from "./VelocityKnob.module.css";

const INCREMENTS = [
  0.08, 0.13, 0.18, 0.25, 0.27, 0.33, 0.4, 0.46, 0.50, 0.56, 0.62, 0.68, 0.75, 0.78, 0.85, 0.9
];

type VelocityKnobProps = {
  velocity: number;
  handleChange: (velocity: number) => void;
  invert?: boolean;
};

export const VelocityKnob = ({
  velocity,
  handleChange,
  invert
}: VelocityKnobProps) => {
  const {
    svgRef,
    knobRef,
    handleMouseDown,
    handleMouseUp,
    handleMouseMove,
    knobCenterX,
    knobCenterY,
    originX,
    originY,
    angle
  } = useKnob({
    value: velocity,
    invert,
    range: 'half',
    wrap: true,
    handleChange
  })

  const invertMultiplier = invert ? -1 : 1;

  const handleMarkClick = useCallback((
    index: number,
    forward: boolean = true
  ) => {
    const shouldInvert = forward ? invert : !invert;
    const increment = shouldInvert ? -INCREMENTS[index] : INCREMENTS[index];
    handleChange(increment);
  }, [invert, handleChange]);

  const handleJump = useCallback((increment: number) => {
    const nextVelocity = velocity + increment;
    handleChange(Math.max(Math.min(nextVelocity, 1), -1));
  }, [velocity, handleChange])

  return (
    <svg
      ref={svgRef}
      className={styles.velocityKnob}
      width="330px"
      height="185px"
      viewBox="0 0 330 185"
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
    >
      <g
        id="knobInteractive"
        ref={knobRef}
        onMouseDown={handleMouseDown}
        onMouseUp={handleMouseUp}
        style={{
          cursor: 'pointer',
          transformOrigin: `${knobCenterX - originX}px ${knobCenterY - originY}px`,
          transform: `rotate(${(angle) * RAD_TO_DEG}deg)`
        }}
      >
        <path id="knob" d="M161.2,46.3c-0.7,0-1.4,0-2,0c-3.5,0-6.4,1.3-8.7,4c-3.9,4.5-9,7.1-14.8,8.2c-2.9,0.5-5.3,1.9-6.8,4.4
          c-1.2,1.9-2.4,3.9-3.6,5.8c-1.6,2.8-1.8,5.7-0.7,8.8c1.1,3,1.6,6.1,1.5,9.1c0,3-0.4,6.1-1.5,9.1c-1.1,3.1-0.9,6,0.7,8.8
          c1.1,2,2.3,3.9,3.6,5.8c1.6,2.5,4,3.9,6.8,4.4c5.9,1,10.9,3.7,14.8,8.2c2.3,2.7,5.2,4,8.7,4c1.9,0,3.7,0.1,5.6-0.1
          c3.1-0.2,5.7-1.3,7.8-3.7c3.8-4.5,8.6-7.2,14.3-8.6c8.1-1.9,13.6-11.6,11.1-19.5c-0.9-2.8-1.3-5.6-1.2-8.4c0-2.8,0.4-5.6,1.2-8.4
          c2.4-7.9-3.1-17.6-11.1-19.5c-5.7-1.3-10.5-4.1-14.3-8.6c-2.1-2.4-4.7-3.5-7.8-3.7C163.6,46.3,162.4,46.3,161.2,46.3"/>
        <circle id="dot" fill="#FFFFFF" cx="161.2" cy="51.4" r="2"/>
        <circle id="knobCenter" fill="#FFFFFF" cx="161.2" cy="86.6" r="29.5"/>
      </g>

      <g id="marksInteractive" fill="#D3D3D3">
        <path className={styles.mark} onClick={() => handleMarkClick(15, true)} id="r15" fill="#D3D3D3" d="M184.2,148l-4.9-13.5c-2.6,0.9-5.2,1.6-7.9,2.1l2.5,14.3C177.4,150.3,180.9,149.3,184.2,148z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(14, true)} id="r14" fill="#D3D3D3" d="M187.9,130.6c-2.4,1.3-4.8,2.5-7.4,3.5l4.9,13.5c3.3-1.3,6.5-3,9.5-4.8L187.9,130.6z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(13, true)} id="r13" fill="#D3D3D3" d="M195.7,125.2c-2.1,1.7-4.4,3.3-6.7,4.7l7,12.1c3-1.9,5.8-4.1,8.4-6.5L195.7,125.2z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(12, true)} id="r12" fill="#D3D3D3" d="M202.5,118.7c-1.8,2.1-3.7,4-5.8,5.7l8.6,10.2c2.5-2.4,4.9-5,7-7.8L202.5,118.7z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(11, true)} id="r11" fill="#D3D3D3" d="M208.1,111c-1.4,2.3-3,4.6-4.7,6.6l9.7,8.1c2-2.8,3.8-5.7,5.3-8.8L208.1,111z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(10, true)} id="r10" fill="#D3D3D3" d="M208.8,109.9l10.2,5.8c1.4-3.1,2.6-6.2,3.5-9.5l-10.2-3.7C211.3,105.1,210.1,107.5,208.8,109.9z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(9, true)} id="r9" fill="#D3D3D3" d="M214.8,93.4c-0.5,2.7-1.2,5.3-2.1,7.9l10.1,3.7c0.8-3.2,1.4-6.5,1.7-9.8L214.8,93.4z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(8, true)} id="r8" fill="#D3D3D3" d="M224.5,84C224.4,84,224.4,84,224.5,84l-8.7,0c0,2.8-0.3,5.5-0.7,8.1l9.6,1.7C224.8,90.6,224.8,87.3,224.5,84z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(7, true)} id="r7" fill="#D3D3D3" d="M215.7,82.7h8.6c-0.3-3.2-0.9-6.4-1.8-9.4l-7.5,1.3C215.5,77.2,215.7,79.9,215.7,82.7z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(6, true)} id="r6" fill="#D3D3D3" d="M214.8,73.2l7.4-1.3c-0.9-3-2.1-6-3.4-8.8l-6.1,2.2C213.6,67.9,214.3,70.5,214.8,73.2z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(5, true)} id="r5" fill="#D3D3D3" d="M212.3,64.1l5.9-2.1c-1.4-2.8-3-5.4-4.8-7.9l-4.6,2.6C210.1,59.1,211.3,61.6,212.3,64.1z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(4, true)} id="r4" fill="#D3D3D3" d="M208.1,55.6l4.5-2.6c-1.8-2.4-3.8-4.7-5.9-6.8l-3.3,2.8C205.1,51.1,206.7,53.3,208.1,55.6z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(3, true)} id="r3" fill="#D3D3D3" d="M202.5,48l3.2-2.7c-2.1-2.1-4.4-4-6.8-5.7l-2.3,2.7C198.8,44,200.7,45.9,202.5,48z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(2, true)} id="r2" fill="#D3D3D3" d="M195.7,41.4l2.2-2.6c-2.4-1.7-4.9-3.2-7.5-4.5l-1.4,2.4C191.4,38.1,193.6,39.7,195.7,41.4z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(1, true)} id="r1" fill="#D3D3D3" d="M187.9,36.1l1.4-2.4c-2.6-1.3-5.3-2.3-8-3.2l-0.8,2.2C183,33.6,185.5,34.8,187.9,36.1z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(0, true)} id="r0" fill="#D3D3D3" d="M171.3,30c2.7,0.5,5.3,1.2,7.9,2.1l0.8-2.1c-2.7-0.8-5.5-1.4-8.3-1.9L171.3,30z"/>

        <path className={styles.mark} onClick={() => handleMarkClick(15, false)} id="l15" fill="#D3D3D3" d="M138.3,148l4.9-13.5c2.6,0.9,5.2,1.6,7.9,2.1l-2.5,14.3C145,150.3,141.6,149.3,138.3,148z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(14, false)} id="l14" fill="#D3D3D3" d="M134.6,130.6c2.4,1.3,4.8,2.5,7.4,3.5l-4.9,13.5c-3.3-1.3-6.5-3-9.5-4.8L134.6,130.6z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(13, false)} id="l13" fill="#D3D3D3" d="M126.7,125.2c2.1,1.7,4.4,3.3,6.7,4.7l-7,12.1c-3-1.9-5.8-4.1-8.4-6.5L126.7,125.2z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(12, false)} id="l12" fill="#D3D3D3" d="M119.9,118.7c1.8,2.1,3.7,4,5.8,5.7l-8.6,10.2c-2.5-2.4-4.9-5-7-7.8L119.9,118.7z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(11, false)} id="l11" fill="#D3D3D3" d="M114.3,111c1.4,2.3,3,4.6,4.7,6.6l-9.7,8.1c-2-2.8-3.8-5.7-5.3-8.8L114.3,111z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(10, false)} id="l10" fill="#D3D3D3" d="M113.6,109.9l-10.2,5.8c-1.4-3.1-2.6-6.2-3.5-9.5l10.2-3.7C111.1,105.1,112.3,107.5,113.6,109.9z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(9, false)} id="l9" fill="#D3D3D3" d="M107.6,93.4c0.5,2.7,1.2,5.3,2.1,7.9l-10.1,3.7c-0.8-3.2-1.4-6.5-1.7-9.8L107.6,93.4z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(8, false)} id="l8" fill="#D3D3D3" d="M98,84C98,84,98,84,98,84l8.7,0c0,2.8,0.3,5.5,0.7,8.1l-9.6,1.7C97.6,90.6,97.6,87.3,98,84z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(7, false)} id="l7" fill="#D3D3D3" d="M106.7,82.7h-8.6c0.3-3.2,0.9-6.4,1.8-9.4l7.5,1.3C106.9,77.2,106.7,79.9,106.7,82.7z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(6, false)} id="l6" fill="#D3D3D3" d="M107.6,73.2l-7.4-1.3c0.9-3,2.1-6,3.4-8.8l6.1,2.2C108.8,67.9,108.1,70.5,107.6,73.2z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(5, false)} id="l5" fill="#D3D3D3" d="M110.2,64.1l-5.9-2.1c1.4-2.8,3-5.4,4.8-7.9l4.6,2.6C112.3,59.1,111.1,61.6,110.2,64.1z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(4, false)} id="l4" fill="#D3D3D3" d="M114.3,55.6l-4.5-2.6c1.8-2.4,3.8-4.7,5.9-6.8L119,49C117.3,51.1,115.7,53.3,114.3,55.6z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(3, false)} id="l3" fill="#D3D3D3" d="M119.9,48l-3.2-2.7c2.1-2.1,4.4-4,6.8-5.7l2.3,2.7C123.6,44,121.7,45.9,119.9,48z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(2, false)} id="l2" fill="#D3D3D3" d="M126.7,41.4l-2.2-2.6c2.4-1.7,4.9-3.2,7.5-4.5l1.4,2.4C131.1,38.1,128.8,39.7,126.7,41.4z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(1, false)} id="l1" fill="#D3D3D3" d="M134.6,36.1l-1.4-2.4c2.6-1.3,5.3-2.3,8-3.2l0.8,2.2C139.4,33.6,136.9,34.8,134.6,36.1z"/>
        <path className={styles.mark} onClick={() => handleMarkClick(0, false)} id="l0" fill="#D3D3D3" d="M151.1,30c-2.7,0.5-5.3,1.2-7.9,2.1l-0.8-2.1c2.7-0.8,5.5-1.4,8.3-1.9L151.1,30z"/>
      </g>

      <g id="labelsStatic" style={{ pointerEvents: 'none' }}>
        <text
          transform="matrix(1 0 0 1 7.0296 25.5429)"
          fill="#5B5B5B"
          font-family="'Roboto-Medium'"
          font-size="18px"
        >
          {invert ? 'LPWM' : 'RPWM'}
        </text>
        <text
          transform="matrix(1 0 0 1 266.74 25.5429)"
          fill="#5B5B5B"
          font-family="'Roboto-Medium'"
          font-size="18px"
        >
          {invert ? 'RPWM' : 'LPWM'}
        </text>
      </g>
      
      <g id="labelsInteractive">
        <text
          className={styles.interactiveLabel}
          transform="matrix(1 0 0 1 71.4851 159.2597)"
          font-family="'Roboto-Medium'"
          font-size="18px"
          onClick={() => handleChange(-0.75 * invertMultiplier)}
        >
          {-75 * invertMultiplier}
        </text>
        <text
          className={styles.interactiveLabel}
          transform="matrix(1 0 0 1 40.99 93.5742)"
          font-family="'Roboto-Medium'"
          font-size="18px"
          onClick={() => handleChange(-0.5 * invertMultiplier)}
        >
          {-50 * invertMultiplier}
        </text>
        <text
          className={styles.interactiveLabel}
          transform="matrix(1 0 0 1 71.4851 25.5429)"
          font-family="'Roboto-Medium'"
          font-size="18px"
          onClick={() => handleChange(-0.25 * invertMultiplier)}
        >
          {-25 * invertMultiplier}
        </text>
        <text
          className={styles.interactiveLabel}
          transform="matrix(1 0 0 1 227.5262 159.2597)"
          font-family="'Roboto-Medium'"
          font-size="18px"
          onClick={() => handleChange(0.75 * invertMultiplier)}
        >
          {75 * invertMultiplier}
        </text>
        <text
          className={styles.interactiveLabel}
          transform="matrix(1 0 0 1 258.0232 93.5742)"
          font-family="'Roboto-Medium'"
          font-size="18px"
          onClick={() => handleChange(0.5 * invertMultiplier)}
        >
          {50 * invertMultiplier}
        </text>
        <text
          className={styles.interactiveLabel}
          transform="matrix(1 0 0 1 227.5262 25.5429)"
          font-family="'Roboto-Medium'"
          font-size="18px"
          onClick={() => handleChange(0.25 * invertMultiplier)}
        >
          {25 * invertMultiplier}
        </text>
        <text
          className={styles.interactiveLabel}
          transform="matrix(1 0 0 1 156.0974 13.9208)"
          font-family="'Roboto-Medium'"
          font-size="18px"
          onClick={() => handleChange(0)}
        >
          0
        </text>
        <text
          className={styles.interactiveLabel}
          transform="matrix(1 0 0 1 145.867 179.0742)"
          font-family="'Roboto-Medium'"
          font-size="18px"
          onClick={() => handleChange(1)}
        >
          100
        </text>
      </g>

      <g
        id="jumpButtonRightPlus"
        style={invert ? { visibility: 'hidden' } : { visibility: 'visible' }}
        className={styles.jumpButton}
        onClick={() => handleJump(0.1)}
      >
        <polygon
          className={styles.jumpBorder}
          fill="white"
          stroke="#A5A5A5"
          strokeMiterlimit="10"
          points="202.4,150.9 186.7,150.9 178.8,158.4 186.7,165.9 202.4,165.9 210.3,158.4"
        />
        <polygon
          id="plusRight"
          className={styles.jumpLabel}
          fillRule="evenodd"
          clipRule="evenodd"
          fill="#EC008C"
          points="200.1,157.7 195.3,157.7 195.3,152.9 193.8,152.9 193.8,157.7 189,157.7 189,159.2 193.8,159.2 193.8,164 195.3,164 195.3,159.2 200.1,159.2"
        />
      </g>

      <g
        id="jumpButtonLeftPlus"
        style={invert ? { visibility: 'visible' } : { visibility: 'hidden' }}
        className={styles.jumpButton}
        onClick={() => handleJump(0.1)}
      >
        <polygon
          className={styles.jumpBorder}
          fill="white"
          stroke="#A5A5A5"
          strokeMiterlimit="10"
          points="202.4,150.9 186.7,150.9 178.8,158.4 186.7,165.9 202.4,165.9 210.3,158.4"
        />
        <polygon
          id="plusLeft"
          className={styles.jumpLabel}
          fillRule="evenodd"
          clipRule="evenodd"
          fill="#EC008C"
          points="132.2,157.7 127.4,157.7 127.4,152.9 125.9,152.9 125.9,157.7 121.1,157.7 121.1,159.2 125.9,159.2 125.9,164 127.4,164 127.4,159.2 132.2,159.2 	"
        />
      </g>

      <g
        id="jumpButtonRightMinus"
        style={invert ? { visibility: 'visible' } : { visibility: 'hidden' }}
        className={styles.jumpButton}
        onClick={() => handleJump(-0.1)}
      >
        <polygon
          className={styles.jumpBorder}
          fill="white"
          stroke="#A5A5A5"
          strokeMiterlimit="10"
          points="134.5,150.9 118.8,150.9 110.9,158.4 118.8,165.9 134.5,165.9 142.4,158.4"
        />
        <rect
          id="minusRight"
          className={styles.jumpLabel}
          x="189"
          y="157.7"
          fill-rule="evenodd"
          clip-rule="evenodd"
          fill="#376BE8"
          width="11.1"
          height="1.5"
        />
      </g>

      <g
        id="jumpButtonLeftMinus"
        style={invert ? { visibility: 'hidden' } : { visibility: 'visible' }}
        className={styles.jumpButton}
        onClick={() => handleJump(-0.1)}
      >
        <polygon
          className={styles.jumpBorder}
          fill="white"
          stroke="#A5A5A5"
          strokeMiterlimit="10"
          points="134.5,150.9 118.8,150.9 110.9,158.4 118.8,165.9 134.5,165.9 142.4,158.4"
        />
        <rect
          id="minusLeft"
          className={styles.jumpLabel}
          x="121.1"
          y="157.7"
          fill-rule="evenodd"
          clip-rule="evenodd"
          fill="#376BE8"
          width="11.1"
          height="1.5"
        />
      </g>

      <g id="ticks" style={{ pointerEvents: 'none' }}>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="236.7" y1="88.1" x2="244.3" y2="88.1"/>
        <path fill="none" stroke="#A5A5A5" stroke-miterlimit="10" d="M222,33l-5.9,5.8c12.7,12.6,20.5,30,20.5,49.3
          c0,19.2-7.8,36.5-20.3,49.1l5.7,5.7"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="231.4" y1="61.5" x2="236.7" y2="59.3"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="231.8" y1="113.8" x2="237.1" y2="116"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="85.7" y1="88.1" x2="78.1" y2="88.1"/>
        <path fill="none" stroke="#A5A5A5" stroke-miterlimit="10" d="M100.4,33l5.9,5.8c-12.7,12.6-20.5,30-20.5,49.3
          c0,19.2,7.8,36.5,20.3,49.1l-5.7,5.7"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="91" y1="61.5" x2="85.7" y2="59.3"/>
        <line fill="none" stroke="#A5A5A5" stroke-miterlimit="10" x1="90.6" y1="113.8" x2="85.4" y2="116"/>
      </g>
    </svg>
  );
};
