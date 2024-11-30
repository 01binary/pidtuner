"use client";

import { FC, useCallback, useEffect, useRef, useState } from "react";
import { Module } from "../Module";
import {
  ConfigurationCommand,
  DEFAULT_CONFIGURATION,
  PositionCommand
} from "@/app/useMotorControl";
import { clamp } from "@/app/utils";
import { PrimaryInput } from "../PrimaryInput";
import { Group } from "../Group";
import { PositionKnob } from "./PositionKnob";
import { Separator } from "../Separator";
import { Gauge } from "../Gauge/Gauge";
import { RadialIcon } from "./RadialIcon";
import { LinearIcon } from "./LinearIcon";
import { PositionSlider } from "./PositionSlider";

type PositionProps = {
  publishPosition: (command: PositionCommand) => void;
  publishConfiguration: (config: ConfigurationCommand) => void;
  position: number;
  goal: number;
  tolerance: number;
  pe: number;
  ie: number;
  de: number;
};

export const Position: FC<PositionProps> = ({
  publishPosition,
  publishConfiguration,
  position,
  goal: initialGoal,
  tolerance: initialTolerance,
  pe,
  ie,
  de
}) => {
  const [goal, setGoal] = useState(initialGoal);
  const [tolerance, setTolerance] = useState(initialTolerance);
  const [Kp, setKp] = useState(DEFAULT_CONFIGURATION.Kp);
  const [Ki, setKi] = useState(DEFAULT_CONFIGURATION.Ki);
  const [Kd, setKd] = useState(DEFAULT_CONFIGURATION.Kd);
  const [iMin, setIMin] = useState(DEFAULT_CONFIGURATION.iMax);
  const [iMax, setIMax] = useState(DEFAULT_CONFIGURATION.iMax);
  const [isRadial, setIsRadial] = useState(true);
  const isInitializedRef = useRef(false);

  useEffect(() => {
    if (!isInitializedRef.current) return;
    publishPosition({ goal, tolerance });
  }, [goal, tolerance, publishPosition]);

  useEffect(() => {
    if (!isInitializedRef.current) return;
    publishConfiguration({
      ...DEFAULT_CONFIGURATION, Kp, Ki, Kd, iMin, iMax
    })
  }, [
    Kp,
    Ki,
    Kd,
    iMin,
    iMax,
    publishConfiguration
  ]);

  const handleChangeGoalInput = useCallback((e) => {
    isInitializedRef.current = true;
    console.log('goal input change')
    setGoal(e.target.value / 100);
  }, []);

  const handleChangeGoalKnob = useCallback((value: number) => {
    isInitializedRef.current = true;
    if (isRadial) {
      console.log('goal knob change', value + 0.5)
      setGoal(value + 0.5);
    } else {
      setGoal(value)
    }
  }, [isRadial]);

  const handleChangeTolerance = useCallback((e) => {
    isInitializedRef.current = true;
    setTolerance(e.target.value / 100);
  }, []);

  const handleToggleLinearRadial = useCallback(() => {
    setIsRadial(lastSetting => !lastSetting);
  }, []);

  return (
    <Module
      title="Position"
      image={
        <img
          src="/position.svg"
          width="32"
          height="32"
        />
      }
    >
      <Group vertical marginTop>
        <PrimaryInput
          type="number"
          value={Math.round(goal * 100)}
          onChange={handleChangeGoalInput}
          min={isRadial ? -50 : 0}
          max={isRadial ? 50 : 100}
          step={1}
        />

        <PrimaryInput
          id="tolerance"
          type="number"
          label="Tolerance"
          value={tolerance * 100}
          onChange={handleChangeTolerance}
          min={0}
          max={100}
        />
      </Group>

      {isRadial
        ? (
          <PositionKnob
            goal={clamp(goal - 0.5, -0.5, 0.5)}
            position={clamp(position - 0.5, -0.5, 0.5)}
            handleChange={handleChangeGoalKnob}
            min={-0.5}
            max={0.5}
          />
        )
        : (
          <PositionSlider
            goal={goal}
            position={position}
            handleChange={handleChangeGoalKnob}
            min={0}
            max={1}
          />
        )
      }

      <Group vertical marginLeft>
        <button
          title={isRadial ? 'Switch to linear control' : 'Switch to radial control'}
          onClick={handleToggleLinearRadial}
        >
          {isRadial ? <LinearIcon /> : <RadialIcon />}
        </button>
      </Group>

      <Separator
        spacingLeft="1rem"
        spacingRight="1rem"
      />

      <Group vertical tight>
        <Group alignTop tight>
          <PrimaryInput
            id="kp"
            type="number"
            value={Kp}
            label="Kp"
            largeLabel
            labelWidth="4rem"
            onChange={(e) => setKp(e.target.value)}
          />
        </Group>

        <Group alignTop tight>
          <PrimaryInput
            id="ki"
            type="number"
            value={Ki}
            label="Ki"
            largeLabel
            labelWidth="4rem"
            onChange={(e) => setKi(e.target.value)}
          />
        </Group>

        <Group alignTop tight>
          <PrimaryInput
            id="kd"
            type="number"
            value={Kd}
            label="Kd"
            largeLabel
            labelWidth="4rem"
            onChange={(e) => setKd(e.target.value)}
          />
        </Group>
      </Group>

      <Separator
        invert
        spacingRight="1rem"
      />

      <Group vertical tight>
        <Group alignTop>
          <PrimaryInput
            id="imin"
            type="number"
            value={iMin}
            label="Imin"
            largeLabel
            labelWidth="6rem"
            onChange={(e) => setIMin(e.target.value)}
          />
        </Group>

        <Group alignTop>
          <PrimaryInput
            id="imax"
            type="number"
            value={iMax}
            label="Imax"
            largeLabel
            labelWidth="6rem"
            onChange={(e) => setIMax(e.target.value)}
          />
        </Group>
      </Group>

      <Separator spacingRight="0.5rem" />

      <Group>
        <Gauge value={pe} label="P error" />
        <Gauge value={ie} label="I error" />
        <Gauge value={de} label="D error" />
      </Group>
    </Module>
  );
};
