"use client";

import { FC, useCallback, useEffect, useRef, useState } from "react";
import { Module } from "../Module";
import {
  ConfigurationCommand,
  DEFAULT_CONFIGURATION,
  PositionCommand
} from "@/app/useMotorControl";
import { PrimaryInput } from "../PrimaryInput";
import { Group } from "../Group";
import { PositionKnob } from "./PositionKnob";
import { Separator } from "../Separator";
import { Gauge } from "../Gauge/Gauge";

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
  const [Kp, setKp] = useState(DEFAULT_CONFIGURATION.Kp)
  const [Ki, setKi] = useState(DEFAULT_CONFIGURATION.Ki)
  const [Kd, setKd] = useState(DEFAULT_CONFIGURATION.Kd)
  const [iMin, setIMin] = useState(DEFAULT_CONFIGURATION.iMax)
  const [iMax, setIMax] = useState(DEFAULT_CONFIGURATION.iMax)

  useEffect(() => {
    publishPosition({ goal, tolerance });
  }, [goal, tolerance, publishPosition]);

  useEffect(() => {
    publishConfiguration({
      ...DEFAULT_CONFIGURATION,
      Kp,
      Ki,
      Kd,
      iMin,
      iMax
    })
  }, [Kp, Ki, Kd, iMin, iMax, publishConfiguration])

  const handleChangeGoal = useCallback((e) => {
    setGoal(e.target.value / 100);
  }, []);

  const error = position - goal;

  return (
    <Module
      title="Position"
      image={<img src="/position.svg" width="32" height="32" />}
    >
      <Group vertical alignTop marginTop>
        <PrimaryInput
          type="number"
          value={Math.round(goal * 100)}
          onChange={handleChangeGoal}
          min={-100}
          max={100}
          step={1}
        />

        <PrimaryInput
          id="tolerance"
          type="number"
          label="Tolerance"
          value={tolerance}
          onChange={e => setTolerance(e.target.value)}
        />
      </Group>

      <Separator
        invert
        spacingLeft="1rem"
      />

      <PositionKnob
        goal={goal}
        position={position}
        error={error}
        handleChange={setGoal}
      />

      <Separator
        spacingLeft="1.5rem"
        spacingRight="1rem"
      />

      <Group vertical tight>
        <Group alignTop tight>
          <PrimaryInput
            id="kp"
            type="number"
            value={Kp}
            label="P"
            largeLabel
            labelWidth="2rem"
            onChange={(e) => setKp(e.target.value)}
          />
        </Group>

        <Group alignTop tight>
          <PrimaryInput
            id="ki"
            type="number"
            value={Ki}
            label="I"
            largeLabel
            labelWidth="2rem"
            onChange={(e) => setKi(e.target.value)}
          />
        </Group>

        <Group alignTop tight>
          <PrimaryInput
            id="kd"
            type="number"
            value={Kd}
            label="D"
            largeLabel
            labelWidth="2rem"
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
            labelWidth="5.5rem"
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
            labelWidth="5.5rem"
            onChange={(e) => setIMax(e.target.value)}
          />
        </Group>
      </Group>

      <Separator spacing="1rem" />

      <Group>
        <Gauge value={pe} label="P error" />
        <Gauge value={ie} label="I error" />
        <Gauge value={de} label="D error" />
      </Group>
    </Module>
  );
};
