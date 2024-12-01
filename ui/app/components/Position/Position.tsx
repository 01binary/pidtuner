"use client";

import { FC, useCallback, useEffect, useRef, useState } from "react";
import { Module } from "../Module";
import {
  ConfigurationCommand,
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
import styles from "./Position.module.css";

const DEFAULT_TOLERANCE = 0.1;

type PositionProps = {
  publishPosition: (command: PositionCommand) => void;
  publishConfiguration: (config: ConfigurationCommand) => void;
  configuration: ConfigurationCommand;
  position: number;
  goal: number;
  pe: number;
  ie: number;
  de: number;
};

export const Position: FC<PositionProps> = ({
  publishPosition,
  publishConfiguration,
  configuration,
  position,
  goal: initialGoal,
  pe,
  ie,
  de
}) => {
  const [goal, setGoal] = useState(initialGoal);
  const [tolerance, setTolerance] = useState(DEFAULT_TOLERANCE);
  const [isRadial, setIsRadial] = useState(true);

  const isChangedRef = useRef(false);

  const { Kp, Ki, Kd, iMin, iMax } = configuration;

  useEffect(() => {
    if (isChangedRef.current) return;
    setGoal(initialGoal);
  }, [initialGoal])

  useEffect(() => {
    if (!isChangedRef.current) return;
    publishPosition({ goal, tolerance });
  }, [goal, tolerance, publishPosition]);

  const handleChangeGoalInput = useCallback((e) => {
    isChangedRef.current = true;
    setGoal(e.target.value / 100);
  }, []);

  const handleChangeGoalKnob = useCallback((value: number) => {
    isChangedRef.current = true;
    if (isRadial) {
      setGoal(value + 0.5);
    } else {
      setGoal(value)
    }
  }, [isRadial]);

  const handleChangeTolerance = useCallback((e) => {
    isChangedRef.current = true;
    setTolerance(e.target.value / 100);
  }, []);

  const handleToggleLinearRadial = useCallback(() => {
    setIsRadial(lastSetting => !lastSetting);
  }, []);

  const handleChangeKp = useCallback((e) => {
    isChangedRef.current = true;
    publishConfiguration({
      ...configuration,
      Kp: e.target.value
    });
  }, [configuration, publishConfiguration]);

  const handleChangeKi = useCallback((e) => {
    isChangedRef.current = true;
    publishConfiguration({
      ...configuration,
      Ki: e.target.value
    });
  }, [configuration, publishConfiguration]);

  const handleChangeKd = useCallback((e) => {
    isChangedRef.current = true;
    publishConfiguration({
      ...configuration,
      Kd: e.target.value
    });
  }, [configuration, publishConfiguration]);

  const handleChangeIMin = useCallback((e) => {
    isChangedRef.current = true;
    publishConfiguration({
      ...configuration,
      iMin: e.target.value
    });
  }, [configuration, publishConfiguration]);

  const handleChangeIMax = useCallback((e) => {
    isChangedRef.current = true;
    publishConfiguration({
      ...configuration,
      iMax: e.target.value
    });
  }, [configuration, publishConfiguration]);

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

      <section className={styles.setPoint}>
        <div className={isRadial ? styles.errorRadial : styles.errorLinear}>
          <span className={styles.errorLabel}>
            Error
          </span>
          {' '}
          <span className={styles.errorValue}>
            {Math.round((goal - position) * 100)}
          </span>
        </div>

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
      </section>

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
            onChange={handleChangeKp}
            style={{ width: '9.4rem' }}
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
            onChange={handleChangeKi}
            style={{ width: '9.4rem' }}
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
            onChange={handleChangeKd}
            style={{ width: '9.4rem' }}
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
            label={<>I<sub>min</sub></>}
            largeLabel
            labelWidth="6rem"
            onChange={handleChangeIMin}
            style={{ width: '1.5em' }}
          />
        </Group>

        <Group alignTop>
          <PrimaryInput
            id="imax"
            type="number"
            value={iMax}
            label={<>I<sub>max</sub></>}
            largeLabel
            labelWidth="6rem"
            onChange={handleChangeIMax}
            style={{ width: '1.5em' }}
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
