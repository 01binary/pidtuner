"use client";

import { FC, useCallback, useEffect, useState } from "react";
import { Module } from "../Module";
import { PositionCommand } from "@/app/useMotorControl";
import { PrimaryInput } from "../PrimaryInput";
import { PositionKnob } from "./PositionKnob";

type PositionProps = {
  publishPosition: (command: PositionCommand) => void;
  Kp: number;
  Ki: number;
  Kd: number;
  iMin: number;
  iMax: number;
  position: number;
  goal: number;
  tolerance: number;
  pe: number;
  ie: number;
  de: number;
};

export const Position: FC<PositionProps> = ({
  publishPosition,
  position: initialPosition,
  goal: initialGoal,
  tolerance: initialTolerance,
  Kp,
  Ki,
  Kd,
  iMin,
  iMax,
  pe,
  ie,
  de
}) => {
  const [goal, setGoal] = useState(initialGoal);
  const [position, setPosition] = useState(0);
  const [tolerance, setTolerance] = useState(initialTolerance);

  useEffect(() => {
    publishPosition({ goal, tolerance });
  }, [goal, tolerance, publishPosition]);

  const handleChangeGoal = useCallback((e) => {
    setGoal(e.target.value / 100);
  }, []);

  const error = position - goal;

  return (
    <Module
      title="Position"
      image={<img src="/position.svg" width="32" height="32" />}
    >
      <PrimaryInput
        type="number"
        value={Math.round(goal * 100)}
        onChange={handleChangeGoal}
        min={-100}
        max={100}
        step={1}
      />

      <PrimaryInput
        type="number"
        value={Math.round(position * 100)}
        onChange={e => setPosition(e.target.value / 100)}
        min={-100}
        max={100}
        step={1}
      />

      <PositionKnob
        position={goal}
        actualPosition={position}
        error={error}
        handleChange={setGoal}
      />
    </Module>
  );
};
