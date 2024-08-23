"use client";

import { useCallback, useState } from "react";
import { PrimaryInput } from "../PrimaryInput";
import { Separator } from "../Separator";
import { Module } from "../Module";
import { VelocitySlider } from "./VelocitySlider";
import { VelocityKnob } from "./VelocityKnob";
import { Invert } from "./Invert";

export const Velocity = () => {
  const [velocity, setVelocity] = useState(0);
  const [invert, setInvert] = useState(false);

  const handleChangeVelocity = useCallback((e) => {
    setVelocity(e.target.value / 100);
  }, []);

  const handleToggleInvert = useCallback(() => {
    setInvert(value => !value);
  }, []);

  return (
    <Module
      title="Velocity"
      image={<img src="/velocity.svg" width="32" height="32" />}
    >
      <PrimaryInput
        type="number"
        value={Math.round(velocity * 100)}
        onChange={handleChangeVelocity}
        min="-100"
        max="100"
        step="1"
      />
      <Separator />
      <VelocityKnob
        velocity={velocity}
        handleChange={setVelocity}
        invert={invert}
      />
      <Separator />
      <VelocitySlider
        velocity={velocity}
        handleChange={setVelocity}
        invert={invert}
      />
      <Separator invert/>
      <Invert onClick={handleToggleInvert} />
    </Module>
  );
}