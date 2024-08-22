"use client";

import { useCallback, useState } from "react";
import { PrimaryInput } from "../PrimaryInput";
import { Separator } from "../Separator";
import { Module } from "../Module";
import { VelocitySlider } from "./VelocitySlider";
import { VelocityKnob } from "./VelocityKnob";

export const Velocity = () => {
  const [velocity, setVelocity] = useState(0);

  const handleChangeVelocity = useCallback((e) => {
    setVelocity(e.target.value / 100);
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
      />
      <Separator />
      <VelocitySlider
        velocity={velocity}
        handleChange={setVelocity}
      />
    </Module>
  );
}