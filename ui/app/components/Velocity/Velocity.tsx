"use client";

import { useCallback, useEffect, useState } from "react";
import { PrimaryInput } from "../PrimaryInput";
import { Separator } from "../Separator";
import { Module } from "../Module";
import { Group } from "../Group";
import { VelocitySlider } from "./VelocitySlider";
import { VelocityKnob } from "./VelocityKnob";
import { Invert } from "./Invert";
import { Meter } from "./Meter";

export const Velocity = () => {
  const [velocity, setVelocity] = useState(0);
  const [invert, setInvert] = useState(false);

  useEffect(() => {
    // publish velocity command
    console.log('publish', velocity);
  }, [velocity]);

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
        min={-100}
        max={100}
        step={1}
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

      <Separator spacingRight="16px" />

      <Group>
        <Meter
          value={velocity}
          label="V"
          color="#376BE8"
        />
        <Meter
          value={velocity}
          label="A"
          color="#EC008C"
        />
      </Group>
    </Module>
  );
}