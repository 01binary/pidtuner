"use client";

import { useState } from "react";
import { Group } from "../Group";
import { Module } from "../Module";
import { PrimaryInput } from "../PrimaryInput";

export const Steps = () => {
  const [step, setStep] = useState(0);

  return (
    <Module
      title="Step"
      image={<img src="/step.svg" width="32" height="32" />}
    >
      <Group vertical>
        <PrimaryInput type="number" value={step} />
        <PrimaryInput
          type="number"
          label="Grid"
          units="s"
          value={step}
          autoSize
        />
      </Group>
    </Module>
  );
};
