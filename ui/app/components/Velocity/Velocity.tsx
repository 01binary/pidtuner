"use client";

import { useState } from "react";
import { PrimaryInput } from "../PrimaryInput";
import { Separator } from "../Separator";
import { VelocityKnob } from "./VelocityKnob";

export const Velocity = () => {
  const [velocity, setVelocity] = useState(0);

  return (
  <section className="module">
    <section className="module__header">
      <img src="/velocity.svg" width="32" height="32" />
      <h2>Velocity</h2>
    </section>

    <section className="module__controls">
      <PrimaryInput
        type="number"
        value={Math.round(velocity * 100)}
        onChange={(e) => setVelocity(e.target.value)}
        min="-100"
        max="100"
        step="1"
      />
     
      <VelocityKnob
        velocity={velocity}
        handleChange={setVelocity}
      />
    </section>
  </section>
);
}