import { PrimaryInput } from "../PrimaryInput";
import { Separator } from "../Separator";
import styles from "./Velocity.module.scss";
import { VelocityKnob } from "./VelocityKnob";

export const Velocity = () => (
  <section className="module">
    <section className="module__header">
      <img src="/velocity.svg" width="32" height="32" />
      <h2>Velocity</h2>
    </section>

    <section className="module__controls">
      <PrimaryInput type="number" value={100} min="-100" max="100" step="1" />
      <Separator />
      <VelocityKnob />
      <Separator invert />
    </section>
  </section>
)