"use client";

import { useState } from "react";
import { Group } from "../Group";
import { Module } from "../Module";
import { PrimaryInput } from "../PrimaryInput";
import { Separator } from "../Separator";

export const Steps = () => {
  const [step, setStep] = useState(0);

  return (
    <Module
      title="Step"
      image={<img src="/step.svg" width="32" height="32" />}
    >
      <Group center>
        <Group vertical>
          <PrimaryInput
            type="number"
            value={step}
            autoSize
          />
          <PrimaryInput
            type="number"
            label="Grid"
            units="s"
            value={step}
            autoSize
          />
        </Group>

        <Separator spacing={'16px'} />

        <Group vertical>
          <button>
            <svg
              width="24px"
              height="24px"
              viewBox="0 0 24 24"
            >
              <polygon
                fill="white"
                points="22.4,12 1.6,0 1.6,23.9 "
              />
            </svg>
          </button>

          <button>
            <svg
              width="24px"
              height="24px"
              viewBox="0 0 24 24"
            >
              <rect
                x="1.2"
                y="1.2"
                fill="#EC008C"
                width="21.6"
                height="21.6"
              />
            </svg>
          </button>
        </Group>

        <Separator spacing={'16px'} />
      </Group>
    </Module>
  );
};
