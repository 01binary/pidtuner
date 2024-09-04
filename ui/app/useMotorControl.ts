"use client";

import { useEffect, useMemo } from "react";

export const DEFAULT_ADDDRESS = "0.0.0.0:8080";

const VELOCITY_TOPIC = "/velocity_feedback";
const VELOCITY_TYPE = "pidtuner/VelocityFeedback";
const DEFAULT_PARAMS = {};

type RosTime = {
  secs: number;
  nsecs: number;
};

export type VelocityFeedback = {
  command: number;
  LPWM: number;
  RPWM: number;
  absolute: number;
  quadrature: number;
  time: RosTime;
  start: RosTime;
  elapsed: number;
  dt: number;
  step: number;
};

export type VelocityCommand = {
  command: number;
};

export type Step = {
  command: number;
  duration: number;
};

export type Steps = {
  steps: Step[];
  loop: boolean;
};

export type PositionFeedback = {
  position: number;   // Absolute encoder position
  goal: number;       // Absolute encoder goal position
  tolerance: number;  // Position tolerance (reached goal)
  pe: number;         // Proportional error
  ie: number;         // Integral error
  de: number;         // Derivative error
  p: number;          // Proportional command
  i: number;          // Integral command
  d: number;          // Derivative command
}

export type EmergencyStop = {
  stop: boolean;
};

export type ConfigurationCommand = {
  // Pins
  LPWMpin: number;           // LPWM pin for driving motor
  RPWMpin: number;           // RPWM pin for driving motor
  ADCpin: number;            // ADC pin for analog/PWM absolute encoder (0 to disable)
  csPin: number;             // Select pin for SPI absolute encoder (0 to disable)
  Apin: number;              // A pin for quadrature encoder
  Bpin: number;              // B pin for quadrature encoder

  // Limits
  pwmMin: number;            // Min PWM value
  pwmMax: number;            // Max PWM value
  absoluteMin: number;       // Min absolute encoder value
  absoluteMax: number;       // Max absolute encoder value

  // Inversions
  pwmInvert: boolean;         // Whether to invert PWM commands
  absoluteInvert : boolean;   // Whether to invert absolute encoder readings
  quadratureInvert: boolean;  // Whether to invert quadrature encoder readings

  // Tuning
  Kp: number                  // P gain
  Ki: number                  // I gain
  Kd: number                  // D gain
  iMin: number                // Min integral
  iMax: number                // Max integral
};

type Params = {
  address?: string;
  onConnection?: () => void;
  onError?: (error: any) => void;
  onVelocity?: (feedback: VelocityFeedback) => void;
};

export const useMotorControl = ({
  address = DEFAULT_ADDDRESS,
  onConnection,
  onError,
  onVelocity
}: Params = DEFAULT_PARAMS) => {
  const ros = useMemo(() => {
    if (!global.ROSLIB) return;

    const instance = new ROSLIB.Ros({ url: `ws://${address}` });

    if (onConnection) {
      instance.on('connection', onConnection);
    }

    if (onError) {
      instance.on('error', onError);
    }

    return instance;
  }, [onConnection, onError, address]);

  useEffect(() => {
    if (!onVelocity) return;

    const velocityFeedbackTopic = new ROSLIB.Topic({
      ros,
      name: VELOCITY_TOPIC,
      messageType: VELOCITY_TYPE
    });

    velocityFeedbackTopic.subscribe(onVelocity);

    () => velocityFeedbackTopic.unsubscribe();
  }, [ros, onVelocity]);

  return ros;
};

export const rosTimeToSec = (rosTime: RosTime) => (
  rosTime.secs + rosTime.nsecs * 1e9
);
