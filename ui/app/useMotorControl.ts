"use client";

import { useCallback, useEffect, useMemo } from "react";

export const DEFAULT_ADDDRESS = "0.0.0.0:8080";

const VELOCITY_FEEDBACK_TOPIC = "/velocity_feedback";
const VELOCITY_FEEDBACK_TYPE = "pidtuner/VelocityFeedback";
const VELOCITY_COMMAND_TOPIC = "/velocity";
const VELOCITY_COMMAND_TYPE = "pidtuner/VelocityCommand";
const STEP_COMMAND_TOPIC = "/step";
const STEP_COMMAND_TYPE = "pidtuner/Steps";
const ESTOP_COMMAND_TOPIC = "/estop";
const ESTOP_COMMAND_TYPE = "pidtuner/EmergencyStop";

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
};

export type VelocityCommand = {
  command: number;
};

export type Step = {
  command: number;
  duration: number;
};

export type StepCommand = {
  steps: Step[];
  loop: boolean;
};

export type EmergencyStopCommand = {
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
    if (!onVelocity || !ros) return;

    const velocityFeedbackTopic = new ROSLIB.Topic({
      ros,
      name: VELOCITY_FEEDBACK_TOPIC,
      messageType: VELOCITY_FEEDBACK_TYPE
    });

    velocityFeedbackTopic.subscribe(onVelocity);

    () => velocityFeedbackTopic.unsubscribe();
  }, [ros, onVelocity]);

  const velocityPublisher = useMemo(() => {
    if (!ros) return;

    const velocityTopic = new ROSLIB.Topic({
      ros,
      name: VELOCITY_COMMAND_TOPIC,
      messageType: VELOCITY_COMMAND_TYPE
    });

    return velocityTopic;
  }, [ros]);

  const stepPublisher = useMemo(() => {
    if (!ros) return;

    const stepTopic = new ROSLIB.Topic({
      ros,
      name: STEP_COMMAND_TOPIC,
      messageType: STEP_COMMAND_TYPE
    });

    return stepTopic;
  }, [ros]);

  const estopPublisher = useMemo(() => {
    if (!ros) return;

    const estopTopic = new ROSLIB.Topic({
      ros,
      name: ESTOP_COMMAND_TOPIC,
      messageType: ESTOP_COMMAND_TYPE
    });

    return estopTopic;
  }, [ros]);

  const publishVelocity = useCallback((velocity: VelocityCommand) => {
    if (!ros) return;

    const message = new ROSLIB.Message(velocity);
    velocityPublisher.publish(message);

  }, [velocityPublisher]);

  const publishSteps = useCallback((steps: StepCommand) => {
    if (!ros) return;

    const message = new ROSLIB.Message(steps);
    stepPublisher.publish(message);

  }, [stepPublisher]);

  const publishEstop = useCallback((estop: EmergencyStopCommand) => {
    if (!ros) return;

    const message = new ROSLIB.Message(estop);
    estopPublisher.publish(message);

  }, [estopPublisher]);

  return {
    publishVelocity,
    publishEstop,
    publishSteps
  };
};

export const rosTimeToSec = (rosTime: RosTime) => (
  rosTime.secs + rosTime.nsecs / 1e9
);
