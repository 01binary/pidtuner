"use client";

import { useCallback, useEffect, useMemo } from "react";

export const DEFAULT_ADDDRESS = "0.0.0.0:8080";

const VELOCITY_FEEDBACK_TOPIC = "/velocity_feedback";
const VELOCITY_FEEDBACK_TYPE = "pidtuner/VelocityFeedback";
const VELOCITY_COMMAND_TOPIC = "/velocity";
const VELOCITY_COMMAND_TYPE = "pidtuner/VelocityCommand";

const POSITION_FEEDBACK_TOPIC = "/position_feedback";
const POSITION_FEEDBACK_TYPE = "pidtuner/PositionFeedback";
const POSITION_COMMAND_TOPIC = "/position";
const POSITION_COMMAND_TYPE = "pidtuner/PositionCommand";

const STEP_COMMAND_TOPIC = "/step";
const STEP_COMMAND_TYPE = "pidtuner/Steps";

const ESTOP_COMMAND_TOPIC = "/estop";
const ESTOP_COMMAND_TYPE = "pidtuner/EmergencyStop";

const DEFAULT_PARAMS = {};

type RosTime = {
  secs: number;
  nsecs: number;
};

export enum ControlMode {
  VELOCITY = 0,       // Velocity control mode
  POSITION = 1,       // Goal position and tolerance mode
  STEP = 2            // Step performance playback mode
}

export type VelocityFeedback = {
  command: number;    // Last PWM command
  absolute: number;   // Absolute encoder position
  quadrature: number; // Quadrature encoder position
  time: RosTime;      // Current time
  start: RosTime;     // Last command time
  elapsed: number;    // Time elapsed since last command
  dt: number;         // Time step
  step: number;       // Performance step index
  estop: boolean;     // Emergency stop active
  mode: ControlMode;  // Current state
  amps: number;       // Current reading
  volts: number;      // Voltage reading
};

export type PositionFeedback = {
  position: number;   // Absolute encoder position (normalized)
  goal: number;       // Goal (PID controller setpoint) position
  tolerance: number;  // Tolerance for stopping when goal is reached
  pe: number;         // Proportional error
  ie: number;         // Integral error
  de: number;         // Derivative error
  p: number;          // Proportional command
  i: number;          // Integral command
  d: number;          // Derivative command
};

export type VelocityCommand = {
  command: number;    // PWM command to send
};

export type PositionCommand = {
  goal: number;       // Goal position to send
  tolerance: number;  // Position tolerance to send
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
  onPosition?: (feedback: PositionFeedback) => void;
};

export const useMotorControl = ({
  address = DEFAULT_ADDDRESS,
  onConnection,
  onError,
  onVelocity,
  onPosition
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
    if (!ros) return;

    let velocityFeedbackTopic = null;
    let positionFeedbackTopic = null;

    if (onVelocity) {
      velocityFeedbackTopic = new ROSLIB.Topic({
        ros,
        name: VELOCITY_FEEDBACK_TOPIC,
        messageType: VELOCITY_FEEDBACK_TYPE
      });

      velocityFeedbackTopic.subscribe(onVelocity);
    }

    if (onPosition) {
      positionFeedbackTopic = new ROSLIB.Topic({
        ros,
        name: POSITION_FEEDBACK_TOPIC,
        messageType: POSITION_FEEDBACK_TYPE
      });

      positionFeedbackTopic.subscribe(onPosition);
    }

    () => {
      velocityFeedbackTopic?.unsubscribe();
      positionFeedbackTopic?.unsubscribe();
    }
  }, [ros, onPosition, onVelocity]);

  const velocityPublisher = useMemo(() => {
    if (!ros) return;

    const velocityTopic = new ROSLIB.Topic({
      ros,
      name: VELOCITY_COMMAND_TOPIC,
      messageType: VELOCITY_COMMAND_TYPE
    });

    return velocityTopic;
  }, [ros]);

  const positionPublisher = useMemo(() => {
    if (!ros) return;

    const positionTopic = new ROSLIB.Topic({
      ros,
      name: POSITION_COMMAND_TOPIC,
      messageType: POSITION_COMMAND_TYPE
    });

    return positionTopic;
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

  }, [ros, velocityPublisher]);

  const publishPosition = useCallback((position: PositionCommand) => {
    if (!ros) return;

    const message = new ROSLIB.Message(position);
    positionPublisher.publish(message);

  }, [ros, positionPublisher]);

  const publishSteps = useCallback((steps: StepCommand) => {
    if (!ros) return;

    const message = new ROSLIB.Message(steps);
    stepPublisher.publish(message);

  }, [ros, stepPublisher]);

  const publishEstop = useCallback((estop: EmergencyStopCommand) => {
    if (!ros) return;

    const message = new ROSLIB.Message(estop);
    estopPublisher.publish(message);

  }, [ros, estopPublisher]);

  return {
    publishVelocity,
    publishPosition,
    publishEstop,
    publishSteps
  };
};

export const rosTimeToSec = (rosTime: RosTime) => (
  rosTime.secs + rosTime.nsecs / 1e9
);
