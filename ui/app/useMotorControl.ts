"use client";

import { useCallback, useEffect, useMemo } from "react";
import pkg from "../package.json";

export const DEFAULT_ADDRESS = pkg.settings.serverAddress;

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

const CONFIG_COMMAND_TOPIC = "/configuration";
const CONFIG_COMMAND_TYPE = "pidtuner/Configuration";

const CONFIG_SERVER = "/configuration";
const CONFIG_SERVER_TYPE = "pidtuner/ConfigurationServer";

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

export type ConfigurationServer = {
  configuration: ConfigurationCommand;
};

export const DEFAULT_CONFIGURATION = {
  LPWMpin: 11,
  RPWMpin: 3,
  ADCpin: 0,
  csPin: 53,
  Apin: 18,
  Bpin: 19,
  pwmMin: 0,
  pwmMax: 1,
  absoluteMin: 0,
  absoluteMax: 1,
  pwmInvert: false,
  absoluteInvert: false,
  quadratureInvert: true,
  pulsesPerRevolution: 0,
  Kp: 1,
  Ki: 0.1,
  Kd: 0.1,
  iMin: -1,
  iMax: 1
}

type Params = {
  address?: string;
  onConnection?: () => void;
  onError?: (error: any) => void;
  onVelocity?: (feedback: VelocityFeedback) => void;
  onPosition?: (feedback: PositionFeedback) => void;
};

export const useMotorControl = ({
  address = DEFAULT_ADDRESS,
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

  const configurationPublisher = useMemo(() => {
    if (!ros) return;

    const configurationTopic = new ROSLIB.Topic({
      ros,
      name: CONFIG_COMMAND_TOPIC,
      messageType: CONFIG_COMMAND_TYPE
    });

    return configurationTopic;
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

  const configurationService = useMemo(() => {
    if (!ros) return;

    const configurationClient = new ROSLIB.Service({
      ros,
      name: CONFIG_SERVER,
      serviceType: CONFIG_SERVER_TYPE
    });

    return configurationClient;
  }, [ros])

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

  const publishConfiguration = useCallback((config: ConfigurationCommand) => {
    if (!ros) return;

    const message = new ROSLIB.Message(config);
    configurationPublisher.publish(message);

  }, [ros, configurationPublisher]);

  const requestConfiguration = useCallback(() => {
    if (!ros) return;

    return new Promise<ConfigurationServer>((resolve, reject) => {
      try {
        configurationService.callService(
          new ROSLIB.ServiceRequest(),
          (result: ConfigurationServer) => resolve(result));
      } catch(e) {
        reject(e);
      }
    });
  }, [ros, configurationService]);

  return {
    publishVelocity,
    publishPosition,
    publishConfiguration,
    requestConfiguration,
    publishEstop,
    publishSteps
  };
};

export const rosTimeToSec = (rosTime: RosTime) => (
  rosTime.secs + rosTime.nsecs / 1e9
);
