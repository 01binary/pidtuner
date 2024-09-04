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

type Params = {
  address?: string;
  onConnection?: () => void;
  onError?: (error: any) => void;
  onVelocity?: (feedback: VelocityFeedback) => void;
};

export const useRosBridge = ({
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
