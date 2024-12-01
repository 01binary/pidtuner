"use client";

import {
  useCallback,
  useState,
  useRef,
  useEffect
} from "react";
import {
  DEFAULT_ADDRESS,
  DEFAULT_CONFIGURATION,
  VelocityFeedback,
  PositionFeedback,
  useMotorControl,
  StepCommand,
  ControlMode,
  rosTimeToSec,
  ConfigurationCommand
} from "./useMotorControl";
import { Plot } from "./components/Plot";
import { PlotType } from "./components/Plot/PlotType";
import { Configuration } from "./components/Configuration";
import { Velocity } from "./components/Velocity";
import { Position } from "./components/Position";
import { Steps } from "./components/Steps";

const RECORD_COMMAND_THRESHOLD = 0.01;
const RECORD_TIME_THRESHOLD = 1;

const Page = () => {
  const [address, setAddress] = useState(DEFAULT_ADDRESS);
  const [config, setConfig] = useState<ConfigurationCommand>(DEFAULT_CONFIGURATION);
  const [isConnected, setConnected] = useState(false);
  const [isCapturing, setCapturing] = useState<boolean>(true);
  const [mode, setMode] = useState<ControlMode>(0);
  const [isEmergencyStop, setEmergencyStop] = useState<boolean>(false);
  const [data, setData] = useState<PlotType[]>([]);
  const [sequenceTime, setSequenceTime] = useState(0);
  const [step, setStep] = useState(0);
  const [volts, setVolts] = useState(0);
  const [amps, setAmps] = useState(0);
  const [position, setPosition] = useState(0);
  const [goal, setGoal] = useState(0);
  const [pe, setPe] = useState(0);
  const [ie, setIe] = useState(0);
  const [de, setDe] = useState(0);

  const goalRef = useRef(goal);
  const isCapturingRef = useRef(isCapturing);
  const firstTimeRef = useRef(0);

  useEffect(() => {
    // Some handlers cannot have state in dependency list
    isCapturingRef.current = isCapturing;
  }, [isCapturing]);

  const handleConnection = useCallback(() => {
    setConnected(true);
  }, []);

  const handleError = useCallback(() => {
    setConnected(false);
  }, []);

  const lastNonZeroVelocityTimeRef = useRef(0);

  const handleVelocity = useCallback((velocity: VelocityFeedback) => {
    const time = rosTimeToSec(velocity.time);
    const start = rosTimeToSec(velocity.start);

    if (!firstTimeRef.current) {
      // Allow continuing capture on page reload
      firstTimeRef.current = time;
    }

    setEmergencyStop(velocity.estop);
    setMode(velocity.mode);
    setVolts(velocity.volts);
    setAmps(velocity.amps);

    if (velocity.mode === ControlMode.STEP) {
      setStep(velocity.step);
      setSequenceTime(time - start);
    } else {
      setSequenceTime(0);
    }

    if (isCapturingRef.current) {
      const isStopped = velocity.mode === ControlMode.STEP
        ? velocity.estop
        : Math.abs(velocity.command) < RECORD_COMMAND_THRESHOLD;

      const pause = isStopped &&
        time - lastNonZeroVelocityTimeRef.current > RECORD_TIME_THRESHOLD

      if (!pause || firstTimeRef.current === time) {
        // Don't capture if the motor is not doing anything for a while
        setData(d => d.concat({
          time: time - firstTimeRef.current,
          command: velocity.command,
          absolute: velocity.absolute,
          quadrature: velocity.quadrature,
          goal: goalRef.current
        }));
      }

      if (velocity.mode !== ControlMode.POSITION) {
        // Always track position
        setPosition(velocity.absolute);
        setGoal(velocity.absolute);
        goalRef.current = velocity.absolute;
      }

      if (Math.abs(velocity.command) > RECORD_COMMAND_THRESHOLD) {
        lastNonZeroVelocityTimeRef.current = time;
      }
    }
  }, []);

  const handlePosition = useCallback(({
    position,
    goal,
    pe,
    ie,
    de
  }: PositionFeedback) => {
    // Update normalized position
    setPosition(position);
    // Update normalized goal position
    setGoal(goal);

    // Keep reference to goal so velocity handler can log it
    goalRef.current = goal;

    // Update current proportional error
    setPe(pe);
    // Update current integral error
    setIe(ie);
    // Update current derivative error
    setDe(de);
  }, []);

  const {
    publishPosition,
    publishVelocity,
    publishConfiguration,
    requestConfiguration,
    publishEstop,
    publishSteps
  } = useMotorControl({
    address,
    onConnection: handleConnection,
    onVelocity: handleVelocity,
    onPosition: handlePosition,
    onError: handleError
  });

  const handleEStop = useCallback(() => {
    publishEstop({ stop: !isEmergencyStop})
    setEmergencyStop(!isEmergencyStop);
  }, [publishEstop, isEmergencyStop]);

  const handlePublishSteps = useCallback((command: StepCommand) => {
    setEmergencyStop(false);
    publishSteps(command);
  }, [publishSteps]);

  const handlePublishConfiguration = useCallback((command: ConfigurationCommand) => {
    setConfig(command);
    publishConfiguration(command);
  }, [publishConfiguration]);

  useEffect(() => {
    // Request initial configuration
    requestConfiguration()?.then(({ configuration }) => setConfig({
      ...configuration,
      Kp: Math.round(configuration.Kp * 1e8) / 1e8,
      Ki: Math.round(configuration.Ki * 1e8) / 1e8,
      Kd: Math.round(configuration.Kd * 1e8) / 1e8
    }));
  }, [requestConfiguration]);

  return (
    <>
      <header>
        <Plot
          {...{
            data,
            mode,
            isCapturing,
            setCapturing,
            server: address,
            onServerChange: () => setAddress,
            onEStop: handleEStop,
            isEmergencyStop,
            isConnected
          }}
        />
      </header>

      <main>
        <Velocity
          publishVelocity={publishVelocity}
          volts={volts}
          amps={amps}
        />
        <Position
          goal={goal}
          position={position}
          configuration={config}
          publishPosition={publishPosition}
          publishConfiguration={handlePublishConfiguration}
          pe={pe}
          ie={ie}
          de={de}
        />
        <Steps
          time={sequenceTime}
          step={step}
          isPlaying={mode === ControlMode.STEP}
          setStep={setStep}
          publishSteps={handlePublishSteps}
        />
        <Configuration
          configuration={config}
          publishConfiguration={handlePublishConfiguration}
        />
      </main>
    </>
  );
};

export default Page;
