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

  const isCapturingRef = useRef(isCapturing);
  const firstTimeRef = useRef(0);

  useEffect(() => {
    isCapturingRef.current = isCapturing;
  }, [isCapturing]);

  const handleConnection = useCallback(() => {
    setConnected(true);
  }, []);

  const handleError = useCallback(() => {
    setConnected(false);
  }, []);

  const handleVelocity = useCallback((velocity: VelocityFeedback) => {
    const time = rosTimeToSec(velocity.time);
    const start = rosTimeToSec(velocity.start);

    if (!firstTimeRef.current) {
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
      setData(d => d.concat({
        time: time - firstTimeRef.current,
        command: velocity.command,
        absolute: velocity.absolute,
        quadrature: velocity.quadrature,
        goal
      }));
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
    console.log('setting', command)
    setConfig(command);
    publishConfiguration(command);
  }, [publishConfiguration])

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
          publishConfiguration={publishConfiguration}
        />
      </main>
    </>
  );
};

export default Page;
