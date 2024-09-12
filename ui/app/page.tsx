"use client";

import { useCallback, useState, useRef, useEffect } from "react";
import {
  DEFAULT_ADDDRESS,
  VelocityFeedback,
  useMotorControl,
  rosTimeToSec,
  StepCommand,
  ControlMode
} from "./useMotorControl";
import { Plot } from "./components/Plot";
import { PlotType } from "./components/Plot/PlotType";
import { Velocity } from "./components/Velocity";
import { Position } from "./components/Position";
import { Steps } from "./components/Steps";
import { Settings } from "./components/Settings";

const Page = () => {
  const [address, setAddress] = useState(DEFAULT_ADDDRESS);
  const [isConnected, setConnected] = useState(false);
  const [isCapturing, setCapturing] = useState<boolean>(true);
  const [mode, setMode] = useState<ControlMode>(0);
  const [isEmergencyStop, setEmergencyStop] = useState<boolean>(false);
  const [data, setData] = useState<PlotType[]>([]);
  const [sequenceTime, setSequenceTime] = useState(0);
  const [step, setStep] = useState(0);
  const [volts, setVolts] = useState(0);
  const [amps, setAmps] = useState(0);
  const isCapturingRef = useRef(isCapturing);
  const firstTimeRef = useRef(0);

  console.log('volts', volts, 'amps', amps)

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

    if (!firstTimeRef.current) firstTimeRef.current = time;

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
        quadrature: velocity.quadrature
      }));
    }
  }, []);

  const {
    publishVelocity,
    publishEstop,
    publishSteps
  } = useMotorControl({
    address,
    onConnection: handleConnection,
    onVelocity: handleVelocity,
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
        {/*<Position />*/}
        <Steps
          time={sequenceTime}
          step={step}
          isPlaying={mode === ControlMode.STEP}
          setStep={setStep}
          publishSteps={handlePublishSteps}
        />
        {/*<Settings />*/}
      </main>
    </>
  );
};

export default Page;
