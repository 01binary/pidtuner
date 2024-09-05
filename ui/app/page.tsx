"use client";

import { useCallback, useState, useRef } from "react";
import {
  DEFAULT_ADDDRESS,
  VelocityFeedback,
  useMotorControl,
  rosTimeToSec
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
  const [data, setData] = useState<PlotType[]>([]);

  const handleConnection = useCallback(() => {
    setConnected(true);
  }, []);

  const handleError = useCallback((e) => {
    setConnected(false);
  }, []);

  const handleVelocity = useCallback((velocity: VelocityFeedback) => {
    if (!isCapturing) return;

    setData(d => d.concat({
      time: rosTimeToSec(velocity.time),
      command: velocity.command,
      absolute: velocity.absolute
    }))
  }, [isCapturing]);

  const { publishVelocity } = useMotorControl({
    address,
    onConnection: handleConnection,
    onVelocity: handleVelocity,
    onError: handleError
  });

  return (
    <>
      <header>
        <Plot
          {...{
            data,
            isCapturing,
            setCapturing,
            server: address,
            onServerChange: () => setAddress,
            isConnected
          }}
        />
      </header>

      <main>
        <Velocity publishVelocity={publishVelocity} />
        {/*<Position />*/}
        <Steps />
        {/*<Settings />*/}
      </main>
    </>
  );
};

export default Page;
