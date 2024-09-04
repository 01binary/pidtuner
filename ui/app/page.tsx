"use client";

import { useCallback, useState } from "react";
import {
  DEFAULT_ADDDRESS,
  VelocityFeedback,
  useRosBridge
} from "./useRosBridge";
import { Plot } from "./components/Plot";
import { PlotType } from "./components/Plot/PlotType";
import { Velocity } from "./components/Velocity";
import { Position } from "./components/Position";
import { Steps } from "./components/Steps";
import { Settings } from "./components/Settings";

const Page = () => {
  const [address, setAddress] = useState(DEFAULT_ADDDRESS);
  const [isCapturing, setCapturing] = useState<boolean>(true);
  const [data, setData] = useState<PlotType[]>([]);

  const handleConnection = useCallback(() => {

  }, []);

  const handleVelocity = useCallback((velocity: VelocityFeedback) => {

  }, []);

  useRosBridge({
    address,
    onConnection: handleConnection,
    onVelocity: handleVelocity
  })

  return (
    <>
      <header>
        <Plot
          {...{ data, isCapturing, setCapturing }}
        />
      </header>

      <main>
        <Velocity />
        {/*<Position />*/}
        <Steps />
        {/*<Settings />*/}
      </main>
    </>
  );
};

export default Page;
