import { PlotType } from "./PlotType";

export const formatSamples = (samples: PlotType[]) => (
  samples
    .reduce((lines, { time, command, absolute }) => ([
      ...lines,
      `${time},${command},${absolute}`
    ]), ['time,command,absolute'])
    .join('\r\n')
)
