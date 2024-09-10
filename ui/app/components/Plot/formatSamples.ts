import { PlotType } from "./PlotType";

export const formatSamples = (samples: PlotType[]) => (
  samples
    .reduce((lines, { time, command, absolute, quadrature }) => ([
      ...lines,
      `${time},${command},${absolute},${quadrature}`
    ]), ['time,command,absolute,quadrature'])
    .join('\r\n')
)
