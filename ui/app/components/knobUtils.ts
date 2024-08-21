export const MAX_ANGLE = Math.PI;
export const BIAS_ANGLE = Math.PI / 2;

export const RAD_TO_DEG = 57.2958;

export const getValueFromAngle = (angle: number, invert?: boolean) => {
  const norm = angle / MAX_ANGLE;
  return invert ? -norm : norm;
};

export const getAngleFromValue = (norm: number, invert?: boolean) => {
  return (invert ? -norm : norm) * MAX_ANGLE;
};

export const getAngleFromPoint = (x: number, y: number, cx: number, cy: number, ox: number, oy: number) => {
  const sin = y - (cy - oy);
  const cos = x - (cx - ox);
  return Math.atan2(sin, cos) + BIAS_ANGLE;
};