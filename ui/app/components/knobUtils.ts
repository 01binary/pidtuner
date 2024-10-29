import { useCallback, useEffect, useRef, useState } from "react";

export const MAX_ANGLE = Math.PI;
export const BIAS_ANGLE = Math.PI / 2;
export const RAD_TO_DEG = 57.2958;

export const getValueFromAngle = (angle: number) => {
  const norm = angle / MAX_ANGLE;
  return norm;
};

export const getAngleFromValue = (norm: number) => {
  return norm * MAX_ANGLE;
};

export const getAngleFromPoint = (
  x: number,
  y: number,
  cx: number,
  cy: number,
  ox: number,
  oy: number) => {
  const sin = y - (cy - oy);
  const cos = x - (cx - ox);
  return Math.atan2(sin, cos) + BIAS_ANGLE;
};

type KnobProps = {
  value: number;
  wrap?: boolean;
  handleChange: (value: number) => void;
  centerX?: number;
  centerY?: number;
}

export const useKnob = ({
  value,
  wrap = true,
  handleChange,
  centerX,
  centerY
}: KnobProps) => {
  const svgRef = useRef<SVGSVGElement>(null);
  const knobRef = useRef<SVGPathElement>(null);
  const [knobCenterX, setKnobCenterX] = useState(centerX ?? 0);
  const [knobCenterY, setKnobCenterY] = useState(centerY ?? 0);
  const [originX, setOriginX] = useState(0);
  const [originY, setOriginY] = useState(0);
  const [angle, setAngle] = useState(0);
  const [offsetAngle, setOffsetAngle] = useState(0);
  const isMouseDownRef = useRef(false);

  useEffect(() => {
    if (!knobRef.current || !svgRef.current) return;

    const { x, y, width, height } = knobRef.current
      .getBoundingClientRect();

    const { x: ox, y: oy } = svgRef.current
      .getBoundingClientRect();

    if (centerX === undefined || centerY === undefined) {
      setKnobCenterX(x + width / 2);
      setKnobCenterY(y + height / 2);
    } else {
      setKnobCenterX(centerX);
      setKnobCenterY(centerY);
    }

    setOriginX(ox);
    setOriginY(oy);
  }, [centerX, centerY]);

  useEffect(() => {
    const nextAngle = getAngleFromValue(value);

    if (Math.abs(nextAngle - angle) > 0.0001) {
      setAngle(nextAngle);
    }
  }, [value, angle]);

  const handleMouseDown = useCallback((e) => {
    e.preventDefault();
    e.stopPropagation();
    isMouseDownRef.current = true;

    const { offsetX, offsetY } = e.nativeEvent;
    const initialOffset = getAngleFromPoint(
      offsetX, offsetY, knobCenterX, knobCenterY, originX, originY);

    setOffsetAngle(initialOffset - angle);
  }, [knobCenterX, knobCenterY, originX, originY, angle]);

  const handleMouseUp = useCallback((e) => {
    e.preventDefault();
    e.stopPropagation();
    isMouseDownRef.current = false;
  }, []);

  const handleMouseMove = useCallback((e) => {
    e.preventDefault();
    e.stopPropagation();

    if (isMouseDownRef.current) {
      const { offsetX, offsetY } = e.nativeEvent;

      const currentAngle = getAngleFromPoint(
        offsetX, offsetY, knobCenterX, knobCenterY, originX, originY);

      let delta = currentAngle - offsetAngle - angle;

      if (wrap && delta < 0) {
        // Wrap
        delta += Math.PI * 2;
      }

      if (delta > Math.PI) {
        // Map 0..360 to -180..180.
        delta -= Math.PI * 2;
      }

      let nextAngle = angle + delta;

      // Prevent crossing over from max positive to max negative
      if (nextAngle > Math.PI || nextAngle < -Math.PI)
        return;

      handleChange(getValueFromAngle(nextAngle));
    }
  }, [
    angle,
    originX,
    originY,
    knobCenterX,
    knobCenterY,
    offsetAngle,
    handleChange,
    wrap
  ]);

  return {
    svgRef,
    knobRef,
    handleMouseDown,
    handleMouseUp,
    handleMouseMove,
    knobCenterX,
    knobCenterY,
    originX,
    originY,
    angle
  }
}
