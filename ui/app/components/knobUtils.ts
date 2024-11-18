import { MouseEventHandler, useCallback, useEffect, useRef, useState } from "react";

export const RAD_TO_DEG = 57.2958;
export const EPSILON = 0.0001;

export const getValueFromAngle = (angle: number, isFullRange: boolean) => {
  if (isFullRange) {
    return Math.max(0, (angle + Math.PI / 2) / (Math.PI * 2))
  } else {
    return angle / Math.PI;
  }
};

export const getAngleFromValue = (norm: number, isFullRange: boolean) => {
  return isFullRange
    ? norm * 2 * Math.PI - Math.PI / 2
    : norm * Math.PI;
};

export const getAngleFromPoint = (
  x: number,
  y: number,
  cx: number,
  cy: number,
  ox: number,
  oy: number,
  isFullRange: boolean
) => {
  let sin = y - (cy - oy);
  let cos = x - (cx - ox);

  const magnitude = Math.sqrt(sin * sin + cos * cos);
  sin = sin / magnitude;
  cos = cos / magnitude;

  const angle = Math.atan2(sin, cos);

  return isFullRange
    ? angle
    : angle + Math.PI / 2;;
};

type KnobProps = {
  value: number;
  wrap?: boolean;
  handleChange: (value: number) => void;
  centerX?: number;
  centerY?: number;
  isFullRange?: boolean;
}

export const useKnob = ({
  value = 0,
  wrap = true,
  handleChange,
  centerX,
  centerY,
  isFullRange = false
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
    setAngle(getAngleFromValue(0, isFullRange));
  }, [isFullRange])

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
    const nextAngle = getAngleFromValue(value, isFullRange);

    if (Math.abs(nextAngle - angle) > EPSILON) {
      setAngle(nextAngle);
    }
  }, [value, angle]);

  const handleMouseDown: MouseEventHandler = useCallback((e) => {
    e.preventDefault();
    isMouseDownRef.current = true;

    const { offsetX, offsetY } = e.nativeEvent;
    const initialOffset = getAngleFromPoint(
      offsetX, offsetY, knobCenterX, knobCenterY, originX, originY, isFullRange);

    if (isFullRange) {
      setOffsetAngle(initialOffset);
      console.log({initialOffset})
    } else {
      setOffsetAngle(initialOffset - angle);
    }
  }, [knobCenterX, knobCenterY, originX, originY, angle, isFullRange]);

  const handleMouseUp: MouseEventHandler = useCallback((e) => {
    if (e.target.tagName === 'INPUT') {
      return;
    }

    e.preventDefault();
    isMouseDownRef.current = false;
  }, []);

  const handleMouseMove: MouseEventHandler = useCallback((e) => {
    e.preventDefault();

    if (isMouseDownRef.current) {
      const { offsetX, offsetY } = e.nativeEvent;

      const currentAngle = getAngleFromPoint(
        offsetX, offsetY, knobCenterX, knobCenterY, originX, originY, isFullRange);

      if (isFullRange) {
        const nextAngle = currentAngle - offsetAngle - Math.PI / 2;
        const nextValue = getValueFromAngle(nextAngle, isFullRange);
        const delta = value - nextValue;

        // Prevent crossing over
        if (Math.abs(delta) > 0.5) {
          console.warn('crossover guard')
          handleChange(delta < 0 ? 0 : 1)
        } else {
          console.log('ok', {delta})
          handleChange(nextValue);
        }
      } else {
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

        const nextValue = getValueFromAngle(nextAngle, isFullRange);
        handleChange(nextValue);
      }
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
