export const SETTINGS = [
  {
    name: 'LPWMpin',
    label: 'LPWM Pin',
    type: 'number',
    min: 0,
    max: 100,
    step: 1
  },
  {
    name: 'RPWMpin',
    label: 'RPWM Pin',
    type: 'number',
    min: 0,
    max: 100,
    step: 1
  },
  {
    name: 'ADCpin',
    label: 'ADC Pin',
    type: 'number',
    min: 0,
    max: 15,
    step: 1
  },
  {
    name: 'csPin',
    label: 'CS Pin',
    type: 'number',
    min: 0,
    max: 100,
    step: 1
  },
  {
    name: 'Apin',
    label: 'A Pin',
    type: 'number',
    min: 0,
    max: 100,
    step: 1
  },
  {
    name: 'Bpin',
    label: 'B Pin',
    type: 'number',
    min: 0,
    max: 100,
    step: 1
  },
  {
    name: 'pwmMin',
    label: 'PWM Min',
    type: 'number',
    min: -1000,
    max: 1000,
    step: 1
  },
  {
    name: 'pwmMax',
    label: 'PWM Max',
    type: 'number',
    min: -1000,
    max: 1000,
    step: 1
  },
  {
    name: 'absoluteMin',
    label: 'Absolute Min',
    type: 'number',
    min: -1000,
    max: 1000,
    step: 0.001
  },
  {
    name: 'absoluteMax',
    label: 'Absolute Max',
    type: 'number',
    min: -1000,
    max: 1000,
    step: 0.001
  },
  {
    name: 'pwmInvert',
    label: 'PWM Invert',
    type: 'boolean'
  },
  {
    name: 'absoluteInvert',
    label: 'Absolute Invert',
    type: 'boolean'
  },
  {
    name: 'quadratureInvert',
    label: 'Quadrature Invert',
    type: 'boolean'
  },
  {
    name: 'Kp',
    label: 'Proportional gain',
    type: 'number',
    min: -10000,
    max: 10000
  },
  {
    name: 'Ki',
    label: 'Integral gain',
    type: 'number',
    min: -10000,
    max: 10000
  },
  {
    name: 'Kd',
    label: 'Derivative gain',
    type: 'number',
    min: -10000,
    max: 10000
  },
  {
    name: 'iMin',
    label: 'Integral Min',
    type: 'number',
    min: -10000,
    max: 10000
  },
  {
    name: 'iMax',
    label: 'Integral Max',
    type: 'number',
    min: -10000,
    max: 10000
  }
];
