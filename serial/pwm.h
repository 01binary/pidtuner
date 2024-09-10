/*
    pwm.h
    PWM command utilities.
*/

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const float MAX_PWM = 0b11111111;

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

float clamp(float value, float min, float max)
{
  if (value < min)
    return min;
  else if (value > max)
    return max;
  
  return value;
}

void commandToPwm(float velocity, float min, float max, bool invert, uint8_t& lpwm, uint8_t& rpwm)
{
  float command = clamp(abs(velocity), min, max);

  if (invert)
  {
    command = 1.0 - command;
  }

  if (velocity < 0.0)
  {
    // -RPWM
    rpwm = uint8_t(command * MAX_PWM);
    lpwm = 0;
  }
  else
  {
    // +LPWM
    lpwm = uint8_t(command * MAX_PWM);
    rpwm = 0;
  }
}
