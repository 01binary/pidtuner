const float MAX_PWM = 0b11111111;
const float NORM_PWM = 1 / MAX_PWM;

void commandToPwm(float norm, uint8_t& lpwm, uint8_t& rpwm)
{
  if (norm < 0.0)
  {
    // -RPWM
    rpwm = uint8_t(abs(norm) * MAX_PWM);
    lpwm = 0;
  }
  else
  {
    // +LPWM
    lpwm = uint8_t(norm * MAX_PWM);
    rpwm = 0;
  }
}

float pwmToCommand(uint8_t lpwm, uint8_t rpwm)
{
  return float(lpwm) * NORM_PWM - float(rpwm) * NORM_PWM;
}
