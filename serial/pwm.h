void commandToPwm(double command, uint8_t& lpwm, uint8_t& rpwm)
{
  if (command < 0.0)
    {
      // -RPWM
      rpwm = (uint8_t)abs(command);
      lpwm = 0;
    }
    else
    {
      // +LPWM
      lpwm = (uint8_t)command;
      rpwm = 0;
    }
}

double pwmToCommand(uint8_t lpwm, uint8_t rpwm)
{
  return double(lpwm - rpwm);
}
