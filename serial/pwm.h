void commandToPwm(double command, uint8_t& lpwm, uint8_t& rpwm)
{
  if (command < 0.0)
    {
      // -LPWM
      lpwm = (uint8_t)abs(command);
      rpwm = 0;
    }
    else
    {
      // +RPWM
      rpwm = (uint8_t)command;
      lpwm = 0;
    }
}

double pwmToCommand(uint8_t lpwm, uint8_t rpwm)
{
  return double(rpwm - lpwm);
}
