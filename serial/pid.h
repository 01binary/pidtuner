#include <ros.h>

class PID
{
public:
  double Kp;    // Proportional gain
  double Ki;    // Integral gain
  double Kd;    // Derivative gain

  double pe;    // Proportional error
  double ie;    // Integral error
  double de;    // Derivative error

  double iMin;  // Min integral
  double iMax;  // Max integral

  double p;     // Proportional term
  double i;     // Integral term
  double d;     // Derivative term

public:
  PID():
    Kp(10.0), Ki(1.0), Kd(1.0),
    pe(0.0), ie(0.0), de(0.0),
    iMin(0.0), iMax(0.0),
    p(0.0), i(0.0), d(0.0)
  {
  }

public:
  double getCommand(double error, double dt)
  {
    // Calculate integral error
    ie += dt * pe;

    // Limit integral error
    if (Ki && iMax) ie = constrain(ie, iMin / Ki, iMax / Ki);

    // Calculate derivative error
    de = (error - pe) / dt;
    pe = error;

    // Calculate proportional contribution
    p = Kp * pe;

    // Calculate integral contribution
    i = Ki * ie;

    // Limit integral contribution
    if (Ki && iMax) i = constrain(i, iMin, iMax);

    // Calculate derivative contribution
    d = Kd * de;

    // Calculate command
    return p + i + d;
  }

  void configure(
    double Kp,
    double Ki,
    double Kd,
    double iMin,
    double iMax)
  {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->iMin = iMin;
    this->iMax = iMax;
  }

  void reset()
  {
    pe = 0.0;
    ie = 0.0;
    de = 0.0;
    p = 0.0;
    i = 0.0;
    d = 0.0;
  }
};
