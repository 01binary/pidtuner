#include <ros.h>
#include <cmath>

struct PID
{
  double Kp;    // Proportional gain
  double Ki;    // Integral gain
  double Kd;    // Derivative gain

  double pe;    // Proportional error
  double ie;    // Integral error
  double de;    // Drivative error

  double iMin;  // Min integral
  double iMax;  // Max integral

  double p;     // Proportional term
  double i;     // Integral term
  double d;     // Derivative term

  PID():
    Kp(10.0), Ki(1.0), Kd(1.0),
    pe(0.0), ie(0.0), de(0.0),
    iMin(0.0), iMax(0.0),
    p(0.0), i(0.0), d(0.0)
  {
  }

  double getCommand(double error, uint64_t dt)
  {
    // Calculate integral error
    ie += (dt / 1e9) * pe;

    // Limit integral error
    if (Ki && iMax) ie = std::clamp(ie, iMin / Ki, iMax / Ki);

    // Calculate derivative error
    de = (error - pe) / (dt / 1e9);
    pe = error;

    // Calculate proportional contribution
    p = Kp * pe;

    // Calculate integral contribution
    i = Ki * ie;

    // Limit integral contribution
    if (Ki && iMax) i = std::clamp(i, iMin, iMax);

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
