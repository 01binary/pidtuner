/*
    pid.h
    Simple PID algorithm with integral limit.
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros.h>

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class PID
{
public:
  float Kp;    // Proportional gain
  float Ki;    // Integral gain
  float Kd;    // Derivative gain

  float pe;    // Proportional error
  float ie;    // Integral error
  float de;    // Derivative error

  float iMin;  // Min integral
  float iMax;  // Max integral

  float p;     // Proportional term
  float i;     // Integral term
  float d;     // Derivative term

public:
  PID():
    Kp(1), Ki(0.1), Kd(0.1),
    pe(0.0), ie(0.0), de(0.0),
    iMin(0), iMax(1.0),
    p(0.0), i(0.0), d(0.0)
  {
  }

public:
  float getCommand(float error, float dt)
  {
    // Calculate integral error
    ie += dt * pe;

    // Limit integral error
    if (Ki && iMax != 0.0 && iMin != 0.0) {
      ie = constrain(ie, iMin / Ki, iMax / Ki);
    }

    // Calculate derivative error
    de = (error - pe) / dt;
    pe = error;

    // Calculate proportional contribution
    p = Kp * pe;

    // Calculate integral contribution
    i = Ki * ie;

    // Limit integral contribution
    if (Ki && iMax != 0.0 && iMin != 0) {
      i = constrain(i, iMin, iMax);
    }

    // Calculate derivative contribution
    d = Kd * de;

    // Calculate command
    return p + i + d;
  }

  void configure(
    float Kp,
    float Ki,
    float Kd,
    float iMin,
    float iMax)
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
