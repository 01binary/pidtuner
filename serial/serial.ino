/*
    serial.ino
    Arduino Serial Node
    Recommended Arduino Mega or Giga
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros.h>
#include <ros/time.h>
#include <pidtuner/VelocityCommand.h>
#include <pidtuner/VelocityFeedback.h>
#include <pidtuner/PositionCommand.h>
#include <pidtuner/PositionFeedback.h>
#include <pidtuner/StepCommand.h>
#include <pidtuner/Configuration.h>
#include <pidtuner/EmergencyStop.h>
#include <QuadratureEncoder.h>
#include "performer.h"
#include "encoder.h"
#include "pwm.h"
#include "pid.h"

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char VELOCITY_COMMAND[] = "velocity";
const char VELOCITY_FEEDBACK[] = "velocity_feedback";
const char POSITION_COMMAND[] = "position";
const char POSITION_FEEDBACK[] = "position_feedback";
const char STEP_COMMAND[] = "step";
const char CONFIGURATION_COMMAND[] = "configuration";
const char ESTOP_COMMAND[] = "estop";

const int RATE = 50;
const float TIMESTEP = 1.0 / float(RATE);
const int STARTUP_DELAY = 3000;

const int ADC_PINS[] =
{
  A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15
};

enum Mode
{
  VELOCITY,
  POSITION,
  STEP
};

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

void velocityCommand(const pidtuner::VelocityCommand& msg);
void velocityFeedback();
void positionCommand(const pidtuner::PositionCommand& msg);
void positionFeedback();
void stepCommand(const pidtuner::StepCommand& msg);
void stepFeedback();
void configurationCommand(const pidtuner::Configuration& msg);
void emergencyStop(const pidtuner::EmergencyStop& msg);
void stop();

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

//
// Settings
//

// PWM pins
// https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
int lpwmPin = 11;
int rpwmPin = 3;

// ADC pins
// https://www.arduino.cc/reference/en/language/functions/analog-io/analogread/
int adcPin = 0;

// SPI pins
// https://www.arduino.cc/reference/en/language/functions/communication/spi/
int csPin = 53;

// Quadrature pins
// https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
int aPin = 18;
int bPin = 19;

// Min normalized PWM command
float pwmMin = 0.0;

// Max normalized PWM command
float pwmMax = 1.0;

// Whether to invert PWM commands
bool pwmInvert;

// Min normalized absolute encoder reading
float absoluteMin = 0.0;

// Max normalized absolute encoder reading
float absoluteMax = 1.0;

// Whether to invert absolute encoder readings
bool absoluteInvert;

// Whether to invert quadrature encoder readings
bool quadratureInvert;

//
// State
//

// Absolute encoder reading
float absolute;

// Absolute encoder
Encoder* absoluteEncoder;

// Quadrature encoder reading
int32_t quadrature;

// Quadrature encoder
Encoders* quadratureEncoder;

// PID controller
PID pid;

// PWM command
float command;
uint8_t lpwm;
uint8_t rpwm;

// Control mode
Mode mode = VELOCITY;

// PID goal
float goal;

// PID tolerance
float tolerance;

// Steps
performer steps;

// Start time
ros::Time start;

// Current time
ros::Time time;

// Elapsed time
float elapsed;

// Time step
float dt;

// Emergency stop
bool estop;

// Velocity feedback publisher
pidtuner::VelocityFeedback velocityFeedbackMsg;
ros::Publisher velocityPub(
  VELOCITY_FEEDBACK, &velocityFeedbackMsg);

// Position feedback publisher
pidtuner::PositionFeedback positionFeedbackMsg;
ros::Publisher positionPub(
  POSITION_FEEDBACK, &positionFeedbackMsg);

// Velocity command subscriber
ros::Subscriber<pidtuner::VelocityCommand> velocitySub(
  VELOCITY_COMMAND, velocityCommand);

// Position command subscriber
ros::Subscriber<pidtuner::PositionCommand> positionSub(
  POSITION_COMMAND, positionCommand);

// Step command subscriber
ros::Subscriber<pidtuner::StepCommand> stepSub(
  STEP_COMMAND, stepCommand);

// Configuration command subscriber
ros::Subscriber<pidtuner::Configuration> configSub(
  CONFIGURATION_COMMAND, configurationCommand);

// Emergency stop command subscriber
ros::Subscriber<pidtuner::EmergencyStop> stopSub(
  ESTOP_COMMAND, emergencyStop);

// ROS node
ros::NodeHandle node;

/*----------------------------------------------------------*\
| Initialization
\*----------------------------------------------------------*/

void initAbsolute()
{
  delete absoluteEncoder;

  if (csPin)
  {
    absoluteEncoder = new AS5045Encoder(csPin);
  }
  else if (adcPin)
  {
    absoluteEncoder = new ADCEncoder(ADC_PINS[adcPin]);
  }
  else
  {
    absoluteEncoder = nullptr;
  }
}

void initQuadrature()
{
  delete quadratureEncoder;

  if (aPin && bPin)
    quadratureEncoder = new Encoders(aPin, bPin);
}

void initPwm()
{
  pinMode(lpwmPin, OUTPUT);
  pinMode(rpwmPin, OUTPUT);
}

void setup()
{
  initAbsolute();
  initQuadrature();
  initPwm();

  node.initNode();

  node.advertise(velocityPub);
  node.advertise(positionPub);

  node.subscribe(velocitySub);
  node.subscribe(positionSub);
  node.subscribe(stepSub);
  node.subscribe(configSub);
  node.subscribe(stopSub);

  node.negotiateTopics();
}

ros::Time getTime()
{
  uint32_t ms = millis();
  uint32_t sec = ms / 1000;
  uint32_t ns = (ms - sec * 1000) * 1000000;

  return ros::Time(sec, ns);
}

void loop()
{
  live();
  love();
  laugh();
}

void live()
{
  ros::Time now = getTime();
  dt = (now - time).toSec();
  elapsed = (now - start).toSec();

  if (dt >= TIMESTEP) time = now;
}

void love()
{
  if (dt < TIMESTEP) return;

  read();

  velocityFeedback();
  positionFeedback();
  stepFeedback();
}

void laugh()
{
  if (dt < TIMESTEP) return;

  write();

  node.spinOnce();
}

void read()
{
  if (absoluteEncoder)
  {
    absolute = clamp(absoluteEncoder->read(), absoluteMin, absoluteMax);

    if (absoluteInvert)
    {
      absolute = 1.0 - absolute;
    }
  }

  if (quadratureEncoder)
  {
    quadrature = quadratureEncoder->getEncoderCount();

    if (quadratureInvert)
    {
      quadrature = -quadrature;
    }
  }
}

void write()
{
  analogWrite(lpwmPin, lpwm);
  analogWrite(rpwmPin, rpwm);
}

void velocityCommand(const pidtuner::VelocityCommand& msg)
{
  mode = VELOCITY;
  start = getTime();
  command = msg.command;
  estop = false;

  commandToPwm(command, pwmMin, pwmMax, pwmInvert, lpwm, rpwm);
}

void positionCommand(const pidtuner::PositionCommand& msg)
{
  mode = POSITION;
  start = getTime();
  estop = false;

  goal = msg.goal;
  tolerance = msg.tolerance;

  pid.reset();
}

void velocityFeedback()
{
  // Publishing too soon causes "tried to publish before configured" error
  if (millis() < STARTUP_DELAY) return;

  pidtuner::VelocityFeedback msg;

  msg.command = command;
  msg.absolute = absolute;
  msg.quadrature = quadrature;
  msg.time = time;
  msg.start = start;
  msg.dt = dt;
  msg.elapsed = elapsed;
  msg.step = steps.step;
  msg.estop = estop;
  msg.mode = mode;
  msg.volts = 0.0;
  msg.amps = 0.0;

  velocityPub.publish(&msg);
}

void positionFeedback()
{
  if (mode != POSITION) return;

  float error = goal - absolute;

  if (abs(error) < tolerance)
  {
    stop();
  }
  else
  {
    command = pid.getCommand(error, dt);
    commandToPwm(command, pwmMin, pwmMax, pwmInvert, lpwm, rpwm);
  }

  pidtuner::PositionFeedback msg;

  msg.position = absolute;
  msg.goal = goal;
  msg.tolerance = tolerance;
  msg.pe = pid.pe;
  msg.ie = pid.ie;
  msg.de = pid.de;
  msg.p = pid.p;
  msg.i = pid.i;
  msg.d = pid.d;

  positionPub.publish(&msg);
}

void stepCommand(const pidtuner::StepCommand& msg)
{
  mode = STEP;
  start = getTime();
  estop = false;
  steps.play(start, msg);
}

void stepFeedback()
{
  if (mode != STEP || estop) return;

  command = steps.getCommand(time);

  if (steps.done)
  {
    stop();
  }
  else
  {
    commandToPwm(command, pwmMin, pwmMax, pwmInvert, lpwm, rpwm);
  }
}

void emergencyStop(const pidtuner::EmergencyStop& msg)
{
  estop = msg.stop;
  if (estop) stop();
}

void stop()
{
  mode = VELOCITY;
  command = 0;
  lpwm = 0;
  rpwm = 0;
}

void configurationCommand(const pidtuner::Configuration& msg)
{
  // Configure PWM
  if (lpwmPin != msg.LPWMpin || rpwmPin != msg.RPWMpin)
  {
    lpwmPin = msg.LPWMpin;
    rpwmPin = msg.RPWMpin;
    initPwm();
  }

  // Configure ADC
  if (msg.ADCpin != adcPin || msg.csPin != csPin)
  {
    adcPin = msg.ADCpin;
    csPin = msg.csPin;
    initAbsolute();
  }

  // Configure Quadrature
  if (msg.Apin != aPin || msg.Bpin != bPin)
  {
    aPin = msg.Apin;
    bPin = msg.Bpin;
    initQuadrature();
  }

  // Configure PID
  if (
    pid.Kp != msg.Kp ||
    pid.Ki != msg.Ki ||
    pid.Kd != msg.Kd ||
    pid.iMin != msg.iMin ||
    pid.iMax != msg.iMax)
  {
    pid.reset();
    pid.configure(
      msg.Kp,
      msg.Ki,
      msg.Kd,
      msg.iMin,
      msg.iMax);
  }

  pwmMin = msg.pwmMin;
  pwmMax = msg.pwmMax;
  pwmInvert = msg.pwmInvert;

  quadratureInvert = msg.quadratureInvert;

  absoluteMin = msg.absoluteMin;
  absoluteMax = msg.absoluteMax;
  absoluteInvert = msg.absoluteInvert;
}
