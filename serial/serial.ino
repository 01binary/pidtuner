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
const double TIMESTEP = 1.0 / double(RATE);

const int ADC_PINS[] =
{
  A0,
  A1,
  A2,
  A3,
  A4,
  A5,
  A6,
  A7,
  A8,
  A9,
  A10,
  A11,
  A12,
  A13,
  A14,
  A15
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

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

// PWM pins
int lpwmPin = 3;
int rpwmPin = 11;

// ADC pins
int adcPin = A0;

// SPI pins
int csPin = 0;

// Quadrature pins
int aPin = 2;
int bPin = 7;

// Absolute encoder reading
double absolute;

// Absolute encoder
Encoder* absoluteEncoder;

// Quadrature encoder reading
int32_t quadrature;

// Quadrature encoder
Encoders* quadratureEncoder;

// PID controller
PID pid;

// PWM command
double command;
uint8_t lpwm;
uint8_t rpwm;

// Position or velocity control mode
Mode mode = VELOCITY;

// PID goal
double goal;

// PID tolerance
double tolerance;

// Steps
pidtuner::Step* steps;
int stepCount;
bool loopSteps;

// Start time
ros::Time start;

// Update time
ros::Time time;

// Time step
ros::Duration timeStep;

// Emergency stop
bool stop;

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
  ros::Time now = getTime();
  timeStep = now - time;

  if (timeStep.toSec() < TIMESTEP) return;
 
  time = now;

  read();

  velocityFeedback();
  positionFeedback();
  stepFeedback();

  write();

  node.spinOnce();
}

void read()
{
  if (absoluteEncoder)
    absolute = absoluteEncoder->read();

  if (quadratureEncoder)
    quadrature = quadratureEncoder->getEncoderCount();
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
  stop = false;

  commandToPwm(command, lpwm, rpwm);
}

void positionCommand(const pidtuner::PositionCommand& msg)
{
  mode = POSITION;
  start = getTime();
  stop = false;

  goal = msg.goal;
  tolerance = msg.tolerance;

  pid.reset();
}

void velocityFeedback()
{
  pidtuner::VelocityFeedback msg;

  msg.command = command;
  msg.LPWM = lpwm;
  msg.RPWM = rpwm;
  msg.absolute = absolute;
  msg.quadrature = quadrature;
  msg.time = time;
  msg.start = start;

  velocityPub.publish(&msg);
}

void positionFeedback()
{
  if (mode != POSITION) return;

  double error = goal - absolute;

  if (stop || abs(error) < tolerance)
  {
    command = 0;
    lpwm = 0;
    rpwm = 0;
  }
  else
  {
    command = pid.getCommand(error, timeStep.toSec());
    commandToPwm(command, lpwm, rpwm);
  }

  pidtuner::PositionFeedback msg;

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
  stop = false;

  if (msg.steps_length)
  {
    pidtuner::Step* oldSteps = steps;
    pidtuner::Step* newSteps = new pidtuner::Step[msg.steps_length];

    memcpy(newSteps, msg.steps, msg.steps_length * sizeof(pidtuner::Step));

    steps = newSteps;
    delete[] oldSteps;

    stepCount = msg.steps_length;
    loopSteps = msg.loop;
  }
}

void stepFeedback()
{
  if (mode != STEP || !stepCount || stop) return;

  double total = steps[stepCount - 1].time + 1.0;
  double elapsed = (time - start).toSec();

  if (elapsed > total && !loopSteps)
  {
    mode = VELOCITY;
    command = 0.0;
    lpwm = 0;
    rpwm = 0;
    return;
  }

  elapsed = fmod(elapsed, total);
  
  for (int step = 0; step < stepCount; step++)
  {
    if (steps[step].time >= elapsed)
    {
      command = steps[step].command;
      commandToPwm(command, lpwm, rpwm);
      break;
    }
  }
}

void emergencyStop(const pidtuner::EmergencyStop& msg)
{
  stop = msg.stop;

  if (stop)
  {
    command = 0;
    lpwm = 0;
    rpwm = 0;
    pid.reset();
  }
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
}
