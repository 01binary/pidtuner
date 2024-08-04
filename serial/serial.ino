/*
    serial.ino
    Arduino Serial Node
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#define USE_USBCON

#include <ros.h>
#include <pidtuner/VelocityCommand.h>
#include <pidtuner/VelocityFeedback.h>
#include <pidtuner/PositionCommand.h>
#include <pidtuner/PositionFeedback.h>
#include <pidtuner/StepCommand.h>
#include <pidtuner/Configuration.h>
#include <QuadratureEncoder.h>
#include "pid.h"

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char VELOCITY_COMMAND_TOPIC[] = "velocity";
const char VELOCITY_FEEDBACK_TOPIC[] = "velocity_feedback";
const char POSITION_COMMAND_TOPIC[] = "position";
const char POSITION_FEEDBACK_TOPIC[] = "position_feedback";
const char STEP_COMMAND_TOPIC[] = "step";
const char CONFIGURATION_COMMAND_TOPIC[] = "configuration";

const int RATE = 50;

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
void stop(const pidtuner::EmergencyStop& msg);

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

// PWM pins
int lpwmPin = 3;
int rpwmPin = 11;

// ADC pins
int adcPin = A0;

// Quadrature pins
int aPin = 2;
int bPin = 7;

// Absolute encoder reading
uint16_t absolute;

// Quadrature encoder reading
int32_t quadrature;

// Quadrature encoder
Encoders* pEncoder;

// PID controller
PID pid;

// PWM command
double command;
uint8_t lpwm;
uint8_t rpwm;

// Position or velocity control mode
Mode mode;

// PID goal
uint16_t goal;

// PID tolerance
uint16_t tolerance;

// Steps
std::vector<pidtuner::Step> steps;

// Update rate
ros::Rate rate(RATE);

// Start time
ros::Time start = ros::Time::now();

// Update time
ros::Time time = ros::Time::now();

// Time step
ros::Duration timeStep;

// Emergency stop
bool stop;

// Velocity feedback publisher
pidtuner::VelocityFeedback velocityFeedbackMsg;
ros::Publisher velocityPub(
  VELOCITY_FEEDBACK_TOPIC, &velocityFeedbackMsg);

// Position feedback publisher
pidtuner::PositionFeedback positionFeedbackMsg;
ros::Publisher positionPub(
  POSITION_FEEDBACK_TOPIC, &positionFeedbackMsg);

// Velocity command subscriber
ros::Subscriber<pidtuner::VelocityCommand> velocitySub(
  VELOCITY_COMMAND_TOPIC, velocityCommand);

// Position command subscriber
ros::Subscriber<pidtuner::PositionCommand> positionSub(
  POSITION_COMMAND_TOPIC, positionCommand);

// Step command subscriber
ros::Subscriber<pidtuner::StepCommand> stepSub(
  STEP_COMMAND_TOPIC, stepCommand);

// Configuration command subscriber
ros::Subscriber<pidtuner::Configuration> configSub(
  CONFIGURATION_COMMAND_TOPIC, configurationCommand);

// ROS node
ros::NodeHandle node;

/*----------------------------------------------------------*\
| Initialization
\*----------------------------------------------------------*/

void initAdc()
{
  pinMode(adcPin, INPUT_PULLUP);
}

void initQuadrature()
{
  pEncoder = new Encoders(aPin, bPin);
}

void initPwm()
{
  pinMode(lpwmPin, OUTPUT);
  pinMode(rpwmPin, OUTPUT);
}

void setup()
{
  initAdc();
  initQuadrature();
  initPwm();

  node.initNode();

  node.advertise(velocityPub);
  node.advertise(positionPub);

  node.subscribe(velocitySub);
  node.subscribe(positionSub);
  node.subscribe(stepSub);
  node.subscribe(configSub);

  node.negotiateTopics();

  delay(STARTUP_DELAY);
}

void loop()
{
  ros::Time now = ros::Time::now();
  timeStep = now - time;
  time = now;

  read();

  velocityFeedback();
  positionFeedback();
  stepFeedback();

  write();

  node.spinOnce();
  rate.sleep();
}

void read()
{
  absolute = (uint16_t)analogRead(adcPin);
  quadrature = pEncoder->getEncoderCount();
}

void write()
{
  analogWrite(lpwmPin, lpwm);
  analogWrite(rpwmPin, rpwm);
}

void velocityCommand(const pidtuner::VelocityCommand& msg)
{
  mode = VELOCITY;
  start = ros::Time::now();
  lpwm = msg.LPWM;
  rpwm = msg.RPWM;
  command = msg.LPWM - msg.RPWM;
  stop = false;
}

void positionCommand(const pidtuner::PositionCommand& msg)
{
  mode = POSITION;
  start = row::Time::now();
  stop = false;

  goal = msg.goal;
  tolerance = msg.tolerance;

  pid.reset();
}

void velocityFeedback()
{
  pidtuner::VelocityFeedback msg;

  msg.header.time = time;
  msg.command = command;
  msg.LPWM = lpwm;
  msg.RPWM = rpwm;
  msg.absolute = absolute;
  msg.quadrature = quadrature;
  msg.start = start;

  velocityPub.publish(&msg);
}

void positionFeedback()
{
  if (mode != POSITION || stop) return;

  double error = double(goal - absolute);

  if (uint16_t(abs(error)) > tolerance)
  {
    command = pid.getCommand(error, timeStep);
    
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
  else
  {
    command = 0;
    lpwm = 0;
    rpwm = 0;
  }

  pidtuner::PositionFeedback msg;

  msg.header.time = time;
  msg.pe = pid.pe;
  msg.ie = pid.ie;
  msg.de = pid.de;
  msg.p = pid.p;
  msg.i = pid.i;
  msg.d = pid.d;
}

void stepCommand(const pidtuner::StepCommand& msg)
{
  mode = STEP;
  start = ros::Time::now();
  stop = false;

  if (!msg.steps.empty())
  {
    steps = msg.steps;
  }
}

void stepFeedback()
{
  if (mode != STEP || stop) return;

  ros::Duration elapsed = time - start;
  double elapsedSec = double(elapsed) / 1000.0;
  
  for (int step = 0; step < steps.size(); step++)
  {
    if (steps[step].time >= elapsedSec)
    {
      lpwm = steps[step].LPWM;
      rpwm = steps[step].RPWM;
      command = lpwm - rpwm;
      break;
    }
  }
}

void stop(const pidtuner::EmergencyStop& msg)
{
  stop = msg.stop;

  if (stop)
  {
    mode = VELOCITY;
    command = 0;
    lpwm = 0;
    rpwm = 0;
    pid.reset();
  }
}

void configurationCommand(const pidtuner::Configuration& msg)
{
  // Configure PWM
  if (lpwmPin != msg.LPWMPin || rpwmPin != msg.RPWMpin)
  {
    lpwmPin = msg.LPWMpin;
    rpwmPin = msg.RPWMpin;
    initPwm();
  }

  // Configure ADC
  if (msg.ADCpin != adcPin)
  {
    adcPin = msg.ADCPin;
    initAdc();
  }

  // Configure Quadrature
  if (msg.Apin != aPin || msg.Bpin != bPin)
  {
    aPin = msg.Apin;
    bPin = msg.Bpin;
    initQuadrature();
  }

  // Configure PID
  pid.reset();
  pid.configure(
    msg.Kp,
    msg.Ki,
    msg.Di,
    msg.iMin,
    msg.iMax);
}
