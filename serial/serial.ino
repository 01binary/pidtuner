/*
    serial.ino
    Arduino Serial Node
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#define USE_USBCON

#include <ros.h>                        // ROS communication
#include <angles/angles.h>
#include <control_toolbox/pid.h>
#include <pidtuner/VelocityCommand.h>
#include <pidtuner/VelocityFeedback.h>
#include <pidtuner/PositionCommand.h>
#include <pidtuner/PositionFeedback.h>
#include <pidtuner/StepCommand.h>
#include <pidtuner/Configuration.h>
#include <QuadratureEncoder.h>          // QuadratureEncoder library

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char VELOCITY_COMMAND_TOPIC[] = "velocity";
const char VELOCITY_FEEDBACK_TOPIC[] = "velocity_feedback";
const char POSITION_COMMAND_TOPIC[] = "position";
const char POSITION_FEEDBACK_TOPIC[] = "position_feedback";
const char STEP_COMMAND_TOPIC[] = "step";
const char CONFIGURATION_COMMAND_TOPIC[] = "configuration";

const double RATE_HZ = 50.0;
const int DELAY = 1000.0 / RATE_HZ;
const int STARTUP_DELAY = 3000;

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

void velocityCommand(const pidtuner::VelocityCommand& msg);
void velocityFeedback();
void positionCommand(const pidtuner::PositionCommand& msg);
void positionFeedback();
void stepCommand(const pidtuner::StepCommand& msg);
void configurationCommand(const pidtuner::Configuration& msg);

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
uint16_t absolute = 0;

// Relative encoder reading
int32_t quadrature = 0;

// Relative encoder
Encoders* pEncoder;

// PWM command
int32_t command = 0;

// Output start time
ros::Time start;

// PID error
int32_t error = 0;

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
  absolute = (uint16_t)analogRead(adcPin);
  quadrature = pEncoder->getEncoderCount();

  node.spinOnce();

  delay(DELAY);
}

void velocityCommand(const pidtuner::VelocityCommand& msg)
{
  // TODO: process command
}

void velocityFeedback()
{
  pidtuner::VelocityFeedback msg;

  msg.command = command;
  msg.absolute = absolute;
  msg.quadrature = quadrature;
  msg.start = start;

  velocityPub.publish(&msg);
}

void positionCommand(const pidtuner::PositionCommand& msg)
{
  // TODO: process command
}

void stepCommand(const pidtuner::StepCommand& msg)
{
  // TODO: process command
}

void configurationCommand(const pidtuner::Configuration& msg)
{
  // TODO: process command
}
