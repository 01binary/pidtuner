/*
    serial.ino
    Arduino Serial Node
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#define USE_USBCON

#include <ros.h>                // ROS communication
#include <pidtuner/Adc.h>       // Analog read request
#include <pidtuner/Pwm.h>       // Analog write request
#include <QuadratureEncoder.h>  // QuadratureEncoder library

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

// ROS topics and spin rate
const char VELOCITY_COMMAND_TOPIC[] = "velocity";
const char VELOCITY_FEEDBACK_TOPIC[] = "velocity_feedback";
const char POSITION_COMMAND_TOPIC[] = "position";
const char STEP_COMMAND_TOPIC[] = "step";
const char CONFIGURATION_COMMAND_TOPIC[] = "configuration";
const double RATE_HZ = 50.0;
const int DELAY = 1000.0 / RATE_HZ;
const int STARTUP_DELAY = 3000;

// PWM pins
const int LPWM = 3;
const int RPWM = 11;

// ADC pins
const int ADC = A0;

// Quadrature pins
const int A = 2;
const int B = 7;

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

void velocityCommand(const pidtuner::VelocityCommand& msg);
void velocityFeedback();
void positionCommand(const pidtuner::PositionCommand& msg);
void stepCommand(const pidtuner::Steps& msg);
void configurationCommand(const pidtuner::Configuration& msg);

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

// Absolute encoder reading
int16_t adc = 0;

// Relative encoder reading
int32_t quadrature = 0;

// Relative encoder
Encoders encoder(A, B);

// PWM command
int32_t pwm = 0;

// Command start time
ros::Time start;

// PID error
int32_t error = 0;

// Velocity feedback publisher
ros::Publisher velocityPub(
  VELOCITY_FEEDBACK_TOPIC, &velocityFeedback);

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
  pinMode(ADC, INPUT_PULLUP);
}

void initPwm()
{
  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);
}

void setup()
{
  initAdc();
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
  adc = (int16_t)analogRead(ADC);
  quadrature = encoder.getEncoderCount();

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

  msg.PWM = pwm;
  msg.ADC = adc;
  msg.quadrature = quadrature;
  msg.start = start;

  pub.publish(&msg);
}

void positionCommand(const pidtuner::PositionCommand& msg)
{
  // TODO: process command
}

void stepCommand(const pidtuner::Steps& msg)
{
  // TODO: process command
}

void configurationCommand(const pidtuner::Configuration& msg)
{
  // TODO: process command
}

