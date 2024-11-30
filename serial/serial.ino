/*
    serial.ino
    Arduino ROS Serial Node
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
#include <Adafruit_INA260.h>
#include "performer.h"
#include "encoder.h"
#include "pwm.h"
#include "pid.h"

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

// Velocity command topic (see VelocityCommand.msg)
const char VELOCITY_COMMAND[] = "velocity";

// Velocity feedback topic (see VelocityFeedback.msg)
const char VELOCITY_FEEDBACK[] = "velocity_feedback";

// Position command topic (see PositionCommand.msg)
const char POSITION_COMMAND[] = "position";

// Position feedback topic (see PositionFeedback.msg)
const char POSITION_FEEDBACK[] = "position_feedback";

// Step command topic (see StepCommand.msg)
const char STEP_COMMAND[] = "step";

// Configuration topic (see Configuration.msg)
const char CONFIGURATION_COMMAND[] = "configuration";

// Emergency stop command topic (see EmergencyStop.msg)
const char ESTOP_COMMAND[] = "estop";

// Update rate
const int RATE = 100;
const float TIMESTEP = 1.0 / float(RATE);

// Startup delay (prevents published too soon errors)
const int STARTUP_DELAY = 3000;

// Multiplier to convert mA to A
const float MA_TO_A = 0.001;

// Multiplier to convert mV to V
const float MV_TO_V = 0.001;

// Map 0-based ADC pin index to hardware-specific pin Id
const int ADC_PINS[] =
{
  A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15
};

// Control mode
enum Mode
{
  VELOCITY, // Default, accepting velocity commands
  POSITION, // Running a PID algorithm to reach a position with tolerance
  STEP      // Playing back a sequence of steps
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

// I2C pins
// Used to read voltage and current w/Adafruit INA260
// 20 SDA
// 21 SCL

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
bool quadratureInvert = true;

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

// Voltage/current measurement
Adafruit_INA260 ina260 = Adafruit_INA260();
bool enableVoltageCurrent;
float volts = 0.0;
float amps = 0.0;

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

// PID goal position tolerance
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

void initVoltageCurrentMeasurement()
{
  enableVoltageCurrent = ina260.begin();
}

void setup()
{
  initAbsolute();
  initQuadrature();
  initPwm();
  initVoltageCurrentMeasurement();

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
  // Track time
  ros::Time now = getTime();
  dt = (now - time).toSec();
  elapsed = (now - start).toSec();

  if (dt >= TIMESTEP) time = now;
}

void love()
{
  // Read sensors
  if (dt < TIMESTEP) return;

  read();

  velocityFeedback();
  positionFeedback();
  stepFeedback();
}

void laugh()
{
  // Write motor
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

  if (enableVoltageCurrent)
  {
    amps = ina260.readCurrent() * MA_TO_A;
    volts = ina260.readBusVoltage() * MV_TO_V;
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
  msg.volts = volts;
  msg.amps = amps;

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
    command = clamp(pid.getCommand(error, dt), -1.0, 1.0);
    commandToPwm(command, pwmMin, pwmMax, pwmInvert, lpwm, rpwm);
  }

  pidtuner::PositionFeedback msg;

  msg.position = absolute;
  msg.goal = goal;
  msg.tolerance = tolerance;
  msg.pe = pid.pe;
  msg.ie = pid.ie;
  msg.de = pid.de;

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
