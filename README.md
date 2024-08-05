# pidtuner

PID Tuning ROS App

## Install Packages

```
sudo apt-get install ros-${ROS_DISTRO}-rosserial-arduino
sudo apt-get install ros-${ROS_DISTRO}-rosserial

cd ~/catkin_ws/src
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
```

## Build

Build the workspace:

```
cd ~/catkin_ws
catkin_make
```

Build the Arduino serial node:

```
rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries
```

> The Arduino libraries are usually in `~/Arduino/libraries`. If you installed Arduino IDE as a *snap*, you could also try looking in `~/snap/arduino`.

Build [serial.ino](./src/serial.ino) ROS Serial node and upload to Arduino.

## Run

This will start the Arduino Serial node:

```
roslaunch pidtuner pidtuner.launch
```

## Velocity

```
rostopic pub /velocity pidtuner/VelocityCommand "{ LPWM: 0, RPWM: 0 }" -1
```

## Position

```
rostopic pub /position pidtuner/PositionCommand "{ goal: 0.0, tolerance: 0.0 }" -1
```

## Step

```
rostopic pub /step pidtuner/StepCommand "{ steps: [{ LPWM: 255, RPWM: 0, time: 1 }] }" -1
```

## Emergency Stop

```
rostopic pub /stop pidtuner/EmergencyStop "{ stop: true }" -1
```