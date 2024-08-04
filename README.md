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
