# Omnidirectional Manipulator Robot using ROS2

Omnidirectional Manipulator Robot using ROS2 and MicroROS.

![Omnidirectional Manipulator](render.png)

## Table of Contents
- [Omnidirectional Manipulator Robot using ROS2](#omnidirectional-manipulator-robot-using-ros2)
  - [Table of Contents](#table-of-contents)
  - [Current Simulation Results](#current-simulation-results)
  - [Hardware Requirements](#hardware-requirements)
  - [Software Requirements](#software-requirements)
  - [Installation](#installation)
  - [Controller Setup](#controller-setup)
  - [Project Setup](#project-setup)

## Current Simulation Results
[![Demonstration](thumbnail.png)](https://youtube.com/playlist?list=PL3C8yCOARHROVQ0J0LYWpv6aPSOsDYQEf&feature=shared)

[Check the project on youtube](https://youtube.com/playlist?list=PL3C8yCOARHROVQ0J0LYWpv6aPSOsDYQEf&feature=shared) 

## Hardware Requirements
- ESP32.
- JGB37-520 motors.
- H bridge L293D.
- Xbox controller.
- 18650 batteries.
- Raspberry Pi 5 (4gb RAM).
- Arduino Mega 2560.
- MG995 servomotors.
- Intel Realsense D415.
- Lidar Sensor LD19 (LDROBOT).

## Software Requirements
In order for the project to work we need to have installed [Ubuntu 24.04 LTS Noble Numbat](https://releases.ubuntu.com/noble/) alocated
in a partition with a Windows dual boot or as the main Operating System of your machine. 

You can try to virtualize it but in my experience, networking and driver problems arouse.

## Installation
We need to install [ROS Jazzy](https://wiki.ros.org/jazzy), you can follow the [documentation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) on how to install it or you can follow and run these few commands inside Ubuntu's terminal:

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

```bash
sudo apt update && sudo apt install ros-dev-tools
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop
```

## Controller Setup
You can use any controller listed in the [documentation](http://wiki.ros.org/joy). I'm personally
using an 8bitdo Ultimate C 2.4GHz. It is not a Xbox Controller per se but I managed to find some drivers to 
trick Linux into thinking it is a generic Xbox Controller ([you can find the post here](https://gist.github.com/ammuench/0dcf14faf4e3b000020992612a2711e2)):

```bash
touch /etc/udev/rules.d/99-8bitdo-xinput.rules
sudo nano /etc/udev/rules.d/99-8bitdo-xinput.rules
```
Inside 99-8bitdo-xinput.rules you type the following and save:
```bash
ACTION=="add", ATTRS{idVendor}=="2dc8", ATTRS{idProduct}=="3106", RUN+="/sbin/modprobe xpad", RUN+="/bin/sh -c 'echo 2dc8 3106 > /sys/bus/usb/drivers/xpad/new_id'"
```

Reload udevadm service
```bash
sudo udevadm control --reload
```

You also need to make your [joystick device accesible](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick):

```bash
ls -l /dev/input/jsX
```

You will see something similar to:
```bash
crw-rw-XX- 1 root dialout 188, 0 2009-08-14 12:04 /dev/input/jsX
```

We need to change XX to rw:
```bash
sudo chmod a+rw /dev/input/jsX
```

## Project Setup

Clone this repository:

```bash
git clone https://github.com/Xtalism/ROS2-Omni-Manipulator.git
```

We need to source, install dependencies and compile our workspace just as follows:

Sourcing ROS2:
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
```

Sourcing the project:
```bash
cd ros2_ws/
source install/setup.bash
```
Installing dependencies and compiling the project:
```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

To run any package or launch file"

```bash
ros2 run <package_name> <program_name>
ros2 launch <package_name> <launch_file>
```
