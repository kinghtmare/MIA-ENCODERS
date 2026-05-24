# MIA-ENCODERS
ROS commination between encoders ,STM and ROS


## 📋 Problem Statement

### The Problem
We need to give the STM32 the current reading of the motors (via encoders) so it can adjust the command to the motor drivers accordingly.

### Why This Matters
In a robotic system, simply sending commands to motors without feedback leads to:
- Inaccurate speed control
- No way to detect if a motor is stalled
- Poor odometry and localization
- Inability to maintain straight-line motion (one wheel may spin faster than the other)

### The Solution

### 🧠 System Mind Map

![STM32 Flow Diagram](images/Stm32%20Flow.png)
The STM32 reads encoder pulses (A and B channels) to compute:
1. **Current wheel ticks** - How much each wheel has rotated
2. **Actual motor speed (RPM)** - How fast each motor is currently spinning

Using this feedback, the STM32:
1. Compares the desired speed (received from ROS via `/cmd_vel`) with the actual speed (from encoders)
2. Adjusts the PWM signals sent to motor drivers to minimize the error
3. Publishes the actual encoder data back to ROS for odometry and monitoring


## set up

### 1. Set up your STM32
Follow the [Programming Guide](https://github.com/Abdalla-El-gohary/Programming-Stm32-Using-Arduino-IDE) to configure the Arduino IDE for STM32 on linux.

### 2. Test the encoder example
Use the code from the [DeepSeek chat](https://chat.deepseek.com/share/d3j6axdn28htyv53vb) to:
- Read encoder pins (A and B channels)
- Compute wheel ticks and speed
- Publish ROS messages over serial

### 3. Connect to ROS
- Run `roscore`
- Start `rosserial` (or `micro-ros-agent`) on your PC
- Upload the STM32 code and verify encoder data appears in ROS topics

 **Note**: the non ros file  is a test code

## 📋 Prerequisites

### 1. Install ROS Noetic (Ubuntu 20.04)

Open a terminal and run the following commands:

```bash
sudo apt install ros-noetic-rosserial
sudo apt install ros-noetic-rosserial-arduino
sudo apt install ros-noetic-rosserial-python 
