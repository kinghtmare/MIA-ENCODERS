# MIA-ENCODERS
ROS commination between encoders ,STM and ROS

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
