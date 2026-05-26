# MIA-ENCODERS
ROS communication between encoders, STM32 and ROS

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

## 🛠️ Setup

### For Linux (Ubuntu 20.04)

#### 1. Set up your STM32
Follow the [Programming Guide](https://github.com/Abdalla-El-gohary/Programming-Stm32-Using-Arduino-IDE) to configure the Arduino IDE for STM32 on Linux.

#### 2. Test the encoder example
Use the code from [non-ROS esp.cpp](https://github.com/kinghtmare/MIA-ENCODERS/blob/main/non-ROS%20esp.cpp) to:
- Read encoder pins (A and B channels)
- Compute wheel ticks and speed
- Publish ROS messages over serial

#### 3. Connect to ROS
- Run `roscore`
- Start `rosserial` (or `micro-ros-agent`) on your PC
- Upload the STM32 code and verify encoder data appears in ROS topics

**Note**: The non-ROS file is a test code

### For Windows

#### 1. Set up your STM32
Follow the [Programming Guide](https://github.com/Abdalla-El-gohary/Programming-Stm32-Using-Arduino-IDE) to configure the Arduino IDE for STM32 on Windows.

#### 2. Install CP210x USB-to-Serial Drivers
The STM32 board typically uses a CP210x chip for USB communication. Windows needs drivers to recognize it:

1. **Download the drivers:**
   Download from Silicon Labs official site:
   [CP210x_VCP_Windows.zip](https://www.silabs.com/documents/public/software/CP210x_VCP_Windows.zip)

2. **Install the drivers:**
   - Extract the downloaded ZIP file
   - Run the installer appropriate for your system (usually `CP210xVCPInstaller_x64.exe` for 64-bit Windows)

#### 3. Verify COM Port Detection

1. **Open Device Manager:**
   - Right-click on Start Menu → Device Manager
   - Or press `Win + X` and select Device Manager

2. **Check COM port assignment:**
   - Plug in your STM32 board via USB
   - In Device Manager, look for **"Ports (COM & LPT)"** section
   - You should see something like `"Silicon Labs CP210x USB to UART Bridge (COM10)"`
   - If you see an **"Other devices"** section with an unknown device that appears/disappears when you plug/unplug the board, the driver is not installed correctly

3. **Confirm the COM port:**
   - The port number (e.g., COM10) is what you'll use in Arduino IDE and ROS serial communication
   - In this example setup, we're using **COM10**

#### 4. Configure Arduino IDE for Windows
- Select the correct COM port (COM10 in this case) in Arduino IDE under Tools → Port
- Select your STM32 board under Tools → Board

#### 5. Test the encoder example
Use the code from [non-ROS stm.cpp](https://github.com/kinghtmare/MIA-ENCODERS/blob/main/non-ROS%20stm.cpp) to:
- Read encoder pins (A and B channels)
- Compute wheel ticks and speed


**Note**: The non-ROS file is a test code

## 📋 Prerequisites

### 1. Install ROS Noetic (Ubuntu 20.04)

Open a terminal and run the following commands:

```bash
# Add ROS repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add ROS keys
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Update package list
sudo apt update

# Install ROS Noetic Desktop Full
sudo apt install ros-noetic-desktop-full

# Install rosserial for communication with STM32
sudo apt install ros-noetic-rosserial
sudo apt install ros-noetic-rosserial-arduino

# Setup ROS environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
# 📊 Technical Report

## 1. How the Code Works

The system relies on hardware interrupts to trace the orientation and velocity of the vehicle's wheels. An Interrupt Service Routine (ISR) monitors **Channel A** of the quadrature encoder. The moment [...]

- If Channel A and Channel B are in **different logical states**, the wheel is spinning **forward**, triggering a tick increment (`counter++`).
- If their logical states **match**, the wheel is spinning in **reverse**, triggering a tick decrement (`counter--`).

A background loop executes asynchronously every **500ms** to snapshot this raw counter data, compute elapsed derivatives safely while interrupts are briefly paused, and parse the data into real-world [...]

---

## 2. Mathematical Formulations

To translate raw digital steps into usable physical metrics for localization and odometry, the firmware executes calculations across three foundational areas: **geometry**, **scaling**, and **differen[...]

---

### A. Wheel Circumference

The physical distance covered by a single full 360° rotation of the actuator is equivalent to the wheel's circumference:

$$\text{Circumference} = \pi \times d$$

| Variable | Value |
|----------|-------|
| Diameter ($d$) | $10.0 \text{ cm}$ |
| Constant ($\pi$) | $\approx 3.14159$ |

$$\text{Circumference} = 3.14159 \times 10.0 = 31.4159 \text{ cm}$$

---

### B. Scaling Factor (Distance Per Count)

To map an isolated tick back to a distance value, the total circumference is evenly segmented by the hardware resolution capability of the encoder ($360.0 \text{ Counts Per Revolution}$):

$$D_{\text{count}} = \frac{\text{Circumference}}{\text{Counts Per Revolution}}$$

$$D_{\text{count}} = \frac{31.4159 \text{ cm}}{360.0} \approx 0.087266 \text{ cm/count}$$

---

### C. Absolute Distance Calculation

Total absolute displacement ($s$) achieved by the wheel since system initialization is a linear mapping of cumulative ticks:

$$s = \text{Current Count} \times D_{\text{count}}$$

---

### D. Velocity Estimation

Velocity ($v$) is calculated dynamically inside the timed loop structure as the change in distance over a discrete time window ($\Delta t \approx 0.5 \text{ seconds}$):

$$v = \frac{\Delta s}{\Delta t}$$

**Where:**

**1. Time Interval ($\Delta t$)** — Evaluated via the processor system clock to dynamically accommodate minor loop jitter:

$$\Delta t = \frac{\text{currentTime} - \text{lastTime}}{1000.0}$$

**2. Counter Delta ($\Delta \text{count}$)** — The net change in ticks during the window:

$$\Delta \text{count} = \text{currentCount} - \text{lastCounter}$$

**3. Displacement Delta ($\Delta s$)** — The physical distance cleared during the window:

$$\Delta s = \Delta \text{count} \times D_{\text{count}}$$



## 3. Sample Serial Output

### ▶ Forward Motion

## 3. Sample Serial Output

### ▶ Forward Motion

```text
Position:  28 counts | Distance:  2.44 cm | Velocity:  4.89 cm/s
Position:  58 counts | Distance:  5.06 cm | Velocity:  5.24 cm/s
Position:  89 counts | Distance:  7.77 cm | Velocity:  5.41 cm/s
```

### ◀ Reverse Motion
```text
Position:  62 counts | Distance:  5.41 cm | Velocity:  4.71 cm/s
Position:  34 counts | Distance:  2.97 cm | Velocity:  4.89 cm/s
Position:  -7 counts | Distance: -0.61 cm | Velocity:  4.71 cm/s
```
### ⏸ No Motion
```text
ESP32 is running... (waiting for encoder input)
Position:  -7 counts | Distance: -0.61 cm | Velocity:  0.00 cm/s
```

> **Direction Key:**
> - Count/Distance **increasing** → Forward
> - Count/Distance **decreasing or negative** → Reverse
> - Velocity is **always positive** — speed magnitude only

