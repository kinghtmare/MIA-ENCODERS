# MIA-ENCODERS

ROS-integrated encoder feedback system for differential-drive robots — STM32 & ESP32 implementations.

---

## What This Does

Without encoder feedback, motor commands are open-loop: the robot has no way to know if a wheel stalled, slipped, or drifted. This system closes that loop.

The microcontroller reads quadrature encoder pulses via hardware interrupts, computes wheel ticks and velocity, then either publishes raw serial output (non-ROS) or pushes data to ROS topics for odometry.

### 🧠 System Mind Map

![ESP-WROOM-32 Flow Diagram](https://github.com/kinghtmare/MIA-ENCODERS/blob/142c8bf8ae9ab0f368e5b923a493bc513ceb3007/images/ESP-WROOM-32/ESP-WROOM-32%20Flow.png)

---

## Repository Structure

```
MCU/
├── non-ros/
│   ├── ESP-WROOM-32/        # Standalone encoder test — ESP32
│   └── STM32/               # Standalone encoder test — STM32
└── ros/
    └── ESP-WROOM-32/
        ├── v1(±ve)/         # Bidirectional signed output
        └── v2(+ve)/         # Positive-only output with X-axis direction flag

TECH-DIVE.md                 # ESP32 Firmware Architecture — V1 vs V2 deep dive
```

---

## Hardware Setup

### Wiring

Connect each encoder's A and B channels to interrupt-capable GPIO pins on the MCU. The ISR monitors Channel A; Channel B determines direction.

| Encoder Channel | Logic                          |
|-----------------|-------------------------------|
| A ≠ B           | Forward → `counter++`         |
| A = B           | Reverse → `counter--`         |

### Supported Hardware

| MCU           | Non-ROS | ROS (rosserial) |
|---------------|---------|-----------------|
| STM32         | ✅      | ❌              |
| ESP-WROOM-32  | ✅      | ✅              |

---

## Getting Started

### Prerequisites

**Ubuntu 20.04 — ROS Noetic**

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full ros-noetic-rosserial ros-noetic-rosserial-arduino
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Arduino IDE — STM32 support**
for linux/windows Follow the [STM32 Arduino IDE setup guide](https://github.com/Abdalla-El-gohary/Programming-Stm32-Using-Arduino-IDE).

**Windows — CP210x USB Driver**
Download from [Silicon Labs](https://www.silabs.com/documents/public/software/CP210x_VCP_Windows.zip), run `CP210xVCPInstaller_x64.exe`, then verify the COM port appears under Device Manager → Ports.

---

### Quickstart (Non-ROS)

Use this first to verify your encoder wiring before introducing ROS.

1. Open the appropriate sketch from `MCU/non-ros/`
2. Flash to your MCU
3. Open Serial Monitor — you should see position, distance, and velocity updating in real time

### Quickstart (ROS)

1. Flash [ESP-V2](https://github.com/kinghtmare/MIA-ENCODERS/tree/142c8bf8ae9ab0f368e5b923a493bc513ceb3007/MCU/non-ros/ESP-WROOM-32/(%2Bve)V2.cpp) to your ESP32
2. On your PC:
```bash
# Terminal 1 — always first
roscore

# Terminal 2 — rosserial bridge (plug in ESP32 first)
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200

# Terminal 3 — your listener
python3 encoder_listener.py
```

3. Verify data:
   ```bash
   rostopic echo /encoder_data
   ```

---

## How It Works

### Interrupt-Driven Counting

An ISR fires on every rising edge of encoder Channel A. It reads Channel B to determine direction and increments or decrements a shared counter.

### Velocity Calculation

Every 500 ms, the main loop snapshots the counter (with interrupts briefly disabled to prevent torn reads), computes the delta, and derives velocity:

$$v = \frac{\Delta\text{count} \times D_{\text{count}}}{\Delta t}$$

Where:

| Symbol | Value |
|--------|-------|
| Wheel diameter | 10.0 cm |
| Counts per revolution | 360 |
| $D_{\text{count}}$ (cm/count) | $\frac{\pi \times 10.0}{360} \approx 0.0873$ cm |

### Distance Calculation

$$s = \text{total count} \times D_{\text{count}}$$

---

## Firmware Versions

| Version | Output | Direction Tracking | Stability |
|---------|--------|--------------------|-----------|
| V1 (±ve) | Signed count, distance, velocity | Via sign | Experimental |
| V2 (+ve) | Unsigned distance, velocity + `X_axis` flag | Via `X_axis = ±1` | Stable |

**V1** — Raw bidirectional output. Negative values on reverse. Useful for debugging and direct odometry integration.

**V2** — Resets counter on direction change; exposes `X_axis` (`+1` / `-1`) as a separate variable. Cleaner for visualization and downstream processing.

→ **[ESP32 Firmware Architecture](./MCU/non-ros/ESP-WROOM-32/TECH-DIVE.md)** — Detailed deep dive into V1 vs V2 design, ISR synchronization, and race condition fixes.

> **ISR synchronization note:** Early versions contained a race condition where the ISR could reset `counter` while the main loop was reading it, producing phantom zero-velocity spikes. Both current versions resolve this through atomic snapshots and main-loop-only resets.

---

## Sample Output

**Forward motion (V1):**
```
Position:  28 counts | Distance:  2.44 cm | Velocity:  4.89 cm/s
Position:  58 counts | Distance:  5.06 cm | Velocity:  5.24 cm/s
Position:  89 counts | Distance:  7.77 cm | Velocity:  5.41 cm/s
```

**Reverse motion (V1):**
```
Position:  62 counts | Distance:  5.41 cm | Velocity:  4.71 cm/s
Position:  34 counts | Distance:  2.97 cm | Velocity:  4.89 cm/s
Position:  -7 counts | Distance: -0.61 cm | Velocity:  4.71 cm/s
```

**Forward motion (V2 with X-axis):**
```
X-Axis: +1 | Position:  28 counts | Distance:  2.44 cm | Velocity:  4.89 cm/s
X-Axis: +1 | Position:  58 counts | Distance:  5.06 cm | Velocity:  5.24 cm/s
```

**No motion:**
```
X-Axis: -1 | Position:  62 counts | Distance:  5.41 cm | Velocity:  0.00 cm/s
```

> Velocity is always a positive magnitude. Direction is tracked separately via sign (V1) or `X_axis` flag (V2).

---

## Roadmap

- [ ] STM32 ROS publisher
- [ ] PID closed-loop wheel control
- [ ] Hardware timer for sub-millisecond timing accuracy
- [ ] Velocity low-pass filtering
- [ ] Differential-drive odometry node
- [ ] IMU fusion (Kalman filter)

---

## License

See [LICENSE](LICENSE).
