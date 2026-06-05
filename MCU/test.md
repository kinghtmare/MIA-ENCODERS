# ESP32 Quadrature Encoder — Odometry System

High-performance wheel position, distance, and velocity tracking for mobile robotics and precision motion control.

---

## Hardware Configuration

| GPIO | Function | I/O | Description |
|------|----------|-----|-------------|
| 23 | Channel A | Input | Primary phase signal; interrupt trigger |
| 19 | Channel B | Input | Secondary phase signal; direction sense |
| 2 | Onboard LED | Output | Heartbeat / boot status |

**Pin mode:** `INPUT_PULLUP` — activates internal pull-up resistors to prevent floating pins, suppress jitter, and eliminate parasitic counts.

---

## Parameters & Constants

| Parameter | Value |
|-----------|-------|
| Encoder Resolution | 360 CPR |
| Wheel Diameter | 10 cm |

**Pre-calculated at init (minimizes runtime overhead):**

```
Circumference       C    = π × diameter
Distance per Count  Dpc  = C / COUNTS_PER_REVOLUTION
```

---

## Memory & Compiler Keywords

| Keyword | Purpose |
|---------|---------|
| `volatile` | Prevents compiler register-caching of ISR-modified variables; forces live memory reads |
| `IRAM_ATTR` | Places ISR in fast internal RAM — bypasses flash latency, prevents missed pulses at high RPM |

---

## ISR Logic — Quadrature Decoding

Interrupt triggered on **Channel A `CHANGE`** (both rising and falling edges) → **2× resolution** vs. single-edge triggering.

**Direction logic** (sampled immediately after Channel A transition):

| Condition | Direction |
|-----------|-----------|
| Signal B ≠ Signal A | Clockwise (increment) |
| Signal B = Signal A | Counter-Clockwise (decrement) |

---

## Initialization Sequence

1. Configure GPIO 23 & 19 as `INPUT_PULLUP`; GPIO 2 as `OUTPUT`
2. Blink LED — confirms successful boot and hardware readiness
3. Attach `encoderISR` to Channel A with `CHANGE` trigger
4. Pre-calculate `distancePerCount`

---

## Main Loop

Non-blocking architecture via `millis()` — no `delay()` stalls.

### Atomic Reads

32-bit reads on ESP32 span multiple clock cycles. An ISR firing mid-read causes a **race condition** → data corruption. Fix: wrap reads in `noInterrupts()` / `interrupts()` to snapshot the register safely.

### Data Processing — 500 ms Interval

```
deltaCount  = currentSnapshot - previousCount
totalDist   = totalCount × Dpc
velocity    = (deltaCount × Dpc) / Δt
```

### System Monitoring

| Task | Interval | Action |
|------|----------|--------|
| Heartbeat | 1000 ms | Toggle LED |
| Stationary check | 5 s timeout | Serial warning if no pulses detected |

---

## Applications

- Wheel odometry and SLAM
- Closed-loop motor speed control
- Conveyor belt distance/sync measurement
- CNC axis position tracking
- Digital measuring wheels and UI dials

---

## Potential Optimizations

| Enhancement | Benefit |
|-------------|---------|
| **PCNT peripheral** | Offload pulse counting to hardware; supports higher pulse frequencies |
| **Direction flags** | Explicit CW/CCW serial output |
| **Acceleration** | Second derivative of position → monitor torque and load |
| **Signal filtering** | Moving average or Kalman filter → smooth velocity spikes from vibration |
