# ESP32 Encoder Firmware — V1 vs V2

Architectural evolution from general-purpose to production-grade embedded firmware.

---

## TL;DR

| | V1 | V2 |
|---|---|---|
| **Counter** | Bidirectional (up/down) | Always-positive odometer |
| **Direction** | Implicit (counter sign) | Explicit `xAxis` multiplier (`+1`/`-1`) |
| **Direction change** | Counter crosses zero | Mathematical `resetAnchor` |
| **ISR complexity** | Heavy (decodes & modifies state) | Ultra-light (flag + increment only) |
| **Data safety** | Medium (basic locks) | High (atomic snapshots) |
| **Use case** | DIY / hobbyist | Robotics / production |

---

## Core Philosophy

**V1 — Monolithic ISR**
The ISR handles quadrature decoding, direction logic, *and* direct counter modification. Any multi-byte read from the main loop during an ISR write risks silent data corruption.

**V2 — ISR as Sensor, Main Loop as Processor**
The ISR touches nothing except a master counter and a single boolean flag. All logic lives in the main loop, operating on a frozen snapshot of sensor state.

---

## Quadrature Signal — What the Hardware Sees

```
         CW Rotation                   CCW Rotation
         ___     ___                   ___     ___
Ch A  __|   |___|   |___           ___|   |___|   |__
           ___     ___             ___     ___
Ch B  ____|   |___|   |___     ___|   |___|   |______

      A↑  B=0 → CW                A↑  B=1 → CCW
      A↓  B=1 → CW                A↓  B=0 → CCW
```

Interrupt fires on **both edges of Channel A** (CHANGE) → 2× resolution vs. single-edge.
Direction is determined by sampling Channel B at the moment of each transition.

---

## ISR Design

### V1
```cpp
void IRAM_ATTR encoderISR() {
    bool a = digitalRead(ENC_A);
    bool b = digitalRead(ENC_B);

    if (a == b) {
        encoderCount--;   // ⚠️ Direct decrement of shared 32-bit variable
    } else {
        encoderCount++;   // ⚠️ Race condition if main loop reads mid-write
    }
}
```
- Branching logic → non-deterministic execution time
- Direct state modification → race condition window on every pulse
- No `IRAM_ATTR` guarantee in minimal setups → may execute from flash

### V2
```cpp
volatile int32_t  isrCount      = 0;
volatile bool     isrDirChanged = false;
volatile bool     lastDirection = true;   // true = CW

void IRAM_ATTR encoderISR() {
    bool a = digitalRead(ENC_A);
    bool b = digitalRead(ENC_B);
    bool currentDir = (a != b);           // CW = true, CCW = false

    if (currentDir != lastDirection) {
        isrDirChanged = true;             // Single-cycle flag set
        lastDirection = currentDir;
    }

    isrCount++;                           // Always increments — never decrements
}
```
- `IRAM_ATTR` → executes from internal RAM, not flash
- Single-cycle flag set → near-zero race window
- No branching on the counter → constant execution time

---

## Counting Mechanism

### V1 — Bidirectional Counter

Direction is embedded in the counter's sign. Intuitive, but forces the ISR to decrement a shared variable asynchronously.

```cpp
// V1: direction inferred from sign
int32_t counts    = encoderCount;        // Could be -350 or +220
float   distance  = counts * distPerCount;
float   velocity  = delta * distPerCount / dt;
// Caller must check sign to know direction — logic leaks into application layer
```

### V2 — Always-Positive Odometer

`isrCount` only ever increases. Direction is an explicit multiplier.

```cpp
// V2: magnitude and direction are separate
int8_t  xAxis    = lastDirection ? +1 : -1;
float   distance = relativeCount * distPerCount;
float   velocity = (deltaCount * distPerCount / dt) * xAxis;
// signed_velocity is ready to use — no sign parsing needed
```

Aligns natively with ROS `geometry_msgs/Twist` and standard Cartesian conventions.

---

## The `resetAnchor` System

Provides a zeroed relative distance without touching the master counter.

```cpp
// On direction change (main loop, inside atomic lock):
if (snap_dirChanged) {
    resetAnchor = snap_isrCount;         // Anchor to current absolute position
}

// Every cycle:
int32_t relativeCount = snap_isrCount - resetAnchor;
```

**Direction change walkthrough:**

| Step | `isrCount` | `resetAnchor` | `relativeCount` |
|------|-----------|---------------|-----------------|
| Forward 100 counts | 100 | 0 | 100 |
| Direction flip | 100 | → 100 | 0 |
| Reverse 50 counts | 150 | 100 | 50 |

The master counter is never reset — no glitches, no missed pulses during transition.

---

## Data Integrity — Atomic Snapshot

**The problem:** A 32-bit read on ESP32 can be split across two CPU cycles by a compiler optimization or interrupt. If the ISR fires between those cycles, the main loop reads half-old / half-new data — silent corruption with no crash, no warning.

### V1 — Partial Lock
```cpp
noInterrupts();
int32_t count = encoderCount;   // Only the counter is protected
interrupts();
// isrDirChanged and lastDirection are read separately — unprotected, can drift
```

### V2 — Full Atomic Snapshot
```cpp
// Freeze entire sensor state in one lock window
noInterrupts();
int32_t snap_count      = isrCount;
bool    snap_dirChanged = isrDirChanged;
bool    snap_lastDir    = lastDirection;
isrDirChanged           = false;        // ← Clear INSIDE the lock, not after
interrupts();

// Main loop works entirely from snapshot — ISR runs freely with no contention
```

Clearing the flag *inside* the lock is non-negotiable. Clearing it after re-enabling interrupts creates a gap where a direction change fires, gets missed, and `resetAnchor` is never updated — producing a permanently offset `relativeCount`.

---

## Main Loop — 500 ms Cycle

```cpp
unsigned long now = millis();
if (now - lastCalcTime >= 500) {

    // 1. Atomic snapshot
    noInterrupts();
    int32_t snap_count      = isrCount;
    bool    snap_dirChanged = isrDirChanged;
    bool    snap_lastDir    = lastDirection;
    isrDirChanged           = false;
    interrupts();

    // 2. Direction change → re-anchor
    if (snap_dirChanged) {
        resetAnchor = snap_count;
        xAxis       = snap_lastDir ? +1 : -1;
    }

    // 3. Derive position and motion
    int32_t relativeCount = snap_count - resetAnchor;
    int32_t deltaCount    = snap_count - previousCount;
    float   dt            = (now - lastCalcTime) / 1000.0f;

    float totalDistance   = relativeCount * distancePerCount;
    float velocity        = (deltaCount   * distancePerCount / dt) * xAxis;

    previousCount  = snap_count;
    lastCalcTime   = now;
}
```

---

## When to Use Which

**V1** — occasional glitch is acceptable:
- Rotary UI knobs and menu dials
- Hobbyist distance measurement
- Non-autonomous motor feedback

**V2** — data integrity is a requirement:
- Autonomous robots and SLAM pipelines
- PID controllers — a corrupted read triggers an over-correction spike that can damage hardware
- State machines that trigger on per-leg distance (direction reset = clean leg zero)
- Any system where a firmware bug is a safety event, not a UX annoyance

---

## Two Guarantees V2 Makes

1. **The ISR never corrupts shared state** — it only increments and sets a flag.
2. **The main loop never reads partial data** — it always operates on a frozen, consistent snapshot.

Those two properties are the difference between firmware that works on a bench and firmware that ships.
