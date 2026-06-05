# ESP32 Encoder Firmware — V1 vs V2

Architectural evolution from general-purpose to production-grade embedded firmware.

---

## TL;DR

| | V1 | V2 |
|---|---|---|
| **Counter** | Bidirectional (up/down) | Always-positive odometer |
| **Direction** | Implicit (counter sign) | Explicit `isrDirection` multiplier (`+1`/`-1`) |
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

   A↑  B != A → CW (+1)         A↑  B == A → CCW (-1)
   A↓  B != A → CW (+1)         A↓  B == A → CCW (-1)
```

Interrupt fires on **both edges of Channel A** (CHANGE) → 2× resolution vs. single-edge.
Direction is determined by sampling Channel B at the moment of each transition.

---

## ISR Design

### V1
```cpp
void IRAM_ATTR encoderISR() {
  bool a = digitalRead(outputA);
  bool b = digitalRead(outputB);

  if (a != aLastState) {
    if (b != a) {
      counter++;    // CW
    } else {
      counter--;    // CCW
    }
  }
  aLastState = a;
}
```
- Branches on direction → non-deterministic execution time
- Direct increment/decrement of shared `counter` → race condition window on every pulse
- `aLastState` guard prevents double-counting on spurious transitions

### V2
```cpp
volatile int32_t isrCount      = 0;   // always increments, never decrements
volatile int8_t  isrDirection  = 1;   // +1 or -1
volatile bool    isrDirChanged = false;
volatile int     aLastState    = 0;
volatile int     lastDirection = 0;   // 0 = unknown at boot

void IRAM_ATTR encoderISR() {
  bool a = digitalRead(outputA);
  bool b = digitalRead(outputB);

  if (a != (bool)aLastState) {
    int8_t currentDirection = (b != a) ? 1 : -1;

    if (lastDirection != 0 && currentDirection != lastDirection) {
      isrDirChanged = true;   // tell main loop to reset — don't reset here
    }

    isrDirection  = currentDirection;
    lastDirection = currentDirection;
    isrCount++;               // always positive
  }
  aLastState = (int)a;
}
```
- `IRAM_ATTR` → executes from internal RAM, not flash
- `aLastState` guard → same edge-deduplication as V1, prevents spurious counts
- `lastDirection != 0` boot-guard → ignores the very first direction assignment at startup
- Single-cycle flag set → near-zero race window
- No branching on the counter → constant execution time

---

## Counting Mechanism

### V1 — Bidirectional Counter

Direction is embedded in the counter's sign. Intuitive, but forces the ISR to decrement a shared variable asynchronously.

```cpp
// V1: direction inferred from sign
noInterrupts();
int currentCount = counter;   // could be -350 or +220
interrupts();

int deltaCount        = currentCount - lastCounter;
float distanceTraveled = deltaCount * distancePerCount;
float velocity         = distanceTraveled / deltaTime;
// Caller must check sign to know direction — logic leaks into application layer
```

### V2 — Always-Positive Odometer

`isrCount` only ever increases. Direction is an explicit `int8_t` multiplier (`+1`/`-1`).

```cpp
// V2: magnitude and direction are separate
int32_t relativeCount = rawCount - resetAnchor;
int32_t deltaCount    = rawCount - lastCount;

float deltaDist   = deltaCount * DISTANCE_PER_COUNT;
totalDistance    += deltaDist;
float velocity    = deltaDist / dt;   // multiply by direction (isrDirection) at output

// signed velocity is ready to use — no sign parsing needed
```

Aligns natively with ROS `geometry_msgs/Twist` and standard Cartesian conventions.

---

## The `resetAnchor` System

Provides a zeroed relative distance without touching the master counter.

```cpp
// On direction change (main loop, inside atomic lock):
if (dirChanged) {
    resetAnchor   = rawCount;   // Anchor to current absolute position
    totalDistance = 0.0f;
    lastCount     = rawCount;
}

// Every cycle:
int32_t relativeCount = rawCount - resetAnchor;
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
int currentCount = counter;   // Only the counter is protected
interrupts();
// No other ISR state is read — but counter type is plain int, not int32_t
```

### V2 — Full Atomic Snapshot
```cpp
// Freeze entire sensor state in one lock window
noInterrupts();
int32_t rawCount   = isrCount;
int8_t  direction  = isrDirection;
bool    dirChanged = isrDirChanged;
if (dirChanged) isrDirChanged = false;   // ← Clear INSIDE the lock, not after
interrupts();

// Main loop works entirely from snapshot — ISR runs freely with no contention
```

Clearing the flag *inside* the lock is non-negotiable. Clearing it after re-enabling interrupts creates a gap where a direction change fires, gets missed, and `resetAnchor` is never updated — producing a permanently offset `relativeCount`.

---

## Main Loop — 500 ms Cycle

```cpp
unsigned long now = millis();
if (now - lastPrintTime >= 500) {
    float dt = (now - lastPrintTime) / 1000.0f;

    // 1. Atomic snapshot
    noInterrupts();
    int32_t rawCount   = isrCount;
    int8_t  direction  = isrDirection;
    bool    dirChanged = isrDirChanged;
    if (dirChanged) isrDirChanged = false;
    interrupts();

    // 2. Direction change → re-anchor
    if (dirChanged) {
        resetAnchor   = rawCount;
        totalDistance = 0.0f;
        lastCount     = rawCount;
        Serial.println(">> Direction changed — counter reset to 0");
    }

    // 3. Derive position and motion
    int32_t relativeCount = rawCount - resetAnchor;
    int32_t deltaCount    = rawCount - lastCount;
    if (deltaCount < 0) deltaCount = 0;

    float deltaDist   = deltaCount * DISTANCE_PER_COUNT;
    totalDistance    += deltaDist;
    float velocity    = deltaDist / dt;

    Serial.print("X-Axis: ");
    Serial.print(direction == 1 ? "+1" : "-1");
    Serial.print(" | Count: ");
    Serial.print(relativeCount);
    Serial.print(" | Distance: ");
    Serial.print(totalDistance, 2);
    Serial.print(" cm | Velocity: ");
    Serial.print(velocity, 2);
    Serial.println(" cm/s");

    lastCount     = rawCount;
    lastPrintTime = now;
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

1. **The ISR never corrupts shared state** — it only increments `isrCount`, updates `isrDirection`, and sets `isrDirChanged`. No resets happen inside the ISR.
2. **The main loop never reads partial data** — it always operates on a frozen, consistent snapshot captured inside a single `noInterrupts()` window.

Those two properties are the difference between firmware that works on a bench and firmware that ships.
