// =============================================
// encoder_publisher_v2.ino
// ESP32 Encoder → rosserial → ROS 1 (Noetic / Ubuntu 20)
//
// Publishes:
//   /encoder/count     (std_msgs/Int32)   — always positive, resets on direction change
//   /encoder/distance  (std_msgs/Float32) — always positive cm, resets on direction change
//   /encoder/velocity  (std_msgs/Float32) — always positive cm/s
//   /encoder/xaxis     (std_msgs/Int32)   — +1 Forward | -1 Reverse
// =============================================

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

// =============================================
// PIN DEFINITIONS
// =============================================
#define outputA     23
#define outputB     19
#define LED_BUILTIN  2

// =============================================
// ENCODER PARAMETERS
// =============================================
const float COUNTS_PER_REVOLUTION = 360.0;
const float WHEEL_DIAMETER_CM     = 10.0;
const float WHEEL_CIRCUMFERENCE   = 3.14159265f * WHEEL_DIAMETER_CM;
const float DISTANCE_PER_COUNT    = WHEEL_CIRCUMFERENCE / COUNTS_PER_REVOLUTION;

// =============================================
// ISR STATE
// These are ONLY written inside the ISR.
// Main loop reads them atomically via noInterrupts().
// NEVER reset counter inside the ISR — race condition.
// Reset is handled safely in the main loop.
// =============================================
volatile int32_t isrCount        = 0;   // always increments, never decrements
volatile int8_t  isrDirection    = 1;   // +1 or -1
volatile bool    isrDirChanged   = false; // flag: direction flip detected
volatile int     aLastState      = 0;
volatile int     lastDirection   = 0;   // 0 = unknown at boot

// =============================================
// MAIN LOOP STATE
// =============================================
int32_t lastCount       = 0;
int32_t publishedCount  = 0;   // count value after last reset
float   totalDistance   = 0.0f;
float   velocity        = 0.0f;

unsigned long lastPublishTime = 0;
const unsigned long PUBLISH_INTERVAL_MS = 100;  // 10 Hz

// =============================================
// ROS
// =============================================
ros::NodeHandle nh;

std_msgs::Int32   count_msg;
std_msgs::Float32 dist_msg;
std_msgs::Float32 vel_msg;
std_msgs::Int32   xaxis_msg;

ros::Publisher pub_count   ("/encoder/count",    &count_msg);
ros::Publisher pub_distance("/encoder/distance", &dist_msg);
ros::Publisher pub_velocity("/encoder/velocity", &vel_msg);
ros::Publisher pub_xaxis   ("/encoder/xaxis",    &xaxis_msg);

// =============================================
// ISR — keep it minimal, no resets here
// =============================================
void IRAM_ATTR encoderISR() {
  bool a = digitalRead(outputA);
  bool b = digitalRead(outputB);

  if (a != (bool)aLastState) {
    int8_t currentDirection = (b != a) ? 1 : -1;

    if (lastDirection != 0 && currentDirection != lastDirection) {
      isrDirChanged = true;   // signal main loop to reset
    }

    isrDirection  = currentDirection;
    lastDirection = currentDirection;
    isrCount++;               // always positive, always increments
  }
  aLastState = (int)a;
}

// =============================================
// SETUP
// =============================================
void setup() {
  pinMode(outputA,     INPUT_PULLUP);
  pinMode(outputB,     INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  // Boot blink
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(150);
    digitalWrite(LED_BUILTIN, LOW);  delay(150);
  }

  // rosserial owns Serial — do NOT call Serial.begin()
  nh.initNode();
  nh.advertise(pub_count);
  nh.advertise(pub_distance);
  nh.advertise(pub_velocity);
  nh.advertise(pub_xaxis);

  aLastState = digitalRead(outputA);
  attachInterrupt(digitalPinToInterrupt(outputA), encoderISR, CHANGE);

  lastPublishTime = millis();

  // Ready blink
  digitalWrite(LED_BUILTIN, HIGH); delay(400);
  digitalWrite(LED_BUILTIN, LOW);
}

// =============================================
// MAIN LOOP
// =============================================
void loop() {
  unsigned long now = millis();

  if (now - lastPublishTime >= PUBLISH_INTERVAL_MS) {
    float dt = (now - lastPublishTime) / 1000.0f;

    // --- Atomic read of all ISR state ---
    noInterrupts();
    int32_t rawCount     = isrCount;
    int8_t  direction    = isrDirection;
    bool    dirChanged   = isrDirChanged;
    if (dirChanged) isrDirChanged = false;  // clear flag inside critical section
    interrupts();

    // --- Handle direction change: reset counters here, not in ISR ---
    if (dirChanged) {
      publishedCount = rawCount;   // anchor point: counts from here are "fresh"
      totalDistance  = 0.0f;
      lastCount      = rawCount;
    }

    // Count relative to last reset point
    int32_t relativeCount = rawCount - publishedCount;
    int32_t deltaCount    = rawCount - lastCount;
    if (deltaCount < 0) deltaCount = 0;  // safety: shouldn't happen now

    float deltaDist   = deltaCount * DISTANCE_PER_COUNT;
    totalDistance    += deltaDist;
    velocity          = deltaDist / dt;

    // --- Publish ---
    count_msg.data = relativeCount;
    pub_count.publish(&count_msg);

    dist_msg.data = totalDistance;
    pub_distance.publish(&dist_msg);

    vel_msg.data = velocity;
    pub_velocity.publish(&vel_msg);

    xaxis_msg.data = (int32_t)direction;
    pub_xaxis.publish(&xaxis_msg);

    lastCount       = rawCount;
    lastPublishTime = now;
  }

  nh.spinOnce();

  // Heartbeat LED at 1 Hz
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  if (now - lastBlink >= 1000) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastBlink = now;
  }
}
