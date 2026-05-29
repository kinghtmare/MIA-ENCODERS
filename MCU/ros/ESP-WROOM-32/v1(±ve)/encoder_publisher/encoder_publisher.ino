// =============================================
// encoder_publisher.ino
// ESP32 Encoder → rosserial → ROS 1 (Noetic / Ubuntu 20)
//
// Dependencies (Arduino Library Manager):
//   - ros_lib  (generated from rosserial_arduino)
//   - Standard ESP32 Arduino core
//
// ROS side setup (once):
//   sudo apt install ros-noetic-rosserial-arduino ros-noetic-rosserial-python
//   rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
// =============================================

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>   // for velocity with timestamp
#include <nav_msgs/Odometry.h>            // optional: full odometry message

// =============================================
// PIN DEFINITIONS
// =============================================
#define outputA     23
#define outputB     19
#define LED_BUILTIN  2

// =============================================
// ENCODER PARAMETERS — tune these
// =============================================
const float COUNTS_PER_REVOLUTION = 360.0;
const float WHEEL_DIAMETER_CM     = 10.0;
const float WHEEL_CIRCUMFERENCE   = 3.14159265f * WHEEL_DIAMETER_CM;
const float DISTANCE_PER_COUNT    = WHEEL_CIRCUMFERENCE / COUNTS_PER_REVOLUTION;

// =============================================
// GLOBAL STATE
// =============================================
volatile int32_t encoderCount = 0;   // raw tick counter (volatile = ISR-safe)
int32_t lastCount              = 0;
int     aLastState             = 0;

unsigned long lastPublishTime  = 0;
const unsigned long PUBLISH_INTERVAL_MS = 100;   // 10 Hz — change if needed

float totalDistance = 0.0f;   // cumulative cm
float velocity      = 0.0f;   // cm/s

// =============================================
// ROS NODE + PUBLISHERS
// =============================================
ros::NodeHandle nh;

std_msgs::Int32   count_msg;
std_msgs::Float32 dist_msg;
std_msgs::Float32 vel_msg;

// Topic names — subscribe to these from the listener node
ros::Publisher pub_count   ("/encoder/count",    &count_msg);
ros::Publisher pub_distance("/encoder/distance", &dist_msg);
ros::Publisher pub_velocity("/encoder/velocity", &vel_msg);

// =============================================
// INTERRUPT SERVICE ROUTINE
// IRAM_ATTR keeps it in fast IRAM on ESP32
// =============================================
void IRAM_ATTR encoderISR() {
  bool a = digitalRead(outputA);
  bool b = digitalRead(outputB);

  if (a != (bool)aLastState) {
    // Standard quadrature decode: direction from relative phase
    encoderCount += (b != a) ? 1 : -1;
  }
  aLastState = (int)a;
}

// =============================================
// SETUP
// =============================================
void setup() {
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  // Boot blink — visual confirmation before ROS connects
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(150);
    digitalWrite(LED_BUILTIN, LOW);  delay(150);
  }

  // Init ROS node over Serial (rosserial)
  // NOTE: do NOT call Serial.begin() yourself — nh.initNode() handles it
  nh.initNode();
  nh.advertise(pub_count);
  nh.advertise(pub_distance);
  nh.advertise(pub_velocity);

  // Attach quadrature interrupt on channel A
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

    // Atomic read of volatile counter
    noInterrupts();
    int32_t currentCount = encoderCount;
    interrupts();

    int32_t deltaCount  = currentCount - lastCount;
    float   deltaDist   = deltaCount * DISTANCE_PER_COUNT;
    totalDistance      += deltaDist;
    velocity            = deltaDist / dt;   // cm/s; negative = reverse

    // Publish count
    count_msg.data = currentCount;
    pub_count.publish(&count_msg);

    // Publish cumulative distance (cm)
    dist_msg.data = totalDistance;
    pub_distance.publish(&dist_msg);

    // Publish instantaneous velocity (cm/s)
    vel_msg.data = velocity;
    pub_velocity.publish(&vel_msg);

    lastCount       = currentCount;
    lastPublishTime = now;
  }

  // Let rosserial process incoming/outgoing serial traffic
  nh.spinOnce();

  // Heartbeat LED: blink at 1 Hz
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  if (now - lastBlink >= 1000) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastBlink = now;
  }
}
