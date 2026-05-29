// =============================================
// encoder_v2_fixed.ino
// Always-positive counter + xAxis direction
// Race condition fixed: resets happen in main loop, NOT in ISR
// No ROS — plain Serial output
// =============================================

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
const float WHEEL_DIAMETER        = 10.0;   // cm
const float WHEEL_CIRCUMFERENCE   = 3.14159265f * WHEEL_DIAMETER;
const float DISTANCE_PER_COUNT    = WHEEL_CIRCUMFERENCE / COUNTS_PER_REVOLUTION;

// =============================================
// ISR STATE — only written inside ISR
// Main loop reads atomically via noInterrupts()
// =============================================
volatile int32_t isrCount      = 0;   // always increments, never decrements
volatile int8_t  isrDirection  = 1;   // +1 or -1
volatile bool    isrDirChanged = false;
volatile int     aLastState    = 0;
volatile int     lastDirection = 0;   // 0 = unknown at boot

// =============================================
// MAIN LOOP STATE
// =============================================
int32_t lastCount      = 0;
int32_t resetAnchor    = 0;   // isrCount value at last direction change
float   totalDistance  = 0.0f;
float   velocity       = 0.0f;

unsigned long lastPrintTime = 0;
unsigned long lastBlinkTime = 0;
bool ledState = false;

// =============================================
// ISR — flag only, zero resets here
// =============================================
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

// =============================================
// SETUP
// =============================================
void setup() {
  pinMode(outputA,     INPUT_PULLUP);
  pinMode(outputB,     INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  // Boot blink
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(200);
    digitalWrite(LED_BUILTIN, LOW);  delay(200);
  }

  Serial.begin(115200);
  delay(1000);

  Serial.println("=================================");
  Serial.println("    ESP32 ENCODER READER v2      ");
  Serial.println("=================================");
  Serial.println("Channel A -> GPIO23");
  Serial.println("Channel B -> GPIO19");
  Serial.println("X-Axis: +1 = Forward | -1 = Reverse");
  Serial.println("Counter resets to 0 on direction change");
  Serial.print("Distance per count: ");
  Serial.print(DISTANCE_PER_COUNT, 4);
  Serial.println(" cm");
  Serial.println("=================================");
  Serial.println("Ready! Turn the encoder...");
  Serial.println();

  aLastState = digitalRead(outputA);
  attachInterrupt(digitalPinToInterrupt(outputA), encoderISR, CHANGE);

  lastPrintTime = millis();
  lastBlinkTime = millis();

  // Ready blink
  digitalWrite(LED_BUILTIN, HIGH); delay(500);
  digitalWrite(LED_BUILTIN, LOW);
}

// =============================================
// MAIN LOOP
// =============================================
void loop() {
  unsigned long now = millis();

  // --- Heartbeat LED at 1 Hz ---
  if (now - lastBlinkTime >= 1000) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastBlinkTime = now;

    static unsigned long lastHeartbeat = 0;
    if (now - lastHeartbeat >= 5000) {
      Serial.println("Running... (waiting for encoder input)");
      lastHeartbeat = now;
    }
  }

  // --- Print every 500ms ---
  if (now - lastPrintTime >= 500) {
    float dt = (now - lastPrintTime) / 1000.0f;

    // Atomic read of all ISR state
    noInterrupts();
    int32_t rawCount   = isrCount;
    int8_t  direction  = isrDirection;
    bool    dirChanged = isrDirChanged;
    if (dirChanged) isrDirChanged = false;  // clear inside critical section
    interrupts();

    // Handle direction change safely in main loop
    if (dirChanged) {
      resetAnchor   = rawCount;
      totalDistance = 0.0f;
      lastCount     = rawCount;
      Serial.println(">> Direction changed — counter reset to 0");
    }

    int32_t relativeCount = rawCount - resetAnchor;
    int32_t deltaCount    = rawCount - lastCount;
    if (deltaCount < 0) deltaCount = 0;

    float deltaDist   = deltaCount * DISTANCE_PER_COUNT;
    totalDistance    += deltaDist;
    velocity          = deltaDist / dt;

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
}