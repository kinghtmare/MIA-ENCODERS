// =============================================
// STM32 HALL EFFECT ENCODER — BIDIRECTIONAL
// =============================================

#define outputA PA0
#define outputB PA1

const float COUNTS_PER_REVOLUTION = 11.0f;  // ← verify your datasheet
const float WHEEL_DIAMETER_CM     = 10.0f;
const float WHEEL_CIRCUMFERENCE   = 3.14159265f * WHEEL_DIAMETER_CM;
const float DISTANCE_PER_COUNT    = WHEEL_CIRCUMFERENCE / COUNTS_PER_REVOLUTION;

// =============================================
// ISR STATE
// =============================================
volatile int32_t isrCounter    = 0;   // bidirectional, can go negative
volatile int8_t  isrDirection  = 1;   // +1 or -1
volatile int8_t  aLastState    = 0;

// =============================================
// MAIN LOOP STATE
// =============================================
int32_t lastCounter    = 0;
unsigned long lastTime = 0;

// =============================================
// ISR — register read, direction + counter
// =============================================
void encoderISR() {
  bool a = (GPIOA->IDR >> 0) & 1;  // PA0
  bool b = (GPIOA->IDR >> 1) & 1;  // PA1

  if (a != (bool)aLastState) {
    if (b != a) {
      isrCounter++;
      isrDirection = 1;
    } else {
      isrCounter--;
      isrDirection = -1;
    }
  }
  aLastState = (int8_t)a;
}

// =============================================
// SETUP
// =============================================
void setup() {
  pinMode(outputA, INPUT);  // Hall effect — no pull-up
  pinMode(outputB, INPUT);

  Serial.begin(115200);
  delay(500);

  Serial.println("STM32 Hall Effect Encoder — Bidirectional");
  Serial.print("dist/count: ");
  Serial.print(DISTANCE_PER_COUNT, 4);
  Serial.println(" cm");
  Serial.println("+1 = Forward | -1 = Reverse");

  aLastState = (GPIOA->IDR >> 0) & 1;  // read initial state via register
  attachInterrupt(digitalPinToInterrupt(outputA), encoderISR, CHANGE);

  lastTime = millis();
}

// =============================================
// MAIN LOOP
// =============================================
void loop() {
  unsigned long now = millis();

  if (now - lastTime >= 500) {
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    // Atomic snapshot
    noInterrupts();
    int32_t currentCount = isrCounter;
    int8_t  direction    = isrDirection;
    interrupts();

    int32_t deltaCount  = currentCount - lastCounter;
    float deltaDist     = deltaCount * DISTANCE_PER_COUNT;   // negative if reversing
    float totalDist     = currentCount * DISTANCE_PER_COUNT; // net position from start
    float velocity      = deltaDist / dt;                    // negative if reversing

    Serial.print("Direction: "); Serial.print(direction == 1 ? "+1" : "-1");
    Serial.print(" | Distance: "); Serial.print(totalDist, 2); Serial.print(" cm");
    Serial.print(" | Velocity: "); Serial.print(velocity, 2); Serial.println(" cm/s");

    lastCounter = currentCount;
  }
}
