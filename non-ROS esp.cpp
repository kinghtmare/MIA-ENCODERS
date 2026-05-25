// =============================================
// SECTION 1: PIN DEFINITIONS FOR ESP32
// =============================================
#define outputA 23    // Channel A connected to GPIO23
#define outputB 19    // Channel B connected to GPIO19
#define LED_BUILTIN 2 // Built-in LED on most ESP32 boards (GPIO2)

// =============================================
// SECTION 2: ENCODER PARAMETERS
// =============================================
const float COUNTS_PER_REVOLUTION = 360.0;  // Adjust to your encoder
const float WHEEL_DIAMETER = 10.0;          // cm
const float WHEEL_CIRCUMFERENCE = 3.14159 * WHEEL_DIAMETER;

// =============================================
// SECTION 3: GLOBAL VARIABLES
// =============================================
volatile int counter = 0;     // 'volatile' needed for ESP32 interrupts
int lastCounter = 0;
int aLastState;
unsigned long lastTime = 0;
unsigned long lastBlinkTime = 0;
float velocity = 0.0;
float distancePerCount = 0.0;
bool ledState = false;

// =============================================
// SECTION 4: INTERRUPT SERVICE ROUTINE
// =============================================
void IRAM_ATTR encoderISR() {
  // Read both channels to determine direction
  bool a = digitalRead(outputA);
  bool b = digitalRead(outputB);
  
  if (a != aLastState) {
    if (b != a) {
      counter++;
    } else {
      counter--;
    }
  }
  aLastState = a;
}

// =============================================
// SECTION 5: SETUP
// =============================================
void setup() { 
  // Initialize pins
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Blink LED to show boot
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
  
  // Start serial
  Serial.begin(115200);
  
  // REMOVED the while(!Serial) loop - it was causing hanging
  // Just add a small delay instead
  delay(1000);
  
  Serial.println("=================================");
  Serial.println("ESP32 ENCODER READER");
  Serial.println("=================================");
  Serial.println("Channel A (P23) -> GPIO23");
  Serial.println("Channel B (P19) -> GPIO19");
  Serial.println("=================================");
  
  // Attach interrupt for ESP32
  attachInterrupt(digitalPinToInterrupt(outputA), encoderISR, CHANGE);
  
  // Read initial state
  aLastState = digitalRead(outputA);
  
  // Calculate distance per count
  distancePerCount = WHEEL_CIRCUMFERENCE / COUNTS_PER_REVOLUTION;
  
  Serial.print("Distance per count: ");
  Serial.print(distancePerCount, 4);
  Serial.println(" cm");
  Serial.println("Ready! Turn the encoder...");
  Serial.println();
  
  lastTime = millis();
  lastBlinkTime = millis();
  
  // Final blink to show setup complete
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
}

// =============================================
// SECTION 6: MAIN LOOP
// =============================================
void loop() { 
  unsigned long currentTime = millis();
  
  // Blink LED every 1 second to show ESP32 is running
  if (currentTime - lastBlinkTime >= 1000) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastBlinkTime = currentTime;
    
    // Also print a heartbeat message if no encoder activity
    static unsigned long lastHeartbeat = 0;
    if (currentTime - lastHeartbeat >= 5000) {
      Serial.println("ESP32 is running... (waiting for encoder input)");
      lastHeartbeat = currentTime;
    }
  }
  
  // Print encoder data every 500ms
  if (currentTime - lastTime >= 500) {
    float deltaTime = (currentTime - lastTime) / 1000.0;
    
    // Disable interrupts while reading counter
    noInterrupts();
    int currentCount = counter;
    interrupts();
    
    int deltaCount = currentCount - lastCounter;
    float distanceTraveled = deltaCount * distancePerCount;
    velocity = distanceTraveled / deltaTime;
    
    // Print to serial monitor
    Serial.print("Position: ");
    Serial.print(currentCount);
    Serial.print(" counts | Distance: ");
    Serial.print(currentCount * distancePerCount, 2);
    Serial.print(" cm | Velocity: ");
    Serial.print(velocity, 2);
    Serial.println(" cm/s");
    
    lastCounter = currentCount;
    lastTime = currentTime;
  }
}
