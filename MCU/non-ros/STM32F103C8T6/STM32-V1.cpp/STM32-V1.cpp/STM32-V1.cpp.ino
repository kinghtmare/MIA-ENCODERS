
// =============================================
// SECTION 1: PIN DEFINITIONS FOR STM32
// =============================================
#define outputA PA0    // C1 connected to A0 (Pin PA0 on STM32)
#define outputB PA1    // C2 connected to A1 (Pin PA1 on STM32)

// =============================================
// SECTION 2: ENCODER PARAMETERS
// =============================================
const float COUNTS_PER_REVOLUTION = 360.0;  // Adjust to your encoder
const float WHEEL_DIAMETER = 10.0;          // cm
const float WHEEL_CIRCUMFERENCE = 3.14159 * WHEEL_DIAMETER;

// =============================================
// SECTION 3: GLOBAL VARIABLES
// =============================================
volatile int counter = 0;     // 'volatile' needed for STM32 interrupts
int lastCounter = 0;
int aState;
int aLastState;
unsigned long lastTime = 0;
float velocity = 0.0;
float distancePerCount = 0.0;

// =============================================
// SECTION 4: INTERRUPT SERVICE ROUTINE
// =============================================
void encoderISR() {
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
  
  // Start serial
  Serial.begin(9600);
  
  // Wait for Serial Monitor (for STM32)
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("=================================");
  Serial.println("STM32 ENCODER READER");
  Serial.println("=================================");
  Serial.println("C1 (Channel A) -> A0 (PA0)");
  Serial.println("C2 (Channel B) -> A1 (PA1)");
  Serial.println("=================================");
  
  // Attach interrupt for STM32
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
}

// =============================================
// SECTION 6: MAIN LOOP
// =============================================
void loop() { 
  unsigned long currentTime = millis();
  
  // Print data every 500ms
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
}
