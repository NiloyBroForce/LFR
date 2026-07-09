/* PWM PINS 3,5,6,9,10,11
   Built-in LED is PIN 13
*/

// Standard L298N Motor Driver Pins
constexpr uint8_t IN1 = 2;  
constexpr uint8_t IN2 = 3;  
constexpr uint8_t ENA = 5;  

constexpr uint8_t IN3 = 4;  
constexpr uint8_t IN4 = 7;  
constexpr uint8_t ENB = 6;  

// Sensor Pins
constexpr uint8_t SensorCount = 6;
const uint8_t sensorPins[SensorCount] = {A0, A1, A2, A3, A4, A5};

// Calibration Arrays
uint16_t sensorMin[SensorCount];
uint16_t sensorMax[SensorCount];
uint16_t sensorValues[SensorCount];

// SAFE & SLOW TUNING PROFILE
float Kp = 0.15;  // Gentle adjustments
float Ki = 0.00;  // Kept at zero to prevent wild over-corrections
float Kd = 1.80;  // Strong damping factor to eliminate wobbles

int P, D, I, previousError, error;
int lsp, rsp;
int lfspeed = 90; // Dropped to 90 for a steady, safe cruising speed

void setup();
void loop();
void robot_control();
uint16_t readLineBlack();
void Linefollow(int error);
void motor_drive(int left, int right);

void setup() {
  // Initialize L298N Control Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize Sensor Pins & Reset Min/Max bounds
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorMin[i] = 1023; 
    sensorMax[i] = 0;    
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Ensure motors are off at boot
  motor_drive(0, 0);

  // 1. Safe 5-second wait immediately after power-on
  unsigned long startupTimer = millis();
  while (millis() - startupTimer < 5000) {
    // Sitting perfectly still, waiting for power supply to stabilize
  }

  // 2. Begin Slow Calibration Spin
  digitalWrite(LED_BUILTIN, HIGH); // Indicator LED ON
  motor_drive(50, -50);            // Start gentle spin

  unsigned long calibrationTimer = millis();
  unsigned long lastSampleTime = millis();

  // Strict subtraction method prevents calculation logic lockups
  while (millis() - calibrationTimer < 5000) {
    unsigned long currentMillis = millis();
    
    // Sample sensors every 10ms
    if (currentMillis - lastSampleTime >= 10) {
      lastSampleTime = currentMillis;
      
      for (uint8_t j = 0; j < SensorCount; j++) {
        uint16_t value = analogRead(sensorPins[j]);
        if (value < sensorMin[j]) sensorMin[j] = value;
        if (value > sensorMax[j]) sensorMax[j] = value;
      }
    }
  }

  // 3. Calibration Finished - FORCE STOP
  motor_drive(0, 0);               
  digitalWrite(LED_BUILTIN, LOW);  // Indicator LED OFF
}

void loop() {
  robot_control();
}

void robot_control() {
  uint16_t position = readLineBlack();
  error = 2000 - position; 

  // Check if all core sensors see black (Using 900+ out of 1000 normalized range)
  if (sensorValues[0] >= 900 && sensorValues[1] >= 900 && sensorValues[2] >= 900 && sensorValues[3] >= 900 && sensorValues[4] >= 900) {
    if (previousError > 0) {
      motor_drive(-100, 100); // Controlled, safer sharp left turn speed
    } else {
      motor_drive(100, -100); // Controlled, safer sharp right turn speed
    }
  } else {
    Linefollow(error);
  }
}

uint16_t readLineBlack() {
  uint32_t avg = 0;
  uint32_t sum = 0;

  for (uint8_t i = 0; i < SensorCount; i++) {
    uint16_t value = analogRead(sensorPins[i]);

    // Normalize values between 0 (White) and 1000 (Black)
    if (value <= sensorMin[i]) value = 0;
    else if (value >= sensorMax[i]) value = 1000;
    else {
      value = map(value, sensorMin[i], sensorMax[i], 0, 1000);
    }
    
    sensorValues[i] = value;

    if (value > 200) {
      avg += (uint32_t)value * (i * 1000);
      sum += value;
    }
  }

  if (sum == 0) {
    if (previousError < 0) return (SensorCount - 1) * 1000; 
    else return 0; 
  }

  return avg / sum;
}

void Linefollow(int error) {
  P = error;
  I = I + error;
  D = error - previousError;
  I = constrain(I, -1000, 1000);

  float PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  // Enforce speed caps to keep the movement smooth
  lsp = constrain(lsp, -180, 180);
  rsp = constrain(rsp, -180, 180);

  motor_drive(lsp, rsp);
}

void motor_drive(int left, int right) {
  // Left Motor Control (ENA, IN1, IN2)
  if (left >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, left);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -left);
  }

  // Right Motor Control (ENB, IN3, IN4)
  if (right >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, right);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -right);
  }
}