/*PWM PINS 3,5,6,9,10,11
Built in led is PIN 13
*/
#include <L298N.h>
#include <QTRSensors.h>

constexpr uint8_t AIN1 = 2; // set arduino pins for motor driver
constexpr uint8_t AIN2 = 3; // set arduino pins for motor driver
constexpr uint8_t BIN1 = 4; // set arduino pins for motor driver
constexpr uint8_t BIN2 = 7; // set arduino pins for motor driver
constexpr uint8_t PWMA = 5; // set arduino pins for motor driver
constexpr uint8_t PWMB = 6; // set arduino pins for motor driver
constexpr uint8_t STBY = 8; // set arduino pins for motor driver

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

QTRSensors qtr;

constexpr uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

float Kp = 2.0 / 10.0;
float Ki = 2.0 / 10.0;
float Kd = 2.0 / 10.0;

uint16_t position;
int P, D, I, previousError, error;
int lsp, rsp;
int lfspeed = 150;

void setup()
{
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4}, SensorCount);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // LED calibration mode

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // calibration finished
}

void loop()
{
  robot_control();
}

void robot_control()
{
  position = qtr.readLineBlack(sensorValues);
  error = 2000 - position;

  if (sensorValues[0] >= 980 && sensorValues[1] >= 980 && sensorValues[2] >= 980 && sensorValues[3] >= 980 && sensorValues[4] >= 980)
  {
    if (previousError > 0)
    {
      motor_drive(-150, 150); // Sharp left turn
    }
    else
    {
      motor_drive(150, -150); // Sharp right turn
    }
  }
  else
  {
    Linefollow(error);
  }
}

void Linefollow(int error)
{
  P = error;
  I = I + error;
  D = error - previousError;
  I = constrain(I, -1000, 1000);

  float PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  lsp = constrain(lsp, -255, 255);
  rsp = constrain(rsp, -255, 255);

  motor_drive(lsp, rsp);
}

void motor_drive(int left, int right)
{
  // Right Motor (motor2)
  if (right >= 0)
  {
    motor2.setSpeed(right);
    motor2.forward();
  }
  else
  {
    motor2.setSpeed(-right);
    motor2.backward();
  }

  // Left Motor (motor1)
  if (left >= 0)
  {
    motor1.setSpeed(left);
    motor1.forward();
  }
  else
  {
    motor1.setSpeed(-left);
    motor1.backward();
  }
}