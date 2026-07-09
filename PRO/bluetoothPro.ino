/*PWM PINS 3,5,6,9,10,11
Built in led is PIN 13
*/


#include <L298N.h>
#include <QTRSensors.h>
#include <SoftwareSerial.h>

constexpr uint8_t AIN1 = 2; // set arduino pins for motor driver
constexpr uint8_t AIN2 = 3; // set arduino pins for motor driver
constexpr uint8_t BIN1 = 4; // set arduino pins for motor driver
constexpr uint8_t BIN2 = 7; // set arduino pins for motor driver
constexpr uint8_t PWMA = 5; // set arduino pins for motor driver
constexpr uint8_t PWMB = 6; // set arduino pins for motor driver
constexpr uint8_t STBY = 8; // set arduino pins for motor driver

// Bluetooth Pins (Using SoftwareSerial). This enables uploading code while bluetooth module is connected to arduino
constexpr uint8_t BT_RX = 10;
constexpr uint8_t BT_TX = 11;

// Bluetooth Commands
// G or g for starting bot
// X or x for stopping bot
// P or p for chaning Kp  example p0.05 will set Kp to 0.05
// D or d for chaning Kd  example d1.5  will set Kd to 1.5
// I or i for chaning ki  example i2.0  will set Kd to 2.0
// S or s for chaning motor base speed example S180 will set (base speed) to 180

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);
QTRSensors qtr;
SoftwareSerial bluetooth(BT_RX, BT_TX);

constexpr uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// PID Constants (Tune via Bluetooth)
float Kp = 0.2;
float Ki = 0.0;
float Kd = 0.0;

uint16_t position;
int P = 0, D = 0, I = 0, previousError = 0, error = 0;
int lsp, rsp;
int lfspeed = 150;//base speed of motor

bool onoff = false;

void setup();
void loop();
void stopRobot();
void checkBluetooth();
void robot_control();
void Linefollow(int error);
void motor_drive(int left, int right);

void setup()
{
  // Initialize Bluetooth Serial
  bluetooth.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4,A5}, SensorCount);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // LED calibration mode

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  bluetooth.print("MIN\n");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    bluetooth.print(qtr.calibrationOn.minimum[i]);
    bluetooth.print("\t");
  }
  bluetooth.println();

  bluetooth.print("MAX\n");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    bluetooth.print(qtr.calibrationOn.maximum[i]);
    bluetooth.print("\t");
  }
  digitalWrite(LED_BUILTIN, LOW); // calibration finished
}
void stopRobot()
{
  motor1.stop();
  motor2.stop();

  I = 0;
  previousError = 0;

  lsp = 0;
  rsp = 0;

  bluetooth.println("ROBOT STOPPED + PID RESET");
}
void loop()
{
  checkBluetooth();
  if (onoff == true)
  {
    robot_control(); // Run normally
  }
  else
  {
    stopRobot();
  }
}

void checkBluetooth()
{
  if (bluetooth.available() > 0)
  {
    char key = bluetooth.read();

    if (key == 'G' || key == 'g')
    {
      onoff = true;
      bluetooth.println("START");
      return;
    }
    else if (key == 'X' || key == 'x')
    {
      onoff = false;
      bluetooth.println("STOPPED");
      return;
    }

    float value = bluetooth.parseFloat();

    switch (key)
    {
    case 'P':
    case 'p':
      Kp = value;
                bluetooth.println(Kp);

      break;
    case 'I':
    case 'i':
      Ki = value;
                bluetooth.println(Ki);

      break;
    case 'D':
    case 'd':
      Kd = value;
                bluetooth.println(Kd);

      break;
    case 'S':
    case 's':
      lfspeed = constrain((int)value, 0, 255);
                bluetooth.println(lfspeed);

      break;
    default:
      break;
    }
  }
}

void robot_control()
{
  position = qtr.readLineBlack(sensorValues);
  error = 2000 - position;

  if (sensorValues[0] >= 980 && sensorValues[1] >= 980 && sensorValues[2] >= 980 && sensorValues[3] >= 980 && sensorValues[4] >= 980)
  {
    if (previousError > 0)
      motor_drive(-150, 150);
    else
      motor_drive(150, -150);
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
  // Right Motor
  if (right >= 0)
  {
    motor2.setSpeed(right);
    motor2.forward();
  }
  else
  {
    motor2.setSpeed(right);
    motor2.backward();
  }

  // Left Motor
  if (left >= 0)
  {
    motor1.setSpeed(left);
    motor1.forward();
  }
  else
  {
    motor1.setSpeed(left);
    motor1.backward();
  }
}