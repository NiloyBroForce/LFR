/*PWM PINS 3,5,6,9,10,11
Built in led is PIN 13
*/
#include <L298NX2.h>
#include <QTRSensors.h>

class PIDController
{
private:
  float kp;
  float ki;
  float kd;

  int integral = 0;
  int previousError = 0;

public:
  PIDController(float p, float i, float d)
  {
    kp = p;
    ki = i;
    kd = d;
  }

  int update(int error)
  {
    int P = error;

    integral += error;
    integral = constrain(integral, -1000, 1000);

    int D = error - previousError;

    previousError = error;

    return kp * P + ki * integral + kd * D;
  }

  int lastError()
  {
    return previousError;
  }
};

class LineSensor
{
private:
  QTRSensors qtr;
  static constexpr uint8_t SensorCount = 6;
  uint16_t values[SensorCount];

public:

  void begin()
  {
    qtr.setTypeAnalog();
    qtr.setSensorPins(
        (const uint8_t[]){A0, A1, A2, A3, A4,A5},
        SensorCount);
  }

  void calibrate()
  {
    for (int i = 0; i < 400; i++)
      qtr.calibrate();
  }

  uint16_t read()
  {
    return qtr.readLineBlack(values);
  }

  bool allBlack()
  {
    for (int i = 0; i < SensorCount; i++)
      if (values[i] < 980)
        return false;

    return true;
  }
};

class Robot
{
private:

  LineSensor sensor;

  PIDController pid;

  int baseSpeed=150;

static constexpr uint8_t AIN1 = 2; // set arduino pins for motor driver
static constexpr uint8_t AIN2 = 3; // set arduino pins for motor driver
static constexpr uint8_t BIN1 = 4; // set arduino pins for motor driver
static constexpr uint8_t BIN2 = 7; // set arduino pins for motor driver
static constexpr uint8_t PWMA = 5; // set arduino pins for motor driver
static constexpr uint8_t PWMB = 6; // set arduino pins for motor driver
static constexpr uint8_t STBY = 8; // set arduino pins for motor driver

L298NX2 driver;
  void setMotorA(int speed)
  {
    speed = constrain(speed, -255, 255);
    driver.setSpeedA(abs(speed));
    speed >= 0 ? driver.forwardA() : driver.backwardA();
  }

  void setMotorB(int speed)
  {
    speed = constrain(speed, -255, 255);
    driver.setSpeedB(abs(speed));
    speed >= 0 ? driver.forwardB() : driver.backwardB();
  }
  void drive(int leftSpeed, int rightSpeed)
  {
    setMotorA(constrain(leftSpeed, -255, 255));
    setMotorB(constrain(rightSpeed, -255, 255));
  }
public:
  Robot() : pid(0.2, 0.2, 0.2), driver(PWMA, AIN1, AIN2, PWMB, BIN1, BIN2)
  {
    
  }

  void begin()
  {
    /*
    //enable this code if standby pin is used
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);
    /*/
    sensor.begin();

    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, HIGH); // LED calibration mode
    sensor.calibrate();
    digitalWrite(LED_BUILTIN, LOW); // calibration finished
  }

  void update()
  {
    int position = sensor.read();

    int error = 2000 - position;

    if (sensor.allBlack())
    {
      if (error > 0)
        drive(-150, 150);
      else
        drive(150, -150);

      return;
    }

    int correction = pid.update(error);

    drive(baseSpeed - correction,
          baseSpeed + correction);
  }


};

Robot robot;

void setup()
{
  robot.begin();
}

void loop()
{
  robot.update();
}
