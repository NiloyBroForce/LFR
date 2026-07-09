/*PWM PINS 3,5,6,9,10,11
Built in led is PIN 13
*/

#include <L298NX2.h>
#include <QTRSensors.h>
#include <SoftwareSerial.h>

// Bluetooth Pins (Using SoftwareSerial). This enables uploading code while bluetooth module is connected to arduino
static constexpr uint8_t BT_RX = 10;
static constexpr uint8_t BT_TX = 11;

SoftwareSerial bluetooth(BT_RX, BT_TX);

class PIDController{
private:
    float kp;
    float ki;
    float kd;

    int integral = 0;
    int previousError = 0;

public:
    void setKp(float v)
    {
        kp = v;
    }

    void setKi(float v)
    {
        ki = v;
    }

    void setKd(float v)
    {
        kd = v;
    }
    float getKp() const { return kp; }
    float getKi() const { return ki; }
    float getKd() const { return kd; }

    PIDController(float p = 0.2, float i = 0.0, float d = 0.8)
        : kp(p), ki(i), kd(d)
    {
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

    int getLastError()
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
            (const uint8_t[]){A0, A1, A2, A3, A4, A5},
            SensorCount);
    }

    void calibrate()
    {
        for (int i = 0; i < 400; i++)
            qtr.calibrate();

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
    }

    uint16_t read()
    {
        return qtr.readLineBlack(values);
    }

     bool noLine()
    {
        for(int i=0;i<SensorCount;i++)
        {
            if(values[i] > 500)
                return false;
        }

        return true;
    }
    bool sharpLeft()
{
    return values[0] > sharpTurnThreshold ||
           values[1] > sharpTurnThreshold;
}


bool sharpRight()
{
    return values[4] > sharpTurnThreshold ||
           values[5] > sharpTurnThreshold;
}
};

class Robot
{
private:
    LineSensor sensor;

    PIDController pid;

    int baseSpeed = 150;
static constexpr int recoveryMinSpeed = 80;
static constexpr int recoveryMaxSpeed = 220;

static constexpr int sharpTurnThreshold = 700;

    static constexpr uint8_t AIN1 = 2; // set arduino pins for motor driver
    static constexpr uint8_t AIN2 = 3; // set arduino pins for motor driver
    static constexpr uint8_t BIN1 = 4; // set arduino pins for motor driver
    static constexpr uint8_t BIN2 = 7; // set arduino pins for motor driver
    static constexpr uint8_t PWMA = 5; // set arduino pins for motor driver
    static constexpr uint8_t PWMB = 6; // set arduino pins for motor driver
    static constexpr uint8_t STBY = 8; // set arduino pins for motor driver

    L298NX2 driver;

    bool robotstate = true;

    int lastDirection = 1;
    
   void handleLostLine()
{
    // Bigger last error means we were farther from the line
    int recoverySpeed = map(
        abs(pid.lastError()),
        0,
        2500,
        recoveryMinSpeed,
        recoveryMaxSpeed
    );

    recoverySpeed = constrain(
        recoverySpeed,
        recoveryMinSpeed,
        recoveryMaxSpeed
    );


    drive(
        -recoverySpeed * lastDirection,
         recoverySpeed * lastDirection
    );
}
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
    Robot() : pid(0.2, 0.0, 0.8), driver(PWMA, AIN1, AIN2, PWMB, BIN1, BIN2)
    {
    }
    void setSpeed(int v)
    {
        baseSpeed = constrain(v, 0, 255);
    }
    int getSpeed() const { return baseSpeed; }

    bool getState()
    {
        return robotstate;
    }
    void begin()
    {
        /*
        //enable this code if standby pin is used
        pinMode(STBY, OUTPUT);
        digitalWrite(STBY, HIGH);
        /*/
        bluetooth.begin(9600);

        sensor.begin();

        pinMode(LED_BUILTIN, OUTPUT);

        digitalWrite(LED_BUILTIN, HIGH); // LED calibration mode
        sensor.calibrate();
        digitalWrite(LED_BUILTIN, LOW); // calibration finished
    }

    void update()
    {
         uint16_t position =
        sensor.read();


        // 6 sensor center
        int error =
        2500 - position;

        if (sensor.noLine())
        {
            handleLostLine();
            return;
        }

        int correction =
    constrain(pid.update(error),
              -255,
              255);


lastDirection =
    (error >= 0) ? 1 : -1;


int speed = calculateSpeed(error);


drive(
    speed - correction,
    speed + correction
);
int calculateSpeed(int error)
{
    int speed = baseSpeed;


    // Strong error means sharp corner
    if(abs(error) > 1200)
    {
        speed *= 0.5;
    }


    // Outer sensors detect incoming corner
    if(sensor.sharpLeft() ||
       sensor.sharpRight())
    {
        speed *= 0.6;
    }


    return max(speed,80);
}
    }

    void checkBluetooth()
    {
        if (bluetooth.available() > 0)
        {
            char key = bluetooth.read();

            if (key == 'G' || key == 'g')
            {
                robotstate = true;
                bluetooth.println("START");
                return;
            }
            else if (key == 'X' || key == 'x')
            {
                robotstate = false;
                bluetooth.println("STOPPED");
                return;
            }

            float value = bluetooth.parseFloat();

            switch (key)
            {
            case 'P':
            case 'p':
                pid.setKp(value);
                bluetooth.println(pid.getKp());
                break;
            case 'I':
            case 'i':
                pid.setKi(value);
                bluetooth.println(pid.getKi());
                break;
            case 'D':
            case 'd':
                pid.setKd(value);
                bluetooth.println(pid.getKd());
                break;
            case 'S':
            case 's':
                setSpeed(value);
                bluetooth.println(getSpeed());
                break;
            default:
                break;
            }
        }
    }
   
    void stop()
    {
        driver.stop();
    }


};

Robot robot;

void setup()
{
    robot.begin();
}

void loop()
{
    robot.checkBluetooth();

    if (!robot.getState())
    {
        robot.stop();
        return;
    }

    robot.update();
}