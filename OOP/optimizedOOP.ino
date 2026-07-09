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

        return (kp * P) +
               (ki * integral) +
               (kd * D);
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
        (const uint8_t[])
        {
            A0,A1,A2,A3,A4,A5
        },
        SensorCount);
    }


    void calibrate()
    {
        for(int i=0;i<400;i++)
            qtr.calibrate();
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
};



class Robot
{

private:

    LineSensor sensor;


    // Competition starting values
    PIDController pid;


    int baseSpeed = 150;


    static constexpr uint8_t AIN1 = 2;
    static constexpr uint8_t AIN2 = 3;
    static constexpr uint8_t BIN1 = 4;
    static constexpr uint8_t BIN2 = 7;

    static constexpr uint8_t PWMA = 5;
    static constexpr uint8_t PWMB = 6;


    L298NX2 driver;


    void setMotorA(int speed)
    {
        speed = constrain(speed,-255,255);

        driver.setSpeedA(abs(speed));

        if(speed >= 0)
            driver.forwardA();
        else
            driver.backwardA();
    }


    void setMotorB(int speed)
    {
        speed = constrain(speed,-255,255);

        driver.setSpeedB(abs(speed));

        if(speed >= 0)
            driver.forwardB();
        else
            driver.backwardB();
    }


    void drive(int left,int right)
    {
        setMotorA(left);
        setMotorB(right);
    }


public:


    Robot() :
    pid(0.2,0.0,1.0),
    driver(PWMA,AIN1,AIN2,
           PWMB,BIN1,BIN2)
    {

    }



    void begin()
    {
        sensor.begin();


        pinMode(LED_BUILTIN,OUTPUT);


        digitalWrite(LED_BUILTIN,HIGH);

        sensor.calibrate();

        digitalWrite(LED_BUILTIN,LOW);
    }



    void update()
    {

        uint16_t position =
        sensor.read();


        // 6 sensor center
        int error =
        2500 - position;



        if(sensor.noLine())
        {

            if(error > 0)
                drive(-180,180);

            else
                drive(180,-180);


            return;
        }



        int correction =
        pid.update(error);



        drive(
            baseSpeed - correction,
            baseSpeed + correction
        );
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