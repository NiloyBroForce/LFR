
#include <Arduino.h>
#include <L298NX2.h>
#include <QTRSensors.h>
#include <EEPROM.h>

static constexpr uint8_t SENSOR_COUNT = 6;
static constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {A0, A1, A2, A3, A4, A5};
// static constexpr uint8_t EMITTER_PIN = 8; // set to QTR_NO_EMITTER_PIN if unused

static constexpr uint8_t AIN1 = 2;
static constexpr uint8_t AIN2 = 3;
static constexpr uint8_t BIN1 = 4;
static constexpr uint8_t BIN2 = 7;
static constexpr uint8_t PWMA = 5;
static constexpr uint8_t PWMB = 6;

static constexpr uint8_t CALIBRATE_BUTTON = 9; // optional: hold LOW at boot to force fresh calibration
static constexpr int EEPROM_MAGIC_ADDR = 0;
static constexpr uint8_t EEPROM_MAGIC_VAL = 0xA5;

// ===================================================================
//  TUNING CONSTANTS  (start here, adjust after test runs)
// ===================================================================
// PID gains
static constexpr float KP = 0.045f;
static constexpr float KI = 0.0006f;
static constexpr float KD = 0.55f;
static constexpr float D_FILTER_ALPHA = 0.6f; // 0..1, higher = trust new sample more

// Speed profile
static constexpr int SPEED_MAX = 220;     // top speed on straights
static constexpr int SPEED_MIN = 90;      // floor speed in hard corners
static constexpr int SPEED_SEARCH = 110;  // pivot speed while searching for line
static constexpr int SPEED_RAMP_STEP = 4; // max change in speed per loop (acceleration limiter)

// Error thresholds (raw error is centerVal - position, centerVal = (SENSOR_COUNT-1)*1000/2)
static constexpr int ERROR_CORNER_1 = 800;   // mild turn -> start shedding speed
static constexpr int ERROR_CORNER_2 = 1500;  // hard turn -> drop to near SPEED_MIN
static constexpr int SENSOR_SATURATED = 900; // 0-1000 scale, "fully on black" threshold on an outer sensor

// Line-loss / gap handling (spec explicitly allows gaps in the line)
static constexpr unsigned long GAP_GRACE_MS = 150;       // coast straight this long before searching
static constexpr unsigned long SEARCH_TIMEOUT_MS = 2500; // safety stop if line never found again

// Sharp turn confirmation - avoid false triggers from noise
static constexpr uint8_t SHARP_TURN_CONFIRM_COUNT = 3;

// Calibration sweep tuning - how wide/long the robot rocks left-right
// over the line at boot. Increase these if a sensor never reaches a
// solid black/white extreme during calibration (check with the
// debug print in Robot::calibrateInPlace).
static constexpr int CALIBRATION_PIVOT_SPEED = 110;   // motor speed while rocking
static constexpr int CALIBRATION_SWEEP_SAMPLES = 40;  // samples per half-rock (edge to center)
static constexpr int CALIBRATION_CROSS_SAMPLES = 80;  // samples for the full-width crossing
static constexpr int CALIBRATION_ROUNDS = 4;          // how many times to repeat the rock
static constexpr int CALIBRATION_SAMPLE_DELAY_MS = 5; // delay between samples (also sets rock speed)

// ===================================================================
//  PID CONTROLLER
// ===================================================================
class PIDController
{
private:
    float kp, ki, kd;
    float integral = 0;
    float prevError = 0;
    float filteredDerivative = 0;
    unsigned long lastTimeUs = 0;
    bool firstRun = true;

public:
    PIDController(float p, float i, float d) : kp(p), ki(i), kd(d) {}

    void reset()
    {
        integral = 0;
        prevError = 0;
        filteredDerivative = 0;
        firstRun = true;
    }

    float update(float error)
    {
        unsigned long now = micros();
        float dt = firstRun ? 0.01f : (now - lastTimeUs) / 1e6f;
        if (dt <= 0)
            dt = 0.001f;
        lastTimeUs = now;
        firstRun = false;

        // Integral with anti-windup clamp
        integral += error * dt;
        integral = constrain(integral, -3000.0f, 3000.0f);

        // Raw derivative, then low-pass filtered to kill sensor jitter
        float rawDerivative = (error - prevError) / dt;
        filteredDerivative = D_FILTER_ALPHA * rawDerivative +
                             (1 - D_FILTER_ALPHA) * filteredDerivative;
        prevError = error;

        return (kp * error) + (ki * integral) + (kd * filteredDerivative);
    }
};


class LineSensor
{
private:
    QTRSensors qtr;
    uint16_t values[SENSOR_COUNT];
    static constexpr int CENTER = (SENSOR_COUNT - 1) * 1000 / 2; // 2500 for 6 sensors

public:
    void begin()
    {
        qtr.setTypeAnalog();
        qtr.setSensorPins((const uint8_t[]){SENSOR_PINS[0], SENSOR_PINS[1], SENSOR_PINS[2],
                                            SENSOR_PINS[3], SENSOR_PINS[4], SENSOR_PINS[5]},
                          SENSOR_COUNT);
        // if (EMITTER_PIN != QTR_NO_EMITTER_PIN)
        // qtr.setEmitterPin(EMITTER_PIN);
    }

    // Take one calibration sample (call repeatedly while physically
    // sweeping the robot over the line - see Robot::calibrateInPlace).
    void calibrateSample()
    {
        qtr.calibrate();
    }

    void printCalibrationDebug()
    {
        Serial.println(F("--- calibration min/max per sensor ---"));
        for (uint8_t i = 0; i < SENSOR_COUNT; i++)
        {
            Serial.print(i);
            Serial.print(F(": min="));
            Serial.print(qtr.calibrationOn.minimum[i]);
            Serial.print(F(" max="));
            Serial.println(qtr.calibrationOn.maximum[i]);
        }
    }

    void saveCalibrationToEEPROM()
    {
        int addr = EEPROM_MAGIC_ADDR;
        EEPROM.update(addr++, EEPROM_MAGIC_VAL);
        for (uint8_t i = 0; i < SENSOR_COUNT; i++)
        {
            EEPROM.put(addr, qtr.calibrationOn.minimum[i]);
            addr += sizeof(uint16_t);
            EEPROM.put(addr, qtr.calibrationOn.maximum[i]);
            addr += sizeof(uint16_t);
        }
    }

    bool loadCalibrationFromEEPROM()
    {
        int addr = EEPROM_MAGIC_ADDR;
        if (EEPROM.read(addr) != EEPROM_MAGIC_VAL)
            return false;
        addr++;

        qtr.calibrationOn.initialized = true;
        static uint16_t mins[SENSOR_COUNT], maxs[SENSOR_COUNT];
        for (uint8_t i = 0; i < SENSOR_COUNT; i++)
        {
            EEPROM.get(addr, mins[i]);
            addr += sizeof(uint16_t);
            EEPROM.get(addr, maxs[i]);
            addr += sizeof(uint16_t);
        }
        qtr.calibrationOn.minimum = mins;
        qtr.calibrationOn.maximum = maxs;
        return true;
    }

    // Returns 0..5000 style position (QTR convention)
    uint16_t read()
    {
        return qtr.readLineBlack(values);
    }

    int error()
    {
        return CENTER - (int)read();
    }

    bool noLine()
    {
        for (uint8_t i = 0; i < SENSOR_COUNT; i++)
            if (values[i] > 200)
                return false;
        return true;
    }

    // Outer sensor saturation = strong signal of a sharp corner / right angle
    bool outerLeftSaturated() { return values[0] > SENSOR_SATURATED; }
    bool outerRightSaturated() { return values[SENSOR_COUNT - 1] > SENSOR_SATURATED; }

    int strength()
    {
        int total = 0;
        for (uint8_t i = 0; i < SENSOR_COUNT; i++)
            total += values[i];
        return total;
    }
};

// ===================================================================
//  ROBOT
// ===================================================================
enum RobotState
{
    FOLLOWING,
    GAP,
    SEARCHING,
    SHARP_TURN
};

class Robot
{
private:
    LineSensor sensor;
    PIDController pid;
    L298NX2 driver;

    RobotState state = FOLLOWING;
    int lastError = 0;            // sign tells us which side the line was last on
    int currentSpeed = SPEED_MIN; // ramped speed, avoids sudden jerks
    unsigned long lineLostAt = 0;
    unsigned long searchStartedAt = 0;
    uint8_t sharpTurnCounter = 0;

    void setMotorA(int speed)
    {
        speed = constrain(speed, -255, 255);
        driver.setSpeedA(abs(speed));
        if (speed >= 0)
            driver.forwardA();
        else
            driver.backwardA();
    }

    void setMotorB(int speed)
    {
        speed = constrain(speed, -255, 255);
        driver.setSpeedB(abs(speed));
        if (speed >= 0)
            driver.forwardB();
        else
            driver.backwardB();
    }

    void drive(int left, int right)
    {
        setMotorA(left);
        setMotorB(right);
    }

    void stopMotors() { drive(0, 0); }

    // Dynamic base speed: fast on straights, progressively slower into
    // corners, with a ramp limiter so it doesn't jump abruptly (which
    // would upset the PID and risk skidding off the 3cm-wide line).
    int calculateTargetSpeed(int error)
    {
        int target = SPEED_MAX;
        int absErr = abs(error);

        if (absErr > ERROR_CORNER_1)
        {
            // linear taper between corner-1 and corner-2 thresholds
            int span = ERROR_CORNER_2 - ERROR_CORNER_1;
            int over = constrain(absErr - ERROR_CORNER_1, 0, span);
            target = map(over, 0, span, SPEED_MAX, SPEED_MIN);
        }

        if (sensor.outerLeftSaturated() || sensor.outerRightSaturated())
            target = min(target, SPEED_MIN + 20);

        return constrain(target, SPEED_MIN, SPEED_MAX);
    }

    void rampSpeedTowards(int target)
    {
        if (currentSpeed < target)
            currentSpeed = min(currentSpeed + SPEED_RAMP_STEP, target);
        else if (currentSpeed > target)
            currentSpeed = max(currentSpeed - SPEED_RAMP_STEP, target);
    }

public:
    Robot() : pid(KP, KI, KD),
              driver(PWMA, AIN1, AIN2, PWMB, BIN1, BIN2)
    {
    }

    void begin()
    {
        pinMode(LED_BUILTIN, OUTPUT);
        pinMode(CALIBRATE_BUTTON, INPUT_PULLUP);

        sensor.begin();

        bool forceCalibrate = (digitalRead(CALIBRATE_BUTTON) == LOW);

        if (!forceCalibrate && sensor.loadCalibrationFromEEPROM())
        {
            // Reuse last calibration - blink twice to confirm and skip
            // the physical sweep (saves time between competition runs).
            for (int i = 0; i < 2; i++)
            {
                digitalWrite(LED_BUILTIN, HIGH);
                unsigned long start = millis();
                while (millis() - start < 150)
                {
                }

                digitalWrite(LED_BUILTIN, LOW);
                start = millis();
                while (millis() - start < 150)
                {
                }
            }
        }
        else
        {
            calibrateInPlace();
        }
    }

    // Physically sweeps the robot left/right across the line while
    // sampling, then stores the min/max readings to EEPROM so the
    // next boot can skip this step (hold CALIBRATE_BUTTON to redo it).
    void calibrateInPlace()
    {
        digitalWrite(LED_BUILTIN, HIGH);
        for (int sweep = 0; sweep < CALIBRATION_ROUNDS; sweep++)
        {
            unsigned long start = millis();
            drive(-CALIBRATION_PIVOT_SPEED, CALIBRATION_PIVOT_SPEED);
            for (int j = 0; j < CALIBRATION_SWEEP_SAMPLES; j++)
            {
                sensor.calibrateSample();
                while (millis() - start < CALIBRATION_SAMPLE_DELAY_MS)
                {
                }
            }

            drive(CALIBRATION_PIVOT_SPEED, -CALIBRATION_PIVOT_SPEED);
            for (int j = 0; j < CALIBRATION_CROSS_SAMPLES; j++)
            {
                sensor.calibrateSample();
                while (millis() - start < CALIBRATION_SAMPLE_DELAY_MS)
                {
                }
            }

            drive(-CALIBRATION_PIVOT_SPEED, CALIBRATION_PIVOT_SPEED);
            for (int j = 0; j < CALIBRATION_SWEEP_SAMPLES; j++)
            {
                sensor.calibrateSample();
                while (millis() - start < CALIBRATION_SAMPLE_DELAY_MS)
                {
                }
            }
        }
        stopMotors();
        sensor.saveCalibrationToEEPROM();
        sensor.printCalibrationDebug(); // check Serial Monitor: widen sweep if any sensor's
                                        // min/max range looks too narrow (e.g. under ~300)
        digitalWrite(LED_BUILTIN, LOW);
    }

    void update()
    {
        sensor.read();
        int error = sensor.error();

        bool sharpLeft = sensor.outerLeftSaturated();
        bool sharpRight = sensor.outerRightSaturated();
        bool lost = sensor.noLine();

        // ---- State transitions -----------------------------------
        if (!lost)
        {
            lastError = error;
            sharpTurnCounter = (sharpLeft || sharpRight) ? min(sharpTurnCounter + 1, 255) : 0;

            if (sharpTurnCounter >= SHARP_TURN_CONFIRM_COUNT && abs(error) > ERROR_CORNER_2)
                state = SHARP_TURN;
            else
                state = FOLLOWING;
        }
        else
        {
            if (state == FOLLOWING || state == SHARP_TURN)
            {
                state = GAP;
                lineLostAt = millis();
            }
            else if (state == GAP && millis() - lineLostAt > GAP_GRACE_MS)
            {
                state = SEARCHING;
                searchStartedAt = millis();
            }
            else if (state == SEARCHING && millis() - searchStartedAt > SEARCH_TIMEOUT_MS)
            {
                // Truly lost - stop rather than wander off the arena
                stopMotors();
                return;
            }
        }

        // ---- Behavior per state ----------------------------------
        switch (state)
        {
        case FOLLOWING:
        {
            int target = calculateTargetSpeed(error);
            rampSpeedTowards(target);

            float correction = pid.update((float)error);
            drive(currentSpeed - (int)correction,
                  currentSpeed + (int)correction);
            break;
        }

        case SHARP_TURN:
        {
            // Confirmed hard corner (45 deg / 150 deg bends, right
            // angles): controlled pivot toward the side with signal,
            // slower than a full spin so we don't overshoot the
            // narrow 3cm line on exit.
            rampSpeedTowards(SPEED_MIN);
            if (error > 0)
                drive(-currentSpeed / 2, currentSpeed);
            else
                drive(currentSpeed, -currentSpeed / 2);
            break;
        }

        case GAP:
        {
            // Line spec allows gaps - coast straight on the last
            // known good speed/heading rather than reacting to
            // sensor noise.
            drive(currentSpeed, currentSpeed);
            break;
        }

        case SEARCHING:
        {
            // Pivot toward whichever side the line was last seen on.
            if (lastError >= 0)
                drive(-SPEED_SEARCH, SPEED_SEARCH);
            else
                drive(SPEED_SEARCH, -SPEED_SEARCH);
            break;
        }
        }
    }
};

Robot robot;

void setup()
{
    Serial.begin(9600);
    robot.begin();
}

void loop()
{
    robot.update();
}
