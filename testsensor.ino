#include <Arduino.h>
#include <QTRSensors.h>

QTRSensors qtr;

constexpr uint8_t SensorCount = 6;

uint16_t rawValues[SensorCount];
uint16_t calibratedValues[SensorCount];

void setup()
{
    Serial.begin(9600);

    qtr.setTypeAnalog();

    qtr.setSensorPins(
        (const uint8_t[]){
            A0, A1, A2, A3, A4, A5},
        SensorCount);

    Serial.println("Starting calibration...");
    Serial.println("Move sensors over black line and white surface");

    delay(1000);

    for (int i = 0; i < 400; i++)
    {
        qtr.calibrate();
        delay(5);
    }

    Serial.println("Calibration finished");
    Serial.println();
}

void loop()
{
    // Raw ADC values (0-1023)
    qtr.read(rawValues);

    // Calibrated values (0-1000)
    qtr.readCalibrated(calibratedValues);

    uint16_t position =
        qtr.readLineBlack(calibratedValues);

    Serial.print("RAW: ");

    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(rawValues[i]);
        Serial.print('\t');
    }

    Serial.print(" | CAL: ");

    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(calibratedValues[i]);
        Serial.print('\t');
    }

    Serial.print(" | POS: ");
    Serial.println(position);

    delay(100);
}