#include "navigation.h"
#include <Arduino.h>

bool Navigation::begin()
{
    const bool gpsOk = _gps.begin();

    const bool imuOk = _imu.begin(
        CompassConfig::RX_PIN,
        CompassConfig::TX_PIN,
        CompassConfig::BAUD,
        CompassConfig::M_HEADING_OFFSET_DEG);

    Serial.printf(
        "[NAV] begin gps=%d imu=%d\n",
        gpsOk ? 1 : 0,
        imuOk ? 1 : 0);

    return gpsOk || imuOk;
}

void Navigation::update(SensorData &sensors)
{
    static GpsFix gpsFix{};
    static ImuHeading motorImuHeading{};
    static uint32_t lastMotorImuUpdateMs = 0;

    const uint32_t now = millis();

    _gps.update(gpsFix);

    if (now - lastMotorImuUpdateMs >= 20)
    {
        lastMotorImuUpdateMs = now;
        _imu.update(motorImuHeading);
    }

    _fusion.update(
        gpsFix,
        motorImuHeading,
        sensors);
}