#include "navigation.h"
#include <Arduino.h>

bool Navigation::begin()
{
    const bool gpsOk = _gps.begin();
    const bool imuOk = _imu.begin();

    _imu.setHeadingOffset(CompassConfig::M_HEADING_OFFSET_DEG);

    Serial.printf("[NAV] begin gps=%d imu=%d\n", gpsOk ? 1 : 0, imuOk ? 1 : 0);

    return gpsOk || imuOk;
}

void Navigation::update(SensorData &sensors)
{
    GpsFix gpsFix{};
    ImuHeading motorImuHeading{};

    _gps.update(gpsFix);
    _imu.update(motorImuHeading);

    _fusion.update(gpsFix, motorImuHeading, sensors);

}