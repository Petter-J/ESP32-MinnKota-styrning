#include "navigation.h"
#include <Arduino.h>

bool Navigation::begin()
{
    const bool gpsOk = _gps.begin();

    const bool motorImuOk = _motorImu.begin(
        CompassConfig::SDA_PIN,
        CompassConfig::SCL_PIN,
        CompassConfig::FREQ_HZ,
        CompassConfig::M_HEADING_OFFSET_DEG);

    const bool boatImuOk = _boatImu.begin(
        BoatCompassConfig::SDA_PIN,
        BoatCompassConfig::SCL_PIN,
        BoatCompassConfig::FREQ_HZ,
        BoatCompassConfig::B_HEADING_OFFSET_DEG);

    Serial.printf(
        "[NAV] begin gps=%d motorImu=%d boatImu=%d\n",
        gpsOk ? 1 : 0,
        motorImuOk ? 1 : 0,
        boatImuOk ? 1 : 0);

    return gpsOk || motorImuOk || boatImuOk;
}

void Navigation::update(SensorData &sensors)
{
    GpsFix gpsFix{};
    ImuHeading motorImuHeading{};
    ImuHeading boatImuHeading{};

    _gps.update(gpsFix);

    _motorImu.update(motorImuHeading);
    _boatImu.update(boatImuHeading);

    _fusion.update(
        gpsFix,
        motorImuHeading,
        boatImuHeading,
        sensors);
}