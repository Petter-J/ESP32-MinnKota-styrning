#include "navigation.h"
#include <cstring>
#include <Arduino.h>
#include "config.h"

bool Navigation::begin()
{
    _gps.begin();
    _imu.begin();
    return true;
}

void Navigation::update(SensorData &sensors)
{
    GpsFix gpsFix{};
    ImuHeading imuHeading{};

    _gps.update(gpsFix);
    _imu.update(imuHeading);

    strcpy(sensors.headingSource, "NONE");

    sensors.gpsValid = gpsFix.locationValid;
    sensors.speedValid = gpsFix.speedValid;

    if (gpsFix.locationValid)
    {
        sensors.latitudeDeg = gpsFix.latDeg;
        sensors.longitudeDeg = gpsFix.lonDeg;
    }

    sensors.satellites = gpsFix.satellites;
    //
    if (gpsFix.speedValid)
    {
        sensors.gpsSpeedMps = gpsFix.speedMps;
        sensors.speedMps = gpsFix.speedMps; // 🔹 NY
    }
    else
    {
        sensors.gpsSpeedMps = 0.0f;
        sensors.speedMps = 0.0f; // 🔹 NY
    }

    if (gpsFix.courseValid)
    {
        sensors.courseOverGroundDeg = gpsFix.courseDeg;
    }
    else
    {
        sensors.courseOverGroundDeg = 0.0f;
    }

    const float maxSpeedMps = 2.5f;
    float pct = (sensors.gpsSpeedMps / maxSpeedMps) * 100.0f;

    if (pct < 0.0f)
        pct = 0.0f;
    if (pct > 100.0f)
        pct = 100.0f;

    sensors.speedPct = pct;

    const float minHeadingSpeedMps = 0.8f;

    if (gpsFix.courseValid &&
        gpsFix.speedValid &&
        gpsFix.speedMps >= minHeadingSpeedMps)
    {
        sensors.headingDeg = sensors.courseOverGroundDeg;
        sensors.headingValid = true;
        strcpy(sensors.headingSource, "GPS");
    }
    else if (imuHeading.valid)
    {
        sensors.headingDeg = imuHeading.headingDeg;
        sensors.headingValid = true;
        strcpy(sensors.headingSource, "IMU");
    }
    else
    {
        sensors.headingDeg = 0.0f;
        sensors.headingValid = false;
        strcpy(sensors.headingSource, "NONE");
    }
}