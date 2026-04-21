// Clean navigation version
#include "navigation.h"
#include <cstring>
#include <Arduino.h>
#include "config.h"

bool Navigation::begin()
{
    const bool gpsOk = _gps.begin();
    const bool imuOk = _imu.begin();

    Serial.printf("[NAV] begin gps=%d imu=%d\n", gpsOk ? 1 : 0, imuOk ? 1 : 0);

    return gpsOk || imuOk;
}

void Navigation::update(SensorData &sensors)
{
    GpsFix gpsFix{};
    ImuHeading imuHeading{};

    _gps.update(gpsFix);
    _imu.update(imuHeading);

    strcpy(sensors.headingSource, "NONE");

    // ----------------------------
    // GPS
    // ----------------------------
    sensors.gpsValid = gpsFix.locationValid;
    sensors.speedValid = gpsFix.speedValid;

    if (gpsFix.locationValid)
    {
        sensors.latitudeDeg = gpsFix.latDeg;
        sensors.longitudeDeg = gpsFix.lonDeg;
    }

    sensors.satellites = gpsFix.satellites;

    if (gpsFix.speedValid)
    {
        sensors.gpsSpeedMps = gpsFix.speedMps;
        sensors.speedMps = gpsFix.speedMps;
    }
    else
    {
        sensors.gpsSpeedMps = 0.0f;
        sensors.speedMps = 0.0f;
    }

    if (gpsFix.courseValid)
    {
        sensors.courseOverGroundDeg = gpsFix.courseDeg;
    }
    else
    {
        sensors.courseOverGroundDeg = 0.0f;
    }

    // ----------------------------
    // Speed → percentage (legacy)
    // ----------------------------
    const float maxSpeedMps = AutoConfig::MAX_SPEED_MPS;
    const float minSpeedThreshold = 0.3f; // ignore jitter

    float speed = sensors.speedMps;

    if (speed < minSpeedThreshold)
    {
        speed = 0.0f;
    }

    float pct = (speed / maxSpeedMps) * 100.0f;

    if (pct < 0.0f)
        pct = 0.0f;
    if (pct > 100.0f)
        pct = 100.0f;

    sensors.speedPct = pct;

    // ----------------------------
    // Heading selection
    // ----------------------------
    const float minHeadingSpeedMps = AutoConfig::MIN_GPS_COURSE_SPEED_MPS;

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
