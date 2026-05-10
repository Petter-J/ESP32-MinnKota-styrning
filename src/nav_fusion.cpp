#include "nav_fusion.h"
#include <cstring>
#include "config.h"

void NavFusion::update(
    const GpsFix &gps,
    const ImuHeading &imu,
    SensorData &s)
{
    s.headingValid = false;
    s.gpsValid = false;
    s.speedValid = false;

    strcpy(s.headingSource, "NONE");

    s.gpsValid = gps.locationValid;
    s.locationUpdated = gps.locationUpdated;
    s.speedValid = gps.speedValid;
    s.courseValid = gps.courseValid;

    if (gps.locationValid)
    {
        s.latitudeDeg = gps.latDeg;
        s.longitudeDeg = gps.lonDeg;
    }

    s.satellites = gps.satellites;
    s.satellitesInView = gps.satellitesInView;

    if (gps.speedValid)
    {
        s.gpsSpeedMps = gps.speedMps;
        s.speedMps = gps.speedMps;
    }
    else
    {
        s.gpsSpeedMps = 0.0f;
        s.speedMps = 0.0f;
    }

    if (gps.courseValid)
        s.courseOverGroundDeg = gps.courseDeg;
    else
        s.courseOverGroundDeg = 0.0f;

    const float maxSpeedMps = AutoConfig::MAX_SPEED_MPS;
    const float minSpeedThreshold = 0.3f;

    float speed = s.speedMps;
    if (speed < minSpeedThreshold)
        speed = 0.0f;

    float pct = (speed / maxSpeedMps) * 100.0f;
    pct = clampf(pct, 0.0f, 100.0f);

    s.speedPct = pct;

    static bool useGpsHeading = false;

    const float enterGpsSpeed = 0.6f;
    const float leaveGpsSpeed = 0.4f;

    if (gps.speedValid)
    {
        if (!useGpsHeading && gps.speedMps >= enterGpsSpeed)
            useGpsHeading = true;
        else if (useGpsHeading && gps.speedMps <= leaveGpsSpeed)
            useGpsHeading = false;
    }
    else
    {
        useGpsHeading = false;
    }

    if (imu.valid)
    {
        s.motorHeadingDeg = imu.headingDeg;
        s.motorPitchDeg = imu.pitchDeg;
        s.motorRollDeg = imu.rollDeg;
        s.motorImuValid = true;
    }
    else
    {
        s.motorImuValid = false;
    }

    static uint32_t lastBoatImuValidMs = 0;
    const uint32_t nowMs = millis();

    if (s.boatImuValid)
    {
        lastBoatImuValidMs = nowMs;
    }

    const bool boatImuUsable =
        s.boatImuValid ||
        ((uint32_t)(nowMs - lastBoatImuValidMs) <
         BoatCompassConfig::BOAT_HEADING_HOLD_MS);

    if (s.motorImuValid && boatImuUsable)
    {
        s.motorAngleDeg =
            shortestAngleErrorDeg(
                s.motorHeadingDeg,
                s.boatHeadingDeg);
    }
    else
    {
        s.motorAngleDeg = 0.0f;
    }

    if (useGpsHeading && gps.courseValid)
    {
        s.headingDeg = s.courseOverGroundDeg;
        s.headingValid = true;
        strcpy(s.headingSource, "GPS");
    }
    else if (boatImuUsable)
    {
        s.headingDeg = s.boatHeadingDeg;
        s.headingValid = true;
        strcpy(s.headingSource, "BIMU");
    }
    else if (s.motorImuValid)
    {
        s.headingDeg = s.motorHeadingDeg;
        s.headingValid = true;
        strcpy(s.headingSource, "MIMU");
    }
    else
    {
        s.headingDeg = 0.0f;
        s.headingValid = false;
        strcpy(s.headingSource, "NONE");
    }
}