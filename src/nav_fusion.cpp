#include "nav_fusion.h"
#include <cstring>
#include "config.h"

void NavFusion::update(const GpsFix &gps, const ImuHeading &imu, SensorData &s)
{
    // reset varje cykel
    s.headingValid = false;
    s.gpsValid = false;
    s.speedValid = false;

    strcpy(s.headingSource, "NONE");

    // GPS
    s.gpsValid = gps.locationValid;
    s.speedValid = gps.speedValid;

    if (gps.locationValid)
    {
        s.latitudeDeg = gps.latDeg;
        s.longitudeDeg = gps.lonDeg;
    }

    s.satellites = gps.satellites;

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
    {
        s.courseOverGroundDeg = gps.courseDeg;
    }
    else
    {
        s.courseOverGroundDeg = 0.0f;
    }

    // Speed -> percentage
    const float maxSpeedMps = AutoConfig::MAX_SPEED_MPS;
    const float minSpeedThreshold = 0.3f;

    float speed = s.speedMps;

    if (speed < minSpeedThreshold)
    {
        speed = 0.0f;
    }

    float pct = (speed / maxSpeedMps) * 100.0f;

    if (pct < 0.0f)
        pct = 0.0f;
    if (pct > 100.0f)
        pct = 100.0f;

    s.speedPct = pct;
    

    // Heading selection
    static bool useGpsHeading = false;

    const float enterGpsSpeed = 0.6f;
    const float leaveGpsSpeed = 0.4f;

    if (gps.speedValid)
    {
        if (!useGpsHeading && gps.speedMps >= enterGpsSpeed)
        {
            useGpsHeading = true;
        }
        else if (useGpsHeading && gps.speedMps <= leaveGpsSpeed)
        {
            useGpsHeading = false;
        }
    }
    else
    {
        useGpsHeading = false;
    }

    if (useGpsHeading && gps.courseValid)
    {
        s.headingDeg = s.courseOverGroundDeg;
        s.headingValid = true;
        strcpy(s.headingSource, "GPS");
    }
    else if (imu.valid)
    {
        s.motorHeadingDeg = imu.headingDeg;
        s.motorImuValid = true;

        if (s.boatImuValid)
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

        s.headingDeg = imu.headingDeg;
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