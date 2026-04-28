#include "anchor_controller.h"
#include "controller.h"
#include <math.h>
#include <string.h>
#include "config.h"

static constexpr float EARTH_RADIUS_M = 6371000.0f;

float AnchorController::degToRad(float deg)
{
    return deg * 0.01745329251994329577f;
}

float AnchorController::radToDeg(float rad)
{
    return rad * 57.295779513082320876f;
}

void AnchorController::resetGpsAverage()
{
    mGpsIndex = 0;
    mGpsCount = 0;

    for (uint8_t i = 0; i < GPS_AVG_COUNT; ++i)
    {
        mLatBuf[i] = 0.0;
        mLonBuf[i] = 0.0;
    }
}

float AnchorController::distanceMeters(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg)
{
    const float lat1 = degToRad((float)lat1Deg);
    const float lon1 = degToRad((float)lon1Deg);
    const float lat2 = degToRad((float)lat2Deg);
    const float lon2 = degToRad((float)lon2Deg);

    const float dLat = lat2 - lat1;
    const float dLon = lon2 - lon1;

    const float a =
        sinf(dLat * 0.5f) * sinf(dLat * 0.5f) +
        cosf(lat1) * cosf(lat2) *
            sinf(dLon * 0.5f) * sinf(dLon * 0.5f);

    const float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
    return EARTH_RADIUS_M * c;
}

float AnchorController::bearingDeg(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg)
{
    const float lat1 = degToRad((float)lat1Deg);
    const float lon1 = degToRad((float)lon1Deg);
    const float lat2 = degToRad((float)lat2Deg);
    const float lon2 = degToRad((float)lon2Deg);

    const float dLon = lon2 - lon1;

    const float y = sinf(dLon) * cosf(lat2);
    const float x =
        cosf(lat1) * sinf(lat2) -
        sinf(lat1) * cosf(lat2) * cosf(dLon);

    return wrap360(radToDeg(atan2f(y, x)));
}

void AnchorController::onEnter(SystemState &sys)
{

    resetGpsAverage();
    // Om InputLogic redan har satt anchor från medelvärde,
    // skriv inte över den här.
    if (!sys.anchorActive)
    {
        if (sys.sensors.gpsValid)
        {
            sys.anchorLatDeg = sys.sensors.latitudeDeg;
            sys.anchorLonDeg = sys.sensors.longitudeDeg;
            sys.anchorActive = true;
        }
        else
        {
            sys.anchorActive = false;
        }
    }

    if (sys.sensors.headingValid)
    {
        sys.targetHeadingDeg = sys.sensors.headingDeg;
    }
}
ActuatorCommand AnchorController::update(float dtSec, SystemState &sys, PidController &headingPid)
{
    ActuatorCommand out{};
    strcpy(sys.sensors.autoState, "ANCHOR");

    if (!sys.anchorActive || !sys.sensors.gpsValid || !sys.sensors.headingValid)
    {
        strcpy(sys.sensors.autoState, "A_WAIT");
        out.thrustPct = 0.0f;
        out.steerPct = 0.0f;
        return out;
    }

    const float anchorRadiusM = 3.0f;
    const float fullThrustDistM = 12.0f;
    const float minAnchorThrustPct = 1.0f;
    const float maxAnchorThrustPct = 45.0f;

    mLatBuf[mGpsIndex] = sys.sensors.latitudeDeg;
    mLonBuf[mGpsIndex] = sys.sensors.longitudeDeg;

    mGpsIndex = (mGpsIndex + 1) % GPS_AVG_COUNT;

    if (mGpsCount < GPS_AVG_COUNT)
    {
        mGpsCount++;
    }

    if (mGpsCount < GPS_AVG_COUNT)
    {
        mGpsCount++;
    }

    double avgLat = 0.0;
    double avgLon = 0.0;

    for (uint8_t i = 0; i < mGpsCount; ++i)
    {
        avgLat += mLatBuf[i];
        avgLon += mLonBuf[i];
    }

    avgLat /= mGpsCount;
    avgLon /= mGpsCount;

    const float distM = distanceMeters(
        avgLat,
        avgLon,
        sys.anchorLatDeg,
        sys.anchorLonDeg);

    if (distM <= anchorRadiusM)
    {
        strcpy(sys.sensors.autoState, "HOLD");
        out.thrustPct = 0.0f;
        out.steerPct = 0.0f;
        return out;
    }

    const float targetBearingDeg = bearingDeg(
        avgLat,
        avgLon,
        sys.anchorLatDeg,
        sys.anchorLonDeg);

    const float headingError =
        shortestAngleErrorDeg(targetBearingDeg, sys.sensors.headingDeg);

    float steerCmd = headingPid.update(headingError, dtSec);
    out.steerPct = clampf(steerCmd, Limits::STEER_MIN_PCT, Limits::STEER_MAX_PCT);

    float thrustPct = minAnchorThrustPct;

    if (distM >= fullThrustDistM)
    {
        thrustPct = maxAnchorThrustPct;
    }
    else
    {
        const float t = (distM - anchorRadiusM) / (fullThrustDistM - anchorRadiusM);
        thrustPct = minAnchorThrustPct + t * (maxAnchorThrustPct - minAnchorThrustPct);
    }

    const float absHeadingError = fabsf(headingError);

    if (absHeadingError > 90.0f)
    {
        thrustPct = 0.0f;
    }
    else if (absHeadingError > 45.0f)
    {
        thrustPct *= 0.5f;
    }

    out.thrustPct = clampf(thrustPct, Limits::THRUST_MIN_PCT, Limits::THRUST_MAX_PCT);
    return out;
}