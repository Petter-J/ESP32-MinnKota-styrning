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

static float clampAnchorThrust(float thrustPct)
{
    return clampf(
        thrustPct,
        AnchorConfig::MIN_THRUST_PCT,
        AnchorConfig::MAX_THRUST_PCT);
}

void AnchorController::onEnter(SystemState &sys)
{
    resetGpsAverage();

    mWasInsideRadius = true;
    mOutsideSinceMs = 0;
    mReturnStartMs = 0;

    mDriftStartMs = 0;
    mDriftTimeSumMs = 0;
    mDriftSamples = 0;

    mStartZoneHits = 0;
    mStopZoneHits = 0;

    mAnchorMode = AnchorMode::Learning;

    mAnchorLearnedThrustPct =
        clampAnchorThrust(AnchorConfig::START_THRUST_PCT);

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

    if (sys.sensors.motorImuValid)
    {
        sys.targetHeadingDeg = sys.sensors.motorHeadingDeg;
    }
}

void AnchorController::onExit()
{
    resetGpsAverage();

    mWasInsideRadius = true;
    mOutsideSinceMs = 0;
    mReturnStartMs = 0;

    mDriftStartMs = 0;
    mDriftTimeSumMs = 0;
    mDriftSamples = 0;

    mStartZoneHits = 0;
    mStopZoneHits = 0;

    mAnchorMode = AnchorMode::Learning;

    mAnchorLearnedThrustPct =
        clampAnchorThrust(AnchorConfig::START_THRUST_PCT);
}

ActuatorCommand AnchorController::update(float dtSec, SystemState &sys, PidController &headingPid)
{
    ActuatorCommand out{};
    strcpy(sys.sensors.autoState, "ANCHOR");

    if (!sys.anchorActive || !sys.sensors.gpsValid || !sys.sensors.motorImuValid)
    {
        strcpy(sys.sensors.autoState, "A_WAIT");
        headingPid.reset();
        out.thrustPct = 0.0f;
        out.steerPct = 0.0f;
        return out;
    }

    const float stopRadiusM = AnchorConfig::STOP_RADIUS_M;

    const float startRadiusM =
        (mAnchorMode == AnchorMode::Learning)
            ? AnchorConfig::LEARN_START_RADIUS_M
            : AnchorConfig::START_RADIUS_M;

    const float fullThrustDistM = AnchorConfig::FULL_THRUST_DIST_M;
    const float maxAnchorThrustPct = AnchorConfig::MAX_THRUST_PCT;

    if (sys.sensors.gpsValid)
    {
        mLatBuf[mGpsIndex] = sys.sensors.latitudeDeg;
        mLonBuf[mGpsIndex] = sys.sensors.longitudeDeg;

        mGpsIndex = (mGpsIndex + 1) % GPS_AVG_COUNT;

        if (mGpsCount < GPS_AVG_COUNT)
        {
            mGpsCount++;
        }
    }

    if (mGpsCount == 0)
    {
        strcpy(sys.sensors.autoState, "A_GPSAVG");
        headingPid.reset();
        out.thrustPct = 0.0f;
        out.steerPct = 0.0f;
        return out;
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

    const float distAvgM = distanceMeters(
        avgLat,
        avgLon,
        sys.anchorLatDeg,
        sys.anchorLonDeg);

    const float distRawM = distanceMeters(
        sys.sensors.latitudeDeg,
        sys.sensors.longitudeDeg,
        sys.anchorLatDeg,
        sys.anchorLonDeg);

    const uint32_t nowMs = millis();

    if (mWasInsideRadius)
    {
        if (distRawM >= startRadiusM && mStartZoneHits < START_CONFIRM_COUNT)
        {
            mStartZoneHits++;
        }
    }
    else
    {
        if (distRawM <= stopRadiusM && mStopZoneHits < STOP_CONFIRM_COUNT)
        {
            mStopZoneHits++;
        }
    }

    const bool insideStop = (!mWasInsideRadius && (mStopZoneHits >= STOP_CONFIRM_COUNT));
    const bool outsideStart = (mWasInsideRadius && (mStartZoneHits >= START_CONFIRM_COUNT));

    const bool returnActive = outsideStart || !mWasInsideRadius;
    const bool inDriftZone = !insideStop && !returnActive;

    if (insideStop)
    {
        if (!mWasInsideRadius && mReturnStartMs != 0)
        {
            const uint32_t returnTimeMs = nowMs - mReturnStartMs;

            const uint32_t target = AnchorConfig::TARGET_RETURN_TIME_MS;
            const uint32_t deadband = AnchorConfig::RETURN_TIME_DEADBAND_MS;

            if (returnTimeMs > target + deadband)
            {
                mAnchorLearnedThrustPct += AnchorConfig::THRUST_ADJUST_STEP_PCT;
            }
            else if (returnTimeMs < target - deadband)
            {
                mAnchorLearnedThrustPct -= AnchorConfig::THRUST_ADJUST_STEP_PCT;
            }

            mAnchorLearnedThrustPct = clampAnchorThrust(mAnchorLearnedThrustPct);
        }

        mWasInsideRadius = true;
        mOutsideSinceMs = 0;
        mReturnStartMs = 0;

        mStartZoneHits = 0;
        mStopZoneHits = 0;

        if (mAnchorMode == AnchorMode::Learning && mDriftStartMs == 0)
        {
            mDriftStartMs = nowMs;
        }

        if (mAnchorMode == AnchorMode::Learning)
        {
            strcpy(sys.sensors.autoState, "L_HOLD");
        }
        else if (mAnchorMode == AnchorMode::Maintenance)
        {
            strcpy(sys.sensors.autoState, "M_HOLD");
        }
        else
        {
            strcpy(sys.sensors.autoState, "HOLD");
        }

        headingPid.reset();
        out.thrustPct = 0.0f;
        out.steerPct = 0.0f;
        return out;
    }

    if (!mWasInsideRadius && returnActive)
    {
        if (mAnchorMode == AnchorMode::Learning)
        {
            strcpy(sys.sensors.autoState, "LEARN_RET");
        }
        else if (mAnchorMode == AnchorMode::Maintenance)
        {
            strcpy(sys.sensors.autoState, "M_RETURN");
        }
        else
        {
            strcpy(sys.sensors.autoState, "RETURN");
        }

        const float targetBearingDeg = bearingDeg(
            sys.sensors.latitudeDeg,
            sys.sensors.longitudeDeg,
            sys.anchorLatDeg,
            sys.anchorLonDeg);

        float headingError =
            shortestAngleErrorDeg(targetBearingDeg, sys.sensors.motorHeadingDeg);

        if (fabsf(headingError) < AnchorConfig::HEADING_DEADBAND_DEG)
        {
            headingError = 0.0f;
        }

        float steerCmd = headingPid.update(headingError, dtSec);

        out.steerPct = clampf(
            steerCmd,
            Limits::STEER_MIN_PCT,
            Limits::STEER_MAX_PCT);

        float thrustPct = mAnchorLearnedThrustPct;

        if (distAvgM >= fullThrustDistM)
        {
            thrustPct = maxAnchorThrustPct;
        }
        else if (distAvgM > startRadiusM)
        {
            const float denom = fullThrustDistM - startRadiusM;

            if (denom > 0.01f)
            {
                const float t = (distAvgM - startRadiusM) / denom;
                thrustPct =
                    mAnchorLearnedThrustPct +
                    t * (maxAnchorThrustPct - mAnchorLearnedThrustPct);
            }
        }

        const float absHeadingError = fabsf(headingError);

        if (absHeadingError > 90.0f)
        {
            thrustPct = AnchorConfig::MIN_THRUST_PCT;
        }
        else if (absHeadingError > 45.0f)
        {
            thrustPct *= 0.5f;
        }

        out.thrustPct = clampf(
            thrustPct,
            Limits::THRUST_MIN_PCT,
            Limits::THRUST_MAX_PCT);

        return out;
    }

    if (outsideStart)
    {
        mWasInsideRadius = false;
        mOutsideSinceMs = nowMs;
        mReturnStartMs = nowMs;

        mStopZoneHits = 0;

        if (mAnchorMode == AnchorMode::Learning && mDriftStartMs != 0)
        {
            const uint32_t driftTimeMs = nowMs - mDriftStartMs;

            mDriftTimeSumMs += driftTimeMs;

            if (mDriftSamples < 255)
            {
                mDriftSamples++;
            }

            mDriftStartMs = 0;

            const uint32_t target = AnchorConfig::TARGET_DRIFT_TIME_MS;
            const uint32_t deadband = AnchorConfig::DRIFT_TIME_DEADBAND_MS;

            if (driftTimeMs < target - deadband)
            {
                mAnchorLearnedThrustPct += AnchorConfig::THRUST_ADJUST_STEP_PCT;
            }
            else if (driftTimeMs > target + deadband)
            {
                mAnchorLearnedThrustPct -= AnchorConfig::THRUST_ADJUST_STEP_PCT;
            }

            mAnchorLearnedThrustPct = clampAnchorThrust(mAnchorLearnedThrustPct);

            if (mDriftSamples >= AnchorConfig::DRIFT_LEARN_SAMPLES)
            {
                const uint32_t avgDriftTimeMs = mDriftTimeSumMs / mDriftSamples;

                if (avgDriftTimeMs < AnchorConfig::TARGET_DRIFT_TIME_MS)
                {
                    mAnchorMode = AnchorMode::Maintenance;
                }
                else
                {
                    mAnchorMode = AnchorMode::OnOff;
                }

                mDriftStartMs = 0;
            }
        }

        if (mAnchorMode == AnchorMode::Learning)
        {
            strcpy(sys.sensors.autoState, "LEARN_RET");
        }
        else if (mAnchorMode == AnchorMode::Maintenance)
        {
            strcpy(sys.sensors.autoState, "M_RETURN");
        }
        else
        {
            strcpy(sys.sensors.autoState, "RETURN");
        }

        const float targetBearingDeg = bearingDeg(
            sys.sensors.latitudeDeg,
            sys.sensors.longitudeDeg,
            sys.anchorLatDeg,
            sys.anchorLonDeg);

        float headingError =
            shortestAngleErrorDeg(targetBearingDeg, sys.sensors.motorHeadingDeg);

        if (fabsf(headingError) < AnchorConfig::HEADING_DEADBAND_DEG)
        {
            headingError = 0.0f;
        }

        float steerCmd = headingPid.update(headingError, dtSec);

        out.steerPct = clampf(
            steerCmd,
            Limits::STEER_MIN_PCT,
            Limits::STEER_MAX_PCT);

        float thrustPct = mAnchorLearnedThrustPct;

        if (distAvgM >= fullThrustDistM)
        {
            thrustPct = maxAnchorThrustPct;
        }
        else if (distAvgM > startRadiusM)
        {
            const float denom = fullThrustDistM - startRadiusM;

            if (denom > 0.01f)
            {
                const float t = (distAvgM - startRadiusM) / denom;
                thrustPct =
                    mAnchorLearnedThrustPct +
                    t * (maxAnchorThrustPct - mAnchorLearnedThrustPct);
            }
        }

        const float absHeadingError = fabsf(headingError);

        if (absHeadingError > 90.0f)
        {
            thrustPct = AnchorConfig::MIN_THRUST_PCT;
        }
        else if (absHeadingError > 45.0f)
        {
            thrustPct *= 0.5f;
        }

        out.thrustPct = clampf(
            thrustPct,
            Limits::THRUST_MIN_PCT,
            Limits::THRUST_MAX_PCT);

        return out;
    }

    if (mAnchorMode == AnchorMode::Learning && mDriftStartMs == 0)
    {
        mDriftStartMs = nowMs;
    }

    if (mAnchorMode == AnchorMode::Maintenance)
    {
        strcpy(sys.sensors.autoState, "MAINTAIN");

        const float targetBearingDeg = bearingDeg(
            sys.sensors.latitudeDeg,
            sys.sensors.longitudeDeg,
            sys.anchorLatDeg,
            sys.anchorLonDeg);

        float headingError =
            shortestAngleErrorDeg(targetBearingDeg, sys.sensors.motorHeadingDeg);

        if (fabsf(headingError) < AnchorConfig::HEADING_DEADBAND_DEG)
        {
            headingError = 0.0f;
        }

        float steerCmd = headingPid.update(headingError, dtSec);

        out.steerPct = clampf(
            steerCmd,
            Limits::STEER_MIN_PCT,
            Limits::STEER_MAX_PCT);

        float maintenanceThrust =
            mAnchorLearnedThrustPct * AnchorConfig::MAINTENANCE_FACTOR;

        maintenanceThrust = clampf(
            maintenanceThrust,
            AnchorConfig::MIN_MAINTENANCE_THRUST_PCT,
            AnchorConfig::MAX_MAINTENANCE_THRUST_PCT);

        out.thrustPct = clampf(
            maintenanceThrust,
            Limits::THRUST_MIN_PCT,
            Limits::THRUST_MAX_PCT);

        return out;
    }

    if (mAnchorMode == AnchorMode::Learning)
    {
        strcpy(sys.sensors.autoState, "L_DRIFT");
    }
    else
    {
        strcpy(sys.sensors.autoState, "DRIFT");
    }

    headingPid.reset();
    out.thrustPct = 0.0f;
    out.steerPct = 0.0f;
    return out;
}
