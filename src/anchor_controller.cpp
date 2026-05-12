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

void AnchorController::resetDriftLearning()
{
    mDriftVecNorthSumM = 0.0f;
    mDriftVecEastSumM = 0.0f;
    mDriftVectorSamples = 0;
    mDriftLineBearingDeg = 0.0f;
    mDriftLineUnitNorth = 1.0f;
    mDriftLineUnitEast = 0.0f;
    mHasDriftLine = false;
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

void AnchorController::addDriftVectorSample(SystemState &sys)
{
    if (mDriftVectorSamples >= DRIFT_VECTOR_MAX_SAMPLES)
    {
        return;
    }

    const float anchorLatRad = degToRad((float)sys.anchorLatDeg);
    const float latScaleM = 111320.0f;
    const float lonScaleM = 111320.0f * cosf(anchorLatRad);

    const float northM =
        (float)((sys.sensors.latitudeDeg - sys.anchorLatDeg) * latScaleM);

    const float eastM =
        (float)((sys.sensors.longitudeDeg - sys.anchorLonDeg) * lonScaleM);

    const float vecLenM = sqrtf(northM * northM + eastM * eastM);

    if (vecLenM < 0.1f)
    {
        return;
    }

    mDriftVecNorthSumM += northM;
    mDriftVecEastSumM += eastM;
    mDriftVectorSamples++;
}

void AnchorController::finalizeDriftLineIfPossible()
{
    if (mDriftVectorSamples == 0)
    {
        return;
    }

    const float avgNorthM = mDriftVecNorthSumM / (float)mDriftVectorSamples;
    const float avgEastM = mDriftVecEastSumM / (float)mDriftVectorSamples;

    const float len = sqrtf(avgNorthM * avgNorthM + avgEastM * avgEastM);

    if (len < 0.1f)
    {
        return;
    }

    mDriftLineUnitNorth = avgNorthM / len;
    mDriftLineUnitEast = avgEastM / len;
    mDriftLineBearingDeg = wrap360(radToDeg(atan2f(mDriftLineUnitEast, mDriftLineUnitNorth)));
    mHasDriftLine = true;
}

bool AnchorController::projectToDriftLineMeters(
    double latDeg,
    double lonDeg,
    double anchorLatDeg,
    double anchorLonDeg,
    float &alongDriftM,
    float &crossDriftM) const
{
    if (!mHasDriftLine)
    {
        alongDriftM = 0.0f;
        crossDriftM = 0.0f;
        return false;
    }

    const float anchorLatRad = degToRad((float)anchorLatDeg);
    const float latScaleM = 111320.0f;
    const float lonScaleM = 111320.0f * cosf(anchorLatRad);

    const float northM = (float)((latDeg - anchorLatDeg) * latScaleM);
    const float eastM = (float)((lonDeg - anchorLonDeg) * lonScaleM);

    alongDriftM =
        northM * mDriftLineUnitNorth +
        eastM * mDriftLineUnitEast;

    crossDriftM =
        -northM * mDriftLineUnitEast +
        eastM * mDriftLineUnitNorth;

    return true;
}

void AnchorController::onEnter(SystemState &sys)
{
    resetGpsAverage();
    resetDriftLearning();

    mWasInsideRadius = true;
    mOutsideSinceMs = 0;
    mReturnStartMs = 0;

    mDriftStartMs = 0;
    mDriftTimeSumMs = 0;
    mDriftSamples = 0;

    mStartZoneHits = 0;
    mDriftZoneHits = 0;
    mStopZoneHits = 0;

    mAnchorMode = AnchorMode::Learning;
    mAnchorLearnedThrustPct = clampAnchorThrust(AnchorConfig::START_THRUST_PCT);
    mBaseThrustPct = mAnchorLearnedThrustPct;
    mCorrectionState = DriftCorrectionState::Neutral;
    mLastBaseAdjustMs = 0;

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
    resetDriftLearning();

    mWasInsideRadius = true;
    mOutsideSinceMs = 0;
    mReturnStartMs = 0;

    mDriftStartMs = 0;
    mDriftTimeSumMs = 0;
    mDriftSamples = 0;

    mStartZoneHits = 0;
    mDriftZoneHits = 0;
    mStopZoneHits = 0;

    mAnchorMode = AnchorMode::Learning;
    mAnchorLearnedThrustPct = clampAnchorThrust(AnchorConfig::START_THRUST_PCT);
    mBaseThrustPct = mAnchorLearnedThrustPct;
    mCorrectionState = DriftCorrectionState::Neutral;
    mLastBaseAdjustMs = 0;
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

    mLatBuf[mGpsIndex] = sys.sensors.latitudeDeg;
    mLonBuf[mGpsIndex] = sys.sensors.longitudeDeg;

    mGpsIndex = (mGpsIndex + 1) % GPS_AVG_COUNT;

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

    const bool inZone3 = (distRawM <= stopRadiusM);
    const bool inZone5 = (distRawM >= startRadiusM);
    const bool inZone4 = !inZone3 && !inZone5;

    if (mWasInsideRadius)
    {
        if (inZone4 && mDriftZoneHits < START_CONFIRM_COUNT)
        {
            mDriftZoneHits++;
        }

        if (mDriftZoneHits >= START_CONFIRM_COUNT)
        {
            if (inZone3)
            {
                mDriftZoneHits = 0;
            }
            else if (inZone5 && mStartZoneHits < START_CONFIRM_COUNT)
            {
                mStartZoneHits++;
            }
        }

        mStopZoneHits = 0;
    }
    else
    {
        if (inZone4 && mDriftZoneHits < START_CONFIRM_COUNT)
        {
            mDriftZoneHits++;
        }

        if (mDriftZoneHits >= START_CONFIRM_COUNT)
        {
            if (inZone5)
            {
                mDriftZoneHits = 0;
            }
            else if (inZone3 && mStopZoneHits < STOP_CONFIRM_COUNT)
            {
                mStopZoneHits++;
            }
        }

        mStartZoneHits = 0;
    }

    const bool leftZone3Confirmed =
        (mWasInsideRadius && (mDriftZoneHits >= START_CONFIRM_COUNT));

    const bool reachedZone5Confirmed =
        (mWasInsideRadius && (mStartZoneHits >= START_CONFIRM_COUNT));

    const bool leftZone5Confirmed =
        (!mWasInsideRadius && (mDriftZoneHits >= START_CONFIRM_COUNT));

    const bool reachedZone3Confirmed =
        (!mWasInsideRadius && (mStopZoneHits >= STOP_CONFIRM_COUNT));

    if (mWasInsideRadius && !leftZone3Confirmed)
    {
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
            headingPid.reset();
            out.thrustPct = 0.0f;
            out.steerPct = 0.0f;
            return out;
        }

        strcpy(sys.sensors.autoState, "M_HOLD");
    }

    if (mWasInsideRadius && leftZone3Confirmed && !reachedZone5Confirmed)
    {
        mOutsideSinceMs = 0;
        mReturnStartMs = 0;
        mStopZoneHits = 0;

        if (mAnchorMode == AnchorMode::Learning)
        {
            strcpy(sys.sensors.autoState, "L_DRIFT");
            headingPid.reset();
            out.thrustPct = 0.0f;
            out.steerPct = 0.0f;
            return out;
        }

        strcpy(sys.sensors.autoState, "M_DRIFT");
    }

    if (reachedZone5Confirmed)
    {
        mWasInsideRadius = false;
        mOutsideSinceMs = nowMs;
        mReturnStartMs = nowMs;

        mStartZoneHits = 0;
        mDriftZoneHits = 0;
        mStopZoneHits = 0;

        if (mAnchorMode == AnchorMode::Learning && mDriftStartMs != 0)
        {
            const uint32_t driftTimeMs = nowMs - mDriftStartMs;

            mDriftTimeSumMs += driftTimeMs;

            if (mDriftSamples < 255)
            {
                mDriftSamples++;
            }

            addDriftVectorSample(sys);
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
                finalizeDriftLineIfPossible();
                mAnchorMode = AnchorMode::Maintenance;
                mBaseThrustPct = mAnchorLearnedThrustPct;
                mCorrectionState = DriftCorrectionState::Neutral;
                mDriftStartMs = 0;
            }
        }
    }

    if (!mWasInsideRadius && reachedZone3Confirmed)
    {
        if (mReturnStartMs != 0)
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
        mDriftZoneHits = 0;
        mStopZoneHits = 0;

        if (mAnchorMode == AnchorMode::Learning && mDriftStartMs == 0)
        {
            mDriftStartMs = nowMs;
        }

        if (mAnchorMode == AnchorMode::Learning)
        {
            strcpy(sys.sensors.autoState, "L_HOLD");
            headingPid.reset();
            out.thrustPct = 0.0f;
            out.steerPct = 0.0f;
            return out;
        }

        strcpy(sys.sensors.autoState, "M_HOLD");
        mCorrectionState = DriftCorrectionState::Neutral;
    }

    float alongDriftM = 0.0f;
    float crossDriftM = 0.0f;
    const bool hasProjection = projectToDriftLineMeters(
        avgLat,
        avgLon,
        sys.anchorLatDeg,
        sys.anchorLonDeg,
        alongDriftM,
        crossDriftM);

    if (mAnchorMode == AnchorMode::Maintenance && hasProjection)
    {
        const bool canAdjustBase =
            (mLastBaseAdjustMs == 0) ||
            ((nowMs - mLastBaseAdjustMs) >= AnchorConfig::BASE_THRUST_ADJUST_INTERVAL_MS);

        if (alongDriftM >= AnchorConfig::DRIFT_BACK_ZONE_M)
        {
            if (canAdjustBase)
            {
                mBaseThrustPct += AnchorConfig::BASE_THRUST_ADJUST_STEP_PCT;
                mBaseThrustPct = clampAnchorThrust(mBaseThrustPct);
                mLastBaseAdjustMs = nowMs;
            }

            mCorrectionState = DriftCorrectionState::RecoverFromBack;
        }
        else if (alongDriftM <= AnchorConfig::DRIFT_FRONT_STOP_LINE_M)
        {
            if (canAdjustBase)
            {
                mBaseThrustPct -= AnchorConfig::BASE_THRUST_ADJUST_STEP_PCT;
                mBaseThrustPct = clampAnchorThrust(mBaseThrustPct);
                mLastBaseAdjustMs = nowMs;
            }

            mCorrectionState = DriftCorrectionState::RecoverFromFront;
        }
        else if (distAvgM <= stopRadiusM)
        {
            mCorrectionState = DriftCorrectionState::Neutral;
        }
    }

    if (mAnchorMode == AnchorMode::Learning)
    {
        if (!mWasInsideRadius && leftZone5Confirmed)
        {
            strcpy(sys.sensors.autoState, "L_DRIFT");
        }
        else
        {
            strcpy(sys.sensors.autoState, "LEARN_RET");
        }
    }
    else
    {
        if (mCorrectionState == DriftCorrectionState::RecoverFromBack)
        {
            strcpy(sys.sensors.autoState, "M_BACK");
        }
        else if (mCorrectionState == DriftCorrectionState::RecoverFromFront)
        {
            strcpy(sys.sensors.autoState, "M_FRONT");
        }
        else
        {
            strcpy(sys.sensors.autoState, "M_RETURN");
        }
    }

    const float targetBearingDeg = bearingDeg(
        avgLat,
        avgLon,
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

    float thrustPct = 0.0f;

    if (mAnchorMode == AnchorMode::Learning)
    {
        thrustPct = mAnchorLearnedThrustPct;

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
    }
    else
    {
        thrustPct = mBaseThrustPct;

        if (mCorrectionState == DriftCorrectionState::RecoverFromBack)
        {
            thrustPct = mBaseThrustPct + AnchorConfig::CORRECTION_THRUST_OFFSET_PCT;
        }
        else if (mCorrectionState == DriftCorrectionState::RecoverFromFront)
        {
            thrustPct = mBaseThrustPct - AnchorConfig::CORRECTION_THRUST_OFFSET_PCT;
        }

        if (distAvgM >= fullThrustDistM)
        {
            thrustPct = maxAnchorThrustPct;
        }
        else if (distAvgM > startRadiusM && mCorrectionState != DriftCorrectionState::RecoverFromFront)
        {
            const float denom = fullThrustDistM - startRadiusM;

            if (denom > 0.01f)
            {
                const float t = (distAvgM - startRadiusM) / denom;
                const float scaled =
                    mBaseThrustPct +
                    t * (maxAnchorThrustPct - mBaseThrustPct);

                if (scaled > thrustPct)
                {
                    thrustPct = scaled;
                }
            }
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