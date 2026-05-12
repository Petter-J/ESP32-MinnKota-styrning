#pragma once

#include <stdint.h>
#include "types.h"

struct SystemState;
struct ActuatorCommand;
class PidController;

class AnchorController
{
public:
    void onEnter(SystemState &sys);
    void onExit();
    ActuatorCommand update(float dtSec, SystemState &sys, PidController &headingPid);

private:
    static constexpr uint8_t GPS_AVG_COUNT = 6;
    static constexpr uint8_t START_CONFIRM_COUNT = 3;
    static constexpr uint8_t STOP_CONFIRM_COUNT = 3;
    static constexpr uint8_t DRIFT_VECTOR_MAX_SAMPLES = 16;

    enum class AnchorMode : uint8_t
    {
        Learning = 0,
        Maintenance
    };

    enum class DriftCorrectionState : uint8_t
    {
        Neutral = 0,
        RecoverFromBack,
        RecoverFromFront
    };

    static float degToRad(float deg);
    static float radToDeg(float rad);
    static float distanceMeters(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg);
    static float bearingDeg(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg);
    

    void resetGpsAverage();
    void resetDriftLearning();
    void addDriftVectorSample(SystemState &sys);
    void finalizeDriftLineIfPossible();
    bool projectToDriftLineMeters(
        double latDeg,
        double lonDeg,
        double anchorLatDeg,
        double anchorLonDeg,
        float &alongDriftM,
        float &crossDriftM) const;

private:
    double mLatBuf[GPS_AVG_COUNT]{};
    double mLonBuf[GPS_AVG_COUNT]{};
    uint8_t mGpsIndex = 0;
    uint8_t mGpsCount = 0;

    bool mWasInsideRadius = true;
    uint32_t mOutsideSinceMs = 0;
    uint32_t mReturnStartMs = 0;

    uint32_t mDriftStartMs = 0;
    uint32_t mDriftTimeSumMs = 0;
    uint8_t mDriftSamples = 0;

    uint8_t mStartZoneHits = 0;
    uint8_t mDriftZoneHits = 0;
    uint8_t mStopZoneHits = 0;

    AnchorMode mAnchorMode = AnchorMode::Learning;

    float mAnchorLearnedThrustPct = 0.0f;
    float mBaseThrustPct = 0.0f;

    float mDriftVecNorthSumM = 0.0f;
    float mDriftVecEastSumM = 0.0f;
    uint8_t mDriftVectorSamples = 0;
    float mDriftLineBearingDeg = 0.0f;
    float mDriftLineUnitNorth = 1.0f;
    float mDriftLineUnitEast = 0.0f;
    bool mHasDriftLine = false;

    DriftCorrectionState mCorrectionState = DriftCorrectionState::Neutral;
    uint32_t mLastBaseAdjustMs = 0;
};