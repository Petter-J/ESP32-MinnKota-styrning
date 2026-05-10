#pragma once

#include "types.h"

class PidController;

enum class AnchorMode
{
    Learning,
    OnOff,
    Maintenance
};

class AnchorController
{
public:
    void onEnter(SystemState &sys);
    void onExit();
    ActuatorCommand update(float dtSec, SystemState &sys, PidController &headingPid);

private:
    static constexpr uint8_t GPS_AVG_COUNT = 8;

    double mLatBuf[GPS_AVG_COUNT] = {};
    double mLonBuf[GPS_AVG_COUNT] = {};
    uint8_t mGpsIndex = 0;
    uint8_t mGpsCount = 0;

    AnchorMode mAnchorMode = AnchorMode::Learning;

    bool mWasInsideRadius = true;

    uint32_t mOutsideSinceMs = 0;
    uint32_t mReturnStartMs = 0;

    uint32_t mDriftStartMs = 0;
    uint32_t mDriftTimeSumMs = 0;
    uint8_t mDriftSamples = 0;

    float mAnchorLearnedThrustPct = 5.0f;

    void resetGpsAverage();

    static float degToRad(float deg);
    static float radToDeg(float rad);
    static float distanceMeters(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg);
    static float bearingDeg(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg);
};