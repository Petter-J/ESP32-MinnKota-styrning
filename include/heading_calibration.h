#pragma once

#include <Arduino.h>
#include "imu_sensor.h"

// 16 punkter = 22.5 grader mellan varje bucket
constexpr uint8_t HEADING_CAL_BUCKET_COUNT = 16;
constexpr float HEADING_CAL_BUCKET_STEP_DEG = 360.0f / HEADING_CAL_BUCKET_COUNT;

// GPS-heading måste ligga inom +/- detta från bucket center
constexpr float HEADING_CAL_BUCKET_WINDOW_DEG = 1.5f;

// Minsta antal samples innan en bucket räknas som användbar
constexpr uint16_t HEADING_CAL_MIN_SAMPLES_PER_BUCKET = 15;

enum class HeadingSweepPhase : uint8_t
{
    None = 0,
    Clockwise = 1,
    CounterClockwise = 2
};

class AngleAverage
{
public:
    void reset();
    void add(float deg);
    bool valid(uint16_t minSamples = 5) const;
    float averageDeg() const;
    uint16_t count() const;

private:
    double _sumSin = 0.0;
    double _sumCos = 0.0;
    uint16_t _count = 0;
};

struct HeadingSweepBucket
{
    AngleAverage gpsCw;
    AngleAverage motorCw;
    AngleAverage boatCw;

    AngleAverage gpsCcw;
    AngleAverage motorCcw;
    AngleAverage boatCcw;

    bool cwValid = false;
    bool ccwValid = false;
};

struct HeadingCalPoint
{
    float gpsDeg = NAN;
    float motorRawDeg = NAN;
    float boatRawDeg = NAN;
    bool valid = false;
};

class HeadingCalibration
{
public:
    static float apply(
        float rawDeg,
        const HeadingCorrectionPoint *table,
        uint8_t count);

    static float bucketCenterDeg(uint8_t bucketIndex);

    static int8_t bucketForGpsHeading(
        float gpsHeadingDeg,
        float windowDeg = HEADING_CAL_BUCKET_WINDOW_DEG);
};