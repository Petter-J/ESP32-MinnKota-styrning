#pragma once

#include <Arduino.h>
#include "heading_calibration.h"
#include "remote_protocol.h"

enum class CalibrationManagerState : uint8_t
{
    Idle = 0,
    SweepClockwise = 1,
    SweepCounterClockwise = 2,
    BuildFinalLut = 3,
    Complete = 4,
    Error = 5
};

class CalibrationManager
{
public:
    void begin();

    void startClockwise(uint32_t commandId);
    void startCounterClockwise(uint32_t commandId);
    void stop();

    void update(
        float gpsCogDeg,
        float gpsSpeedMs,
        float motorRawHeadingDeg);

    bool hasPendingBucketSample() const;
    CalBucketSamplePacket takePendingBucketSample();

    void handleBoatBucketResult(const CalBoatBucketResultPacket &packet);

    bool isClockwiseComplete() const;
    bool isCounterClockwiseComplete() const;
    bool isComplete() const;
    bool active() const;

    uint16_t mainBucketsValidMask() const;
    uint16_t boatBucketsValidMask() const;

    const HeadingCalPoint *finalPoints() const;

private:
    void resetSweep(HeadingSweepPhase phase, uint32_t commandId);
    void resetAll();

    bool phaseComplete(HeadingSweepPhase phase) const;
    void updateValidMasks();

    void buildFinalPoints();

    CalibrationManagerState _state = CalibrationManagerState::Idle;
    HeadingSweepPhase _phase = HeadingSweepPhase::None;

    uint32_t _commandId = 0;

    HeadingSweepBucket _buckets[HEADING_CAL_BUCKET_COUNT];
    HeadingCalPoint _finalPoints[HEADING_CAL_BUCKET_COUNT];

    bool _boatCwReceived[HEADING_CAL_BUCKET_COUNT];
    bool _boatCcwReceived[HEADING_CAL_BUCKET_COUNT];

    float _boatCwAvg[HEADING_CAL_BUCKET_COUNT];
    float _boatCcwAvg[HEADING_CAL_BUCKET_COUNT];

    uint16_t _mainBucketsValidMask = 0;
    uint16_t _boatBucketsValidMask = 0;

    bool _hasPendingBucketSample = false;
    CalBucketSamplePacket _pendingBucketSample;
};