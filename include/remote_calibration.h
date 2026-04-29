#pragma once

#include <Arduino.h>
#include "heading_calibration.h"
#include "remote_protocol.h"

class RemoteCalibration
{
public:
    void begin();

    void startSweep(const CalStartSweepPacket &packet);
    void endSweep(const CalEndSweepPacket &packet);

    void update(float boatRawHeadingDeg);

    bool hasPendingBucketResult() const;
    CalBoatBucketResultPacket takePendingBucketResult();

    void saveBoatLutPoint(const CalSaveBoatLutPointPacket &packet);
    void addBucketSample(const CalBucketSamplePacket &packet, float boatRawHeadingDeg);

    bool active() const;
    bool complete() const;
    uint8_t phase() const;
    uint16_t bucketsValidMask() const;

    const HeadingCorrectionPoint *boatLut() const;
    uint8_t boatLutCount() const;

private:
    void resetSweepBuckets();
    void updateBucketValidFlags();

    bool _active = false;
    bool _complete = false;
    bool _error = false;

    uint8_t _phase = static_cast<uint8_t>(RemoteCalPhase::None);

    uint8_t _bucketCount = HEADING_CAL_BUCKET_COUNT;
    float _bucketWindowDeg = HEADING_CAL_BUCKET_WINDOW_DEG;
    uint16_t _minSamplesPerBucket = HEADING_CAL_MIN_SAMPLES_PER_BUCKET;

    uint32_t _commandId = 0;

    AngleAverage _boatBuckets[HEADING_CAL_BUCKET_COUNT];
    bool _bucketSent[HEADING_CAL_BUCKET_COUNT];

    uint16_t _bucketsValidMask = 0;

    bool _hasPendingResult = false;
    CalBoatBucketResultPacket _pendingResult;

    HeadingCorrectionPoint _boatLut[HEADING_CAL_BUCKET_COUNT];
    bool _boatLutValid[HEADING_CAL_BUCKET_COUNT];
    uint8_t _boatLutCount = 0;
};