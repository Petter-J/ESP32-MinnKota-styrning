#include "calibration_manager.h"

static float circularMean2(float a, float b)
{
    AngleAverage avg;
    avg.add(a);
    avg.add(b);
    return avg.averageDeg();
}

void CalibrationManager::begin()
{
    resetAll();
}

void CalibrationManager::resetAll()
{
    _state = CalibrationManagerState::Idle;
    _phase = HeadingSweepPhase::None;
    _commandId = 0;

    for (uint8_t i = 0; i < HEADING_CAL_BUCKET_COUNT; i++)
    {
        _buckets[i] = HeadingSweepBucket{};

        _finalPoints[i] = HeadingCalPoint{};

        _boatCwReceived[i] = false;
        _boatCcwReceived[i] = false;

        _boatCwAvg[i] = NAN;
        _boatCcwAvg[i] = NAN;
    }

    _mainBucketsValidMask = 0;
    _boatBucketsValidMask = 0;

    _hasPendingBucketSample = false;
    _pendingBucketSample = CalBucketSamplePacket{};
}

void CalibrationManager::resetSweep(
    HeadingSweepPhase phase,
    uint32_t commandId)
{
    _phase = phase;
    _commandId = commandId;

    _hasPendingBucketSample = false;
    _pendingBucketSample = CalBucketSamplePacket{};

    for (uint8_t i = 0; i < HEADING_CAL_BUCKET_COUNT; i++)
    {
        if (phase == HeadingSweepPhase::Clockwise)
        {
            _buckets[i].gpsCw.reset();
            _buckets[i].motorCw.reset();
            _buckets[i].boatCw.reset();
            _buckets[i].cwValid = false;

            _boatCwReceived[i] = false;
            _boatCwAvg[i] = NAN;
        }
        else if (phase == HeadingSweepPhase::CounterClockwise)
        {
            _buckets[i].gpsCcw.reset();
            _buckets[i].motorCcw.reset();
            _buckets[i].boatCcw.reset();
            _buckets[i].ccwValid = false;

            _boatCcwReceived[i] = false;
            _boatCcwAvg[i] = NAN;
        }
    }

    updateValidMasks();
}

void CalibrationManager::startClockwise(uint32_t commandId)
{
    _state = CalibrationManagerState::SweepClockwise;
    resetSweep(HeadingSweepPhase::Clockwise, commandId);
}

void CalibrationManager::startCounterClockwise(uint32_t commandId)
{
    _state = CalibrationManagerState::SweepCounterClockwise;
    resetSweep(HeadingSweepPhase::CounterClockwise, commandId);
}

void CalibrationManager::stop()
{
    resetAll();
}

void CalibrationManager::update(
    float gpsCogDeg,
    float gpsSpeedMs,
    float motorRawHeadingDeg)
{
    if (_state != CalibrationManagerState::SweepClockwise &&
        _state != CalibrationManagerState::SweepCounterClockwise)
    {
        return;
    }

    if (!isfinite(gpsCogDeg) || !isfinite(motorRawHeadingDeg))
        return;

    if (gpsSpeedMs < 1.2f)
        return;

    const int8_t bucket =
        HeadingCalibration::bucketForGpsHeading(gpsCogDeg);

    if (bucket < 0)
        return;

    const uint8_t i = static_cast<uint8_t>(bucket);

    if (_phase == HeadingSweepPhase::Clockwise)
    {
        if (!_buckets[i].cwValid)
        {
            _buckets[i].gpsCw.add(gpsCogDeg);
            _buckets[i].motorCw.add(motorRawHeadingDeg);

            _pendingBucketSample = CalBucketSamplePacket{};
            _pendingBucketSample.phase = static_cast<uint8_t>(RemoteCalPhase::Clockwise);
            _pendingBucketSample.bucketIndex = i;
            _pendingBucketSample.commandId = _commandId;
            _hasPendingBucketSample = true;

            if (_buckets[i].gpsCw.valid(HEADING_CAL_MIN_SAMPLES_PER_BUCKET) &&
                _buckets[i].motorCw.valid(HEADING_CAL_MIN_SAMPLES_PER_BUCKET) &&
                _boatCwReceived[i])
            {
                _buckets[i].cwValid = true;
            }
        }
    }
    else if (_phase == HeadingSweepPhase::CounterClockwise)
    {
        if (!_buckets[i].ccwValid)
        {
            _buckets[i].gpsCcw.add(gpsCogDeg);
            _buckets[i].motorCcw.add(motorRawHeadingDeg);

            _pendingBucketSample = CalBucketSamplePacket{};
            _pendingBucketSample.phase = static_cast<uint8_t>(RemoteCalPhase::CounterClockwise);
            _pendingBucketSample.bucketIndex = i;
            _pendingBucketSample.commandId = _commandId;
            _hasPendingBucketSample = true;

            if (_buckets[i].gpsCcw.valid(HEADING_CAL_MIN_SAMPLES_PER_BUCKET) &&
                _buckets[i].motorCcw.valid(HEADING_CAL_MIN_SAMPLES_PER_BUCKET) &&
                _boatCcwReceived[i])
            {
                _buckets[i].ccwValid = true;
            }
        }
    }

    updateValidMasks();

    if (_state == CalibrationManagerState::SweepClockwise &&
        isClockwiseComplete())
    {
        _state = CalibrationManagerState::Complete;
    }

    if (_state == CalibrationManagerState::SweepCounterClockwise &&
        isCounterClockwiseComplete())
    {
        _state = CalibrationManagerState::BuildFinalLut;
        buildFinalPoints();
        _state = CalibrationManagerState::Complete;
    }
}

bool CalibrationManager::hasPendingBucketSample() const
{
    return _hasPendingBucketSample;
}

CalBucketSamplePacket CalibrationManager::takePendingBucketSample()
{
    _hasPendingBucketSample = false;
    return _pendingBucketSample;
}

void CalibrationManager::handleBoatBucketResult(
    const CalBoatBucketResultPacket &packet)
{
    if (packet.commandId != _commandId)
        return;

    if (!packet.valid)
        return;

    if (packet.bucketIndex >= HEADING_CAL_BUCKET_COUNT)
        return;

    if (!isfinite(packet.boatRawAvgDeg))
        return;

    const uint8_t i = packet.bucketIndex;

    if (packet.phase == static_cast<uint8_t>(RemoteCalPhase::Clockwise))
    {
        _boatCwReceived[i] = true;
        _boatCwAvg[i] = packet.boatRawAvgDeg;

        if (_buckets[i].gpsCw.valid(HEADING_CAL_MIN_SAMPLES_PER_BUCKET) &&
            _buckets[i].motorCw.valid(HEADING_CAL_MIN_SAMPLES_PER_BUCKET))
        {
            _buckets[i].cwValid = true;
        }
    }
    else if (packet.phase == static_cast<uint8_t>(RemoteCalPhase::CounterClockwise))
    {
        _boatCcwReceived[i] = true;
        _boatCcwAvg[i] = packet.boatRawAvgDeg;

        if (_buckets[i].gpsCcw.valid(HEADING_CAL_MIN_SAMPLES_PER_BUCKET) &&
            _buckets[i].motorCcw.valid(HEADING_CAL_MIN_SAMPLES_PER_BUCKET))
        {
            _buckets[i].ccwValid = true;
        }
    }

    updateValidMasks();
}

bool CalibrationManager::phaseComplete(HeadingSweepPhase phase) const
{
    for (uint8_t i = 0; i < HEADING_CAL_BUCKET_COUNT; i++)
    {
        if (phase == HeadingSweepPhase::Clockwise)
        {
            if (!_buckets[i].cwValid)
                return false;
        }
        else if (phase == HeadingSweepPhase::CounterClockwise)
        {
            if (!_buckets[i].ccwValid)
                return false;
        }
    }

    return true;
}

bool CalibrationManager::isClockwiseComplete() const
{
    return phaseComplete(HeadingSweepPhase::Clockwise);
}

bool CalibrationManager::isCounterClockwiseComplete() const
{
    return phaseComplete(HeadingSweepPhase::CounterClockwise);
}

bool CalibrationManager::isComplete() const
{
    return _state == CalibrationManagerState::Complete;
}

bool CalibrationManager::active() const
{
    return _state == CalibrationManagerState::SweepClockwise ||
           _state == CalibrationManagerState::SweepCounterClockwise ||
           _state == CalibrationManagerState::BuildFinalLut;
}

void CalibrationManager::updateValidMasks()
{
    _mainBucketsValidMask = 0;
    _boatBucketsValidMask = 0;

    for (uint8_t i = 0; i < HEADING_CAL_BUCKET_COUNT; i++)
    {
        if (_buckets[i].cwValid || _buckets[i].ccwValid)
        {
            _mainBucketsValidMask |= (1 << i);
        }

        if (_boatCwReceived[i] || _boatCcwReceived[i])
        {
            _boatBucketsValidMask |= (1 << i);
        }
    }
}

uint16_t CalibrationManager::mainBucketsValidMask() const
{
    return _mainBucketsValidMask;
}

uint16_t CalibrationManager::boatBucketsValidMask() const
{
    return _boatBucketsValidMask;
}

void CalibrationManager::buildFinalPoints()
{
    for (uint8_t i = 0; i < HEADING_CAL_BUCKET_COUNT; i++)
    {
        HeadingCalPoint &p = _finalPoints[i];

        if (!_buckets[i].cwValid || !_buckets[i].ccwValid)
        {
            p.valid = false;
            continue;
        }

        if (!_boatCwReceived[i] || !_boatCcwReceived[i])
        {
            p.valid = false;
            continue;
        }

        p.gpsDeg = circularMean2(
            _buckets[i].gpsCw.averageDeg(),
            _buckets[i].gpsCcw.averageDeg());

        p.motorRawDeg = circularMean2(
            _buckets[i].motorCw.averageDeg(),
            _buckets[i].motorCcw.averageDeg());

        p.boatRawDeg = circularMean2(
            _boatCwAvg[i],
            _boatCcwAvg[i]);

        p.valid =
            isfinite(p.gpsDeg) &&
            isfinite(p.motorRawDeg) &&
            isfinite(p.boatRawDeg);
    }
}

const HeadingCalPoint *CalibrationManager::finalPoints() const
{
    return _finalPoints;
}