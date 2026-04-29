#include "remote_calibration.h"

void RemoteCalibration::begin()
{
    _active = false;
    _complete = false;
    _error = false;
    _phase = static_cast<uint8_t>(RemoteCalPhase::None);
    _commandId = 0;

    resetSweepBuckets();

    for (uint8_t i = 0; i < HEADING_CAL_BUCKET_COUNT; i++)
    {
        _boatLut[i].raw = NAN;
        _boatLut[i].corr = NAN;
        _boatLutValid[i] = false;
    }

    _boatLutCount = 0;
}

void RemoteCalibration::startSweep(const CalStartSweepPacket &packet)
{
    _active = true;
    _complete = false;
    _error = false;

    _phase = packet.phase;
    _bucketCount = packet.bucketCount;
    _bucketWindowDeg = packet.bucketWindowDeg;
    _minSamplesPerBucket = packet.minSamplesPerBucket;
    _commandId = packet.commandId;

    if (_bucketCount == 0 || _bucketCount > HEADING_CAL_BUCKET_COUNT)
    {
        _active = false;
        _error = true;
        return;
    }

    resetSweepBuckets();
}

void RemoteCalibration::endSweep(const CalEndSweepPacket &packet)
{
    _active = false;
    _phase = static_cast<uint8_t>(RemoteCalPhase::None);

    _complete = packet.saveToFlash;
}

void RemoteCalibration::update(float boatRawHeadingDeg)
{
    if (!_active)
        return;
    if (!isfinite(boatRawHeadingDeg))
        return;

    // OBS:
    // Remote har ingen GPS-heading.
    // Därför kan Remote inte själv veta bucket utifrån GPS.
    //
    // I nästa steg skickar Main "current bucket" till Remote,
    // eller så låter vi Remote samla kontinuerligt efter separat bucket-kommando.
    //
    // För nu gör update() inget aktivt.
}

bool RemoteCalibration::hasPendingBucketResult() const
{
    return _hasPendingResult;
}

CalBoatBucketResultPacket RemoteCalibration::takePendingBucketResult()
{
    _hasPendingResult = false;
    return _pendingResult;
}

void RemoteCalibration::saveBoatLutPoint(const CalSaveBoatLutPointPacket &packet)
{
    if (packet.lutIndex >= HEADING_CAL_BUCKET_COUNT)
        return;
    if (!packet.valid)
        return;
    if (!isfinite(packet.rawDeg))
        return;
    if (!isfinite(packet.corrDeg))
        return;

    _boatLut[packet.lutIndex].raw = packet.rawDeg;
    _boatLut[packet.lutIndex].corr = packet.corrDeg;
    _boatLutValid[packet.lutIndex] = true;

    _boatLutCount = 0;
    for (uint8_t i = 0; i < HEADING_CAL_BUCKET_COUNT; i++)
    {
        if (_boatLutValid[i])
        {
            _boatLutCount++;
        }
    }
}

bool RemoteCalibration::active() const
{
    return _active;
}

bool RemoteCalibration::complete() const
{
    return _complete;
}

uint8_t RemoteCalibration::phase() const
{
    return _phase;
}

uint16_t RemoteCalibration::bucketsValidMask() const
{
    return _bucketsValidMask;
}

const HeadingCorrectionPoint *RemoteCalibration::boatLut() const
{
    return _boatLut;
}

uint8_t RemoteCalibration::boatLutCount() const
{
    return _boatLutCount;
}

void RemoteCalibration::resetSweepBuckets()
{
    for (uint8_t i = 0; i < HEADING_CAL_BUCKET_COUNT; i++)
    {
        _boatBuckets[i].reset();
        _bucketSent[i] = false;
    }

    _bucketsValidMask = 0;
    _hasPendingResult = false;
}

void RemoteCalibration::updateBucketValidFlags()
{
    _bucketsValidMask = 0;

    for (uint8_t i = 0; i < HEADING_CAL_BUCKET_COUNT; i++)
    {
        if (_boatBuckets[i].valid(_minSamplesPerBucket))
        {
            _bucketsValidMask |= (1 << i);
        }
    }
}

void RemoteCalibration::addBucketSample(
    const CalBucketSamplePacket &packet,
    float boatRawHeadingDeg)
{
    if (!_active)
        return;
    if (!isfinite(boatRawHeadingDeg))
        return;

    if (packet.commandId != _commandId)
        return;
    if (packet.phase != _phase)
        return;
    if (packet.bucketIndex >= _bucketCount)
        return;

    const uint8_t i = packet.bucketIndex;

    if (_bucketSent[i])
        return;

    _boatBuckets[i].add(boatRawHeadingDeg);

    if (_boatBuckets[i].valid(_minSamplesPerBucket))
    {
        _bucketSent[i] = true;
        updateBucketValidFlags();

        _pendingResult = CalBoatBucketResultPacket{};
        _pendingResult.phase = _phase;
        _pendingResult.bucketIndex = i;
        _pendingResult.boatRawAvgDeg = _boatBuckets[i].averageDeg();
        _pendingResult.sampleCount = _boatBuckets[i].count();
        _pendingResult.valid = true;
        _pendingResult.commandId = _commandId;

        _hasPendingResult = true;
    }
}