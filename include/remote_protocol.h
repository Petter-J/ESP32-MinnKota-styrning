#pragma once
#include <Arduino.h>

struct RemotePacket
{
    uint32_t buttonMask = 0;

    // Fast BNO085 i båten / remote-enheten
    uint16_t boatHeadingDeg10 = 0; // grader * 10
    uint8_t boatFlags = 0;
};

struct StatusPacket
{
    uint8_t mode = 0;
    uint8_t manualThrustPct = 0;     // 0..100
    uint8_t targetSpeedPct = 0;      // 0..100
    uint16_t headingDeg10 = 0;       // 0..3600  (grader * 10)
    uint16_t targetHeadingDeg10 = 0; // 0..3600  (grader * 10)
    uint8_t satellites = 0;
    uint8_t flags = 0;     // bit0 = gpsValid
    int8_t steerState = 0; // -1 = vänster, 0 = ingen, 1 = höger
    uint8_t counter = 0;
    uint8_t calFlags = 0;       // bit0 = active, bit1 = complete
    uint16_t calBucketMask = 0; // bit 0..15 = bucket klar
    uint8_t calPhase = 0;       // 0=none, 1=CW, 2=CCW
    uint16_t gpsSpeedCmps = 0;  // GPS speed * 100
    uint16_t gpsCogDeg10 = 0;   // GPS COG * 10
};

static constexpr uint8_t STATUS_FLAG_GPS_VALID = 1 << 0;
static constexpr uint8_t REMOTE_FLAG_BOAT_IMU_VALID = 1 << 0;
static constexpr uint8_t STATUS_CAL_FLAG_ACTIVE = 1 << 0;
static constexpr uint8_t STATUS_CAL_FLAG_COMPLETE = 1 << 1;

// ------------------------------------------------------------
// Calibration ESP-NOW packets
// ------------------------------------------------------------
//
// Viktigt:
// RemotePacket saknar msgType och behålls oförändrad.
// Därför ska mottagaren senare skilja paket på len:
//
// if (len == sizeof(RemotePacket)) ...
// if (len == sizeof(CalStartSweepPacket)) ...
// if (len == sizeof(CalBoatBucketResultPacket)) ...
//

enum class RemoteMsgType : uint8_t
{
    NormalRemotePacket = 1,

    CalStartSweep = 20,
    CalBoatBucketResult = 21,
    CalSaveBoatLutPoint = 22,
    CalEndSweep = 23,
    CalStatus = 24,
    CalBucketSample = 25
};

enum class RemoteCalPhase : uint8_t
{
    None = 0,
    Clockwise = 1,
    CounterClockwise = 2
};

struct CalStartSweepPacket
{
    uint8_t msgType = static_cast<uint8_t>(RemoteMsgType::CalStartSweep);
    uint8_t version = 1;

    // RemoteCalPhase: 1 = Clockwise, 2 = CounterClockwise
    uint8_t phase = static_cast<uint8_t>(RemoteCalPhase::None);

    uint8_t bucketCount = 16;
    float bucketWindowDeg = 1.5f;
    uint16_t minSamplesPerBucket = 15;

    uint32_t commandId = 0;
};

struct CalBoatBucketResultPacket
{
    uint8_t msgType = static_cast<uint8_t>(RemoteMsgType::CalBoatBucketResult);
    uint8_t version = 1;

    // RemoteCalPhase: 1 = Clockwise, 2 = CounterClockwise
    uint8_t phase = static_cast<uint8_t>(RemoteCalPhase::None);

    uint8_t bucketIndex = 0; // 0..15

    float boatRawAvgDeg = NAN;
    uint16_t sampleCount = 0;

    bool valid = false;
    uint32_t commandId = 0;
};

struct CalSaveBoatLutPointPacket
{
    uint8_t msgType = static_cast<uint8_t>(RemoteMsgType::CalSaveBoatLutPoint);
    uint8_t version = 1;

    uint8_t lutIndex = 0; // 0..15

    float rawDeg = NAN;
    float corrDeg = NAN;

    bool valid = false;
    uint32_t commandId = 0;
};

struct CalEndSweepPacket
{
    uint8_t msgType = static_cast<uint8_t>(RemoteMsgType::CalEndSweep);
    uint8_t version = 1;

    bool saveToFlash = false;
    uint32_t commandId = 0;
};

struct CalStatusPacket
{
    uint8_t msgType = static_cast<uint8_t>(RemoteMsgType::CalStatus);
    uint8_t version = 1;

    uint8_t phase = static_cast<uint8_t>(RemoteCalPhase::None);

    // Bit 0..15 motsvarar bucket 0..15
    uint16_t bucketsValidMask = 0;

    bool active = false;
    bool complete = false;
    bool error = false;

    uint32_t commandId = 0;
};

struct CalBucketSamplePacket
{
    uint8_t msgType = static_cast<uint8_t>(RemoteMsgType::CalBucketSample);
    uint8_t version = 1;

    // RemoteCalPhase: 1 = Clockwise, 2 = CounterClockwise
    uint8_t phase = static_cast<uint8_t>(RemoteCalPhase::None);

    uint8_t bucketIndex = 0; // 0..15

    uint32_t commandId = 0;
};