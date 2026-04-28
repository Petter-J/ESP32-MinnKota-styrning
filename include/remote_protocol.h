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
    uint8_t flags = 0;               // bit0 = gpsValid
    int8_t steerState = 0;           // -1 = vänster, 0 = ingen, 1 = höger
    uint8_t counter = 0;
};

static constexpr uint8_t STATUS_FLAG_GPS_VALID = 1 << 0;
static constexpr uint8_t REMOTE_FLAG_BOAT_IMU_VALID = 1 << 0;