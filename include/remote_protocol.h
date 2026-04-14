#pragma once
#include <Arduino.h>

struct RemotePacket
{
    uint32_t buttonMask = 0;
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
    uint8_t counter = 0;
};

static constexpr uint8_t STATUS_FLAG_GPS_VALID = 1 << 0;