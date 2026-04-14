#pragma once
#include <Arduino.h>

struct RemotePacket
{
    uint32_t buttonMask = 0;
};

struct StatusPacket
{
    uint8_t mode = 0;

    float manualThrustPct = 0.0f;
    float targetSpeedPct = 0.0f;

    float headingDeg = 0.0f;
    float targetHeadingDeg = 0.0f;

    uint8_t satellites = 0;
    bool gpsValid = false;

    uint32_t counter = 0;
};