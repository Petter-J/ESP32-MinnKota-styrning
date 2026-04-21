#pragma once
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "config.h"

struct GpsFix
{
    bool locationValid = false;
    bool speedValid = false;
    bool courseValid = false;

    double latDeg = 0.0;
    double lonDeg = 0.0;
    float speedMps = 0.0f;
    float courseDeg = 0.0f;
    uint8_t satellites = 0;
};

class GpsSensor
{
public:
    bool begin();
    void update(GpsFix& out);

private:
    HardwareSerial _serial{1};
    TinyGPSPlus _gps;
};