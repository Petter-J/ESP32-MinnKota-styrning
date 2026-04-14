#pragma once
#include <Arduino.h>
#include "types.h"
#include "buttons.h"

struct DisplayLines
{
    String line1;
    String line2;
    String line3;
    String line4;
};

DisplayLines buildDisplayLines(const SystemState& sys, uint32_t buttonMask, bool linkAlive);