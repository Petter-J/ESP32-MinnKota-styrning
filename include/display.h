#pragma once
#include <Arduino.h>
#include "remote_protocol.h"

bool display_is_available();
void display_begin();

void display_update(
    const StatusPacket &status,
    bool hasStatus,
    uint32_t buttonMask,
    bool linkAlive,
    bool calActive = false,
    bool calComplete = false,
    uint16_t calBucketMask = 0,
    uint8_t calPhase = 0);

void display_set_brightness(uint8_t value);