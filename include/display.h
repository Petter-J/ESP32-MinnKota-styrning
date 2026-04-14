#pragma once
#include <Arduino.h>
#include "remote_protocol.h"

bool display_is_available();
void display_begin();
void display_update(const StatusPacket& status, bool hasStatus, uint32_t buttonMask, bool linkAlive);
void display_set_brightness(uint8_t value);