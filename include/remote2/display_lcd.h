#pragma once

#include <stdint.h>
#include "remote_protocol.h"

void display_lcd_begin();
void display_lcd_update(const StatusPacket& status, bool hasStatus, uint32_t buttonMask, bool linkAlive);