#pragma once

#include <stdint.h>
#include "remote_protocol.h"

bool remote2_espnow_begin(const uint8_t* peerMac);
bool remote2_espnow_update(StatusPacket& outStatus, bool& outHasStatus, uint32_t now);