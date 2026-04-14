#pragma once
#include <Arduino.h>
#include "types.h"

class RemoteEspNow
{
public:
    void begin();

    bool update(uint32_t& outButtonMask);
    uint32_t getLatestMask() const;
    bool isAlive(uint32_t nowMs) const;

    void setLatestButtonMask(uint32_t buttonMask, uint32_t rxTimeMs);

    bool sendStatus(const StatusPacket& status);

private:
    bool _initialized = false;
    uint32_t _latestButtonMask = 0;
    bool _hasNewData = false;
    uint32_t _lastRxTimeMs = 0;
};