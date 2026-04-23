#pragma once

#include "types.h"

class RemoteEspNow
{
public:
    void begin();

    // Nytt: skicka separat
    bool sendStatusRemote1(const StatusPacket &status);
    bool sendStatusRemote2(const StatusPacket &status);

    // Behåll för bakåtkompabilitet: skickar till båda (via metoderna ovan)
    bool sendStatus(const StatusPacket &status);

    uint32_t getCombinedMask(uint32_t nowMs) const;

private:
    friend void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len);

    void setRemote1Mask(uint32_t buttonMask, uint32_t rxTimeMs);
    void setRemote2Mask(uint32_t buttonMask, uint32_t rxTimeMs);

    bool _initialized = false;

    uint32_t _remote1Mask = 0;
    uint32_t _remote2Mask = 0;

    uint32_t _remote1LastRxTimeMs = 0;
    uint32_t _remote2LastRxTimeMs = 0;
};