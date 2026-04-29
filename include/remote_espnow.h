#pragma once

#include "types.h"

enum class RemoteTarget : uint8_t
{
    Remote1BoatBno = 1,
    Remote2Handheld = 2,
    Both = 3
};

class RemoteEspNow
{
public:
    void begin();

    bool sendToTarget(RemoteTarget target, const uint8_t *data, size_t len);

    bool sendStatusRemote1(const StatusPacket &status);
    bool sendStatusRemote2(const StatusPacket &status);

    bool sendStatus(const StatusPacket &status);

    bool sendCalibrationPacket(const uint8_t *data, size_t len);
    bool getBoatCalibrationResult(CalBoatBucketResultPacket &outPacket);

    uint32_t getCombinedMask(uint32_t nowMs) const;
    uint32_t lastRxTimeMs() const;

    bool getBoatHeading(float &headingDeg, uint32_t nowMs) const;
    bool hasBoatImu(uint32_t nowMs) const;

private:
    friend void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len);

    void setRemote1Mask(uint32_t buttonMask, uint32_t rxTimeMs);
    void setRemote2Mask(uint32_t buttonMask, uint32_t rxTimeMs);

    bool _initialized = false;

    uint32_t _remote1Mask = 0;
    uint32_t _remote2Mask = 0;

    uint32_t _remote1LastRxTimeMs = 0;
    uint32_t _remote2LastRxTimeMs = 0;

    float _boatHeadingDeg = 0.0f;
    bool _boatImuValid = false;
    uint32_t _boatImuLastRxTimeMs = 0;

    bool _hasBoatCalibrationResult = false;
    CalBoatBucketResultPacket _boatCalibrationResult;
};