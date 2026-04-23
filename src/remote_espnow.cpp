#include "remote_espnow.h"
#include "config.h"
#include <WiFi.h>
#include <esp_now.h>
#include <cstring>

// ------------------------------------------------------------
// Static instance pointer for ESP-NOW callback
// ------------------------------------------------------------
static RemoteEspNow* s_instance = nullptr;

// ------------------------------------------------------------
// MAC addresses of remotes
// ------------------------------------------------------------
static uint8_t s_remote1PeerMac[6] = { 0xF0, 0xF5, 0xBD, 0x73, 0x87, 0x28 }; // gamla remote
static uint8_t s_remote2PeerMac[6] = { 0x20, 0x6E, 0xF1, 0x9B, 0xB3, 0x08 }; // remote2 S3

// ------------------------------------------------------------
// ESP-NOW receive callback
// ------------------------------------------------------------
void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len)
{
    if (s_instance == nullptr)
        return;

    if (mac == nullptr || data == nullptr || len != (int)sizeof(RemotePacket))
        return;

    RemotePacket pkt;
    memcpy(&pkt, data, sizeof(RemotePacket));

    const uint32_t now = millis();

    if (memcmp(mac, s_remote1PeerMac, 6) == 0)
    {
        s_instance->setRemote1Mask(pkt.buttonMask, now);
    }
    else if (memcmp(mac, s_remote2PeerMac, 6) == 0)
    {
        s_instance->setRemote2Mask(pkt.buttonMask, now);
    }
}

// ------------------------------------------------------------
// RemoteEspNow
// ------------------------------------------------------------
void RemoteEspNow::begin()
{
    _initialized = false;

    _remote1Mask = 0;
    _remote2Mask = 0;
    _remote1LastRxTimeMs = 0;
    _remote2LastRxTimeMs = 0;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK)
    {
        return;
    }

    esp_now_register_recv_cb(onEspNowRecv);

    // Lägg till remote 1
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, s_remote1PeerMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    esp_err_t addPeerResult1 = esp_now_add_peer(&peerInfo);
    DBG_PRINTF("add_peer remote1 result = %d\n", addPeerResult1);

    // Lägg till remote 2
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, s_remote2PeerMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    esp_err_t addPeerResult2 = esp_now_add_peer(&peerInfo);
    DBG_PRINTF("add_peer remote2 result = %d\n", addPeerResult2);

    s_instance = this;
    _initialized = true;
}

uint32_t RemoteEspNow::getCombinedMask(uint32_t nowMs) const
{
    if (!_initialized)
        return 0;

    static constexpr uint32_t TIMEOUT_MS = 500;

    const uint32_t remote1Mask =
        ((nowMs - _remote1LastRxTimeMs) < TIMEOUT_MS) ? _remote1Mask : 0;

    const uint32_t remote2Mask =
        ((nowMs - _remote2LastRxTimeMs) < TIMEOUT_MS) ? _remote2Mask : 0;

    return remote1Mask | remote2Mask;
}

void RemoteEspNow::setRemote1Mask(uint32_t buttonMask, uint32_t rxTimeMs)
{
    _remote1Mask = buttonMask;
    _remote1LastRxTimeMs = rxTimeMs;
}

void RemoteEspNow::setRemote2Mask(uint32_t buttonMask, uint32_t rxTimeMs)
{
    _remote2Mask = buttonMask;
    _remote2LastRxTimeMs = rxTimeMs;
}

bool RemoteEspNow::sendStatus(const StatusPacket& status)
{
    if (!_initialized)
        return false;

    const esp_err_t result1 = esp_now_send(
        s_remote1PeerMac,
        reinterpret_cast<const uint8_t*>(&status),
        sizeof(StatusPacket)
    );

    const esp_err_t result2 = esp_now_send(
        s_remote2PeerMac,
        reinterpret_cast<const uint8_t*>(&status),
        sizeof(StatusPacket)
    );

    return (result1 == ESP_OK) || (result2 == ESP_OK);
}