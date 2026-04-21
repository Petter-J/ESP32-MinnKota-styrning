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
static uint8_t s_remote1PeerMac[6] = { 0xA0, 0xB7, 0x65, 0x06, 0xC1, 0x14 }; // gamla remote
static uint8_t s_remote2PeerMac[6] = { 0x20, 0x6E, 0xF1, 0x9B, 0xB3, 0x08 }; // remote2 S3

// ------------------------------------------------------------
// ESP-NOW receive callback
// ------------------------------------------------------------
static void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len)
{
    (void)mac;

    if (s_instance == nullptr)
        return;

    if (data == nullptr || len != (int)sizeof(RemotePacket))
        return;

    RemotePacket pkt;
    memcpy(&pkt, data, sizeof(RemotePacket));

    s_instance->setLatestButtonMask(pkt.buttonMask, millis());
}

// ------------------------------------------------------------
// RemoteEspNow
// ------------------------------------------------------------
void RemoteEspNow::begin()
{
    _latestButtonMask = 0;
    _hasNewData = false;
    _lastRxTimeMs = 0;
    _initialized = false;

    WiFi.mode(WIFI_STA);

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

bool RemoteEspNow::update(uint32_t& outButtonMask)
{
    if (!_initialized)
        return false;

    if (!_hasNewData)
        return false;

    outButtonMask = _latestButtonMask;
    _hasNewData = false;
    return true;
}

uint32_t RemoteEspNow::getLatestMask() const
{
    return _latestButtonMask;
}

bool RemoteEspNow::isAlive(uint32_t nowMs) const
{
    static constexpr uint32_t TIMEOUT_MS = 500;
    return (nowMs - _lastRxTimeMs) < TIMEOUT_MS;
}

void RemoteEspNow::setLatestButtonMask(uint32_t buttonMask, uint32_t rxTimeMs)
{
    _latestButtonMask = buttonMask;
    _lastRxTimeMs = rxTimeMs;
    _hasNewData = true;
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