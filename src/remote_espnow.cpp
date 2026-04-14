#include "remote_espnow.h"
#include "config.h"
#include <WiFi.h>
#include <esp_now.h>
#include <cstring>

// ------------------------------------------------------------
// Packet format from remote -> head unit
// ------------------------------------------------------------
struct RemotePacket
{
    uint32_t buttonMask = 0;
};

// ------------------------------------------------------------
// Static instance pointer for ESP-NOW callback
// ------------------------------------------------------------
static RemoteEspNow* s_instance = nullptr;

// ------------------------------------------------------------
// MAC address of remote_unit (DISPLAY/REMOTE)
// Fyll i rätt MAC här från remote_unit Serial.print(WiFi.macAddress())
// ------------------------------------------------------------
static uint8_t s_remotePeerMac[6] = { 0xA0, 0xB7, 0x65, 0x06, 0xC1, 0x14 };

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

    // Lägg till peer för att kunna skicka status tillbaka
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, s_remotePeerMac, 6);
    peerInfo.channel = 0;      // samma kanal
    peerInfo.encrypt = false;

    // Om MAC inte är satt korrekt kommer add_peer sannolikt misslyckas
    esp_err_t addPeerResult = esp_now_add_peer(&peerInfo);
    DBG_PRINTF("add_peer result = %d\n", addPeerResult);

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

    esp_err_t result = esp_now_send(s_remotePeerMac,
                                    reinterpret_cast<const uint8_t*>(&status),
                                    sizeof(StatusPacket));

    return (result == ESP_OK);
}