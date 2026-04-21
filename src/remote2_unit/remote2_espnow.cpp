#include "remote2/remote2_espnow.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <cstring>

static StatusPacket gStatus{};
static bool gHasStatus = false;
static uint32_t gLastStatusMs = 0;

// ------------------------------------------------------------
static void onRecv(const uint8_t*, const uint8_t* data, int len)
{
    Serial.printf("[ESPNOW] RX len=%d\n", len);

    if (!data || len != (int)sizeof(StatusPacket))
    {
        Serial.println("[ESPNOW] wrong packet");
        return;
    }

    memcpy(&gStatus, data, sizeof(StatusPacket));
    gHasStatus = true;
    gLastStatusMs = millis();

    Serial.printf("[ESPNOW] status rx mode=%u sats=%u counter=%u\n",
                  gStatus.mode,
                  gStatus.satellites,
                  gStatus.counter);
}

// ------------------------------------------------------------
bool remote2_espnow_begin(const uint8_t* peerMac)
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.print("[ESPNOW] remote MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[ESPNOW] init failed");
        return false;
    }

    esp_now_register_recv_cb(onRecv);

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, peerMac, 6);
    peer.channel = 0;
    peer.encrypt = false;

    const esp_err_t addPeerResult = esp_now_add_peer(&peer);
    if (addPeerResult != ESP_OK)
    {
        Serial.printf("[ESPNOW] peer add failed: %d\n", (int)addPeerResult);
        return false;
    }

    Serial.println("[ESPNOW] ready");
    return true;
}

// ------------------------------------------------------------
bool remote2_espnow_update(StatusPacket& outStatus, bool& outHasStatus, uint32_t now)
{
    outStatus = gStatus;
    outHasStatus = gHasStatus;

    return gHasStatus && ((now - gLastStatusMs) < 1000);
}