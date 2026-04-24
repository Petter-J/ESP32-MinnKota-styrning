#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <cstring>

#include "remote_protocol.h"
#include "remote2/display_lcd.h"

// ============================================================
// BUTTON IDS
// ============================================================
enum class ButtonId : uint8_t
{
    STOP = 0,
    MODE_MANUAL,
    MODE_AUTO,
    MODE_ANCHOR,
    THRUST_UP,
    THRUST_DOWN,
    STEER_LEFT,
    STEER_RIGHT
};

constexpr uint32_t buttonBit(ButtonId id)
{
    return (1UL << static_cast<uint8_t>(id));
}

// ============================================================
// PIN CONFIG - REMOTE2
// ============================================================
namespace ButtonPins
{
    static constexpr int STOP         = 2;
    static constexpr int MODE_MANUAL  = 3;
    static constexpr int MODE_AUTO    = 16;
    static constexpr int MODE_ANCHOR  = 17;

    static constexpr int THRUST_UP    = 18;
    static constexpr int THRUST_DOWN  = 10;

    static constexpr int STEER_LEFT   = 11;
    static constexpr int STEER_RIGHT  = 44;
}

// ============================================================
// RECEIVER MAC
// ============================================================
static uint8_t RECEIVER_MAC[6] = {0xF0, 0x9E, 0x9E, 0x74, 0x8F, 0x9C};

// ============================================================
// STATUS
// ============================================================
static StatusPacket gStatus;
static bool gHasStatus = false;
static uint32_t gLastStatusMs = 0;

// ============================================================
// BUTTON READ
// ============================================================
static uint32_t readButtons()
{
    uint32_t mask = 0;

    if (digitalRead(ButtonPins::STOP) == LOW)
        mask |= buttonBit(ButtonId::STOP);

    if (digitalRead(ButtonPins::MODE_MANUAL) == LOW)
        mask |= buttonBit(ButtonId::MODE_MANUAL);

    if (digitalRead(ButtonPins::MODE_AUTO) == LOW)
        mask |= buttonBit(ButtonId::MODE_AUTO);

    if (digitalRead(ButtonPins::MODE_ANCHOR) == LOW)
        mask |= buttonBit(ButtonId::MODE_ANCHOR);

    if (digitalRead(ButtonPins::THRUST_UP) == LOW)
        mask |= buttonBit(ButtonId::THRUST_UP);

    if (digitalRead(ButtonPins::THRUST_DOWN) == LOW)
        mask |= buttonBit(ButtonId::THRUST_DOWN);

    if (digitalRead(ButtonPins::STEER_LEFT) == LOW)
        mask |= buttonBit(ButtonId::STEER_LEFT);

    if (digitalRead(ButtonPins::STEER_RIGHT) == LOW)
        mask |= buttonBit(ButtonId::STEER_RIGHT);

    return mask;
}

// ============================================================
// CALLBACKS
// ============================================================
void onSent(const uint8_t*, esp_now_send_status_t) {}

void onRecv(const uint8_t*, const uint8_t* data, int len)
{
    if (!data || len != (int)sizeof(StatusPacket))
        return;

    memcpy(&gStatus, data, sizeof(StatusPacket));
    gHasStatus = true;
    gLastStatusMs = millis();
}

// ============================================================
// SETUP
// ============================================================
void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    delay(1500);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.println();
    Serial.println("REMOTE2 START");
    Serial.print("REMOTE2 MAC: ");
    Serial.println(WiFi.macAddress());

    pinMode(ButtonPins::STOP, INPUT_PULLUP);
    pinMode(ButtonPins::MODE_MANUAL, INPUT_PULLUP);
    pinMode(ButtonPins::MODE_AUTO, INPUT_PULLUP);
    pinMode(ButtonPins::MODE_ANCHOR, INPUT_PULLUP);

    pinMode(ButtonPins::THRUST_UP, INPUT_PULLUP);
    pinMode(ButtonPins::THRUST_DOWN, INPUT_PULLUP);

    pinMode(ButtonPins::STEER_LEFT, INPUT_PULLUP);
    pinMode(ButtonPins::STEER_RIGHT, INPUT_PULLUP);

    display_lcd_begin();

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW init failed");
        return;
    }

    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onRecv);

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, RECEIVER_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;

    esp_now_add_peer(&peer);
}

// ============================================================
// LOOP
// ============================================================
void loop()
{
    static uint32_t lastSendMs = 0;
    static uint32_t lastPrintMask = 0;
    static uint32_t lastSentMask = 0;

    const uint32_t now = millis();
    const uint32_t buttonMask = readButtons();

    // Skicka knappar 
    const bool changed = (buttonMask != lastSentMask);
    const bool heartbeat = (now - lastSendMs >= 150);

    if (changed || heartbeat)
    {
        lastSendMs = now;
        lastSentMask = buttonMask;

        RemotePacket pkt;
        pkt.buttonMask = buttonMask;

        esp_now_send(RECEIVER_MAC,
                     reinterpret_cast<const uint8_t *>(&pkt),
                     sizeof(pkt));
    }

    // Link status
    const bool linkAlive = gHasStatus && ((now - gLastStatusMs) < 1000);

    // Uppdatera display
    display_lcd_update(gStatus, gHasStatus, buttonMask, linkAlive);

    // Print endast när knappmask ändras
    if (buttonMask != lastPrintMask)
    {
        lastPrintMask = buttonMask;

        Serial.print("Mask: 0x");
        Serial.print(buttonMask, HEX);
        Serial.print("  BIN: ");
        Serial.println(buttonMask, BIN);
    }
}