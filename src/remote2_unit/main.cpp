#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "remote2/display_lcd.h"
#include "remote_protocol.h"
#include "remote2/remote2_espnow.h"

// ÄNDRA till din main-unit MAC
static uint8_t RECEIVER_MAC[6] = { 0xA0, 0xB7, 0x65, 0x07, 0xCF, 0x24 };

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

static uint32_t readButtons()
{
    uint32_t mask = 0;

    if (digitalRead(ButtonPins::STOP) == LOW)
        mask |= (1UL << 0);

    if (digitalRead(ButtonPins::MODE_MANUAL) == LOW)
        mask |= (1UL << 1);

    if (digitalRead(ButtonPins::MODE_AUTO) == LOW)
        mask |= (1UL << 2);

    if (digitalRead(ButtonPins::MODE_ANCHOR) == LOW)
        mask |= (1UL << 3);

    if (digitalRead(ButtonPins::THRUST_UP) == LOW)
        mask |= (1UL << 4);

    if (digitalRead(ButtonPins::THRUST_DOWN) == LOW)
        mask |= (1UL << 5);

    if (digitalRead(ButtonPins::STEER_LEFT) == LOW)
        mask |= (1UL << 6);

    if (digitalRead(ButtonPins::STEER_RIGHT) == LOW)
        mask |= (1UL << 7);

    return mask;
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    delay(1500);

    Serial.println();
    Serial.println("REMOTE2 START");

    pinMode(ButtonPins::STOP, INPUT_PULLUP);
    pinMode(ButtonPins::MODE_MANUAL, INPUT_PULLUP);
    pinMode(ButtonPins::MODE_AUTO, INPUT_PULLUP);
    pinMode(ButtonPins::MODE_ANCHOR, INPUT_PULLUP);

    pinMode(ButtonPins::THRUST_UP, INPUT_PULLUP);
    pinMode(ButtonPins::THRUST_DOWN, INPUT_PULLUP);

    pinMode(ButtonPins::STEER_LEFT, INPUT_PULLUP);
    pinMode(ButtonPins::STEER_RIGHT, INPUT_PULLUP);

    display_lcd_begin();

    remote2_espnow_begin(RECEIVER_MAC);
}

void loop()
{
    static uint32_t lastMs = 0;
    static uint32_t lastPrintedMask = 0;
    static bool ledState = false;

    const uint32_t now = millis();
    const uint32_t buttonMask = readButtons();

    StatusPacket status{};
    bool hasStatus = false;

    const bool linkAlive = remote2_espnow_update(status, hasStatus, now);

    // Blink LED + uppdatera display
    if (now - lastMs >= 200)
    {
        lastMs = now;

        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);

        display_lcd_update(status, hasStatus, buttonMask, linkAlive);
    }

    // Print endast när något ändras
    if (buttonMask != lastPrintedMask)
    {
        lastPrintedMask = buttonMask;

        Serial.print("Mask: 0x");
        Serial.print(buttonMask, HEX);
        Serial.print("  BIN: ");
        Serial.println(buttonMask, BIN);
    }
}