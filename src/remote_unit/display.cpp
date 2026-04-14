#include "display.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// =====================================================
// Display object
// =====================================================
Adafruit_SH1106G display(128, 64, &Wire, -1);

// =====================================================
// Internal state
// =====================================================
static bool gDisplayAvailable = false;

// =====================================================
// Helpers
// =====================================================
static bool i2cDevicePresent(uint8_t addr)
{
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

static const char* modeText(uint8_t mode)
{
    switch (mode)
    {
        case 0: return "STOP";
        case 1: return "MANUAL";
        case 2: return "AUTO";
        case 3: return "ANCHOR";
        default: return "UNKNOWN";
    }
}

static const char* steerArrow(uint32_t buttonMask)
{
    constexpr uint32_t LEFT_BIT  = (1UL << 6); // STEER_LEFT
    constexpr uint32_t RIGHT_BIT = (1UL << 7); // STEER_RIGHT

    const bool left  = (buttonMask & LEFT_BIT) != 0;
    const bool right = (buttonMask & RIGHT_BIT) != 0;

    if (left && !right)  return "STEER: <--|";
    if (right && !left)  return "STEER:    |-->";
    return "STEER:    |";
}

// =====================================================
// Public API
// =====================================================
bool display_is_available()
{
    return gDisplayAvailable;
}

void display_set_brightness(uint8_t value)
{
    display.oled_command(0x81);
    display.oled_command(value);  
}


void display_begin()
{
    Wire.begin();
    Wire.setTimeOut(50);

    gDisplayAvailable = display.begin(0x3C, true);
    if (!gDisplayAvailable)
        return;

    display_set_brightness(20); // Set contrast to a fixed value (0-255). Adjust as needed.

    display.setTextColor(SH110X_WHITE);
    display.clearDisplay();
    display.display();
}


void display_update(const StatusPacket& status, bool hasStatus, uint32_t buttonMask, bool linkAlive)
{
    if (!gDisplayAvailable)
        return;

    if (!i2cDevicePresent(0x3C))
        return;

    display.clearDisplay();

    if (!hasStatus)
    {
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.print("NO DATA");

        display.setTextSize(1);
        display.setCursor(0, 54);
        display.print(linkAlive ? "LINK OK" : "LINK LOST");

        display.display();
        return;
    }

    // Row 1: mode, size 2
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(modeText(status.mode));

    // Row 2
    display.setTextSize(1);
    display.setCursor(0, 24);

    if (status.mode == 1) // MANUAL
    {
        display.print("THR ");
        display.print((int)roundf(status.manualThrustPct));
        display.print("%");
    }
    else
    {
        display.print("SPD ");
        display.print((int)roundf(status.targetSpeedPct));
        display.print("%");
    }

    // Row 3
    display.setCursor(0, 38);

    if (status.mode == 1) // MANUAL
    {
        display.print(steerArrow(buttonMask));
    }
    else
    {
        display.print("HDG ");
        display.print((int)roundf(status.targetHeadingDeg));
    }

    // Row 4
    display.setCursor(0, 54);
    display.print(linkAlive ? "LINK OK" : "LINK LOST");

    display.display();
}