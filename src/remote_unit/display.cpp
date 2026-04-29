#include "display.h"
#include "display_status.h"

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1327.h>

#define OLED_CS 8
#define OLED_DC 38
#define OLED_RST 39

Adafruit_SSD1327 display(128, 96, &SPI, OLED_DC, OLED_RST, OLED_CS);

static bool gDisplayAvailable = false;

bool display_is_available()
{
    return gDisplayAvailable;
}

void display_set_brightness(uint8_t value)
{
    if (!gDisplayAvailable)
        return;

    display.setContrast(value);
}

void display_begin()
{
    SPI.begin(36, -1, 35, OLED_CS);

    gDisplayAvailable = display.begin(0x3D);
    if (!gDisplayAvailable)
        return;

    display.clearDisplay();
    display.setTextColor(SSD1327_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("REMOTE OK");
    display.display();
}

static void drawLine(uint8_t line, const String &text, uint8_t textSize = 1)
{
    const int y = line * 16;

    display.setTextSize(textSize);
    display.setCursor(0, y);
    display.print(text);
}

void display_update(
    const StatusPacket &status,
    bool hasStatus,
    uint32_t buttonMask,
    bool linkAlive,
    bool calActive,
    bool calComplete,
    uint16_t calBucketMask,
    uint8_t calPhase)
{
    if (!gDisplayAvailable)
        return;

    display.clearDisplay();
    display.setTextColor(SSD1327_WHITE);

    if (!hasStatus)
    {
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.print("NO DATA");

        display.setTextSize(1);
        display.setCursor(0, 80);
        display.print(linkAlive ? "LINK OK" : "LINK LOST");

        display.display();
        return;
    }

    SystemState sys{};
    sys.mode = static_cast<SystemMode>(status.mode);
    sys.manualThrustPct = status.manualThrustPct;
    sys.targetSpeedPct = status.targetSpeedPct;
    sys.targetHeadingDeg = status.targetHeadingDeg10 / 10.0f;
    sys.sensors.headingDeg = status.headingDeg10 / 10.0f;
    sys.sensors.satellites = status.satellites;

    DisplayLines lines = buildDisplayLines(
        sys,
        buttonMask,
        linkAlive,
        calActive,
        calComplete,
        calBucketMask,
        calPhase);

    drawLine(0, lines.line1, 2);
    drawLine(2, lines.line2, 1);

    if (calActive || calComplete)
    {
        drawLine(3, "SPD " + String(status.gpsSpeedCmps / 100.0f, 1), 1);

        display.setCursor(0, 64);
        display.setTextSize(1);
        display.print("COG ");
        display.print(status.gpsCogDeg10 / 10);
    }
    else
    {
        String steerLine = "    |";

        if (status.steerState < 0)
            steerLine = "<--";
        else if (status.steerState > 0)
            steerLine = "     -->";

        drawLine(3, steerLine, 1);

        display.setCursor(0, 64);
        display.setTextSize(1);
        display.print("SAT ");
        display.print(status.satellites);
    }

    display.setCursor(0, 80);
    display.print(lines.line4);

    display.display();
}