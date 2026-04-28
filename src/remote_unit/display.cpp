#include "display.h"

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1327.h>

#define OLED_CS 8
#define OLED_DC 38
#define OLED_RST 39

Adafruit_SSD1327 display(128, 96, &SPI, OLED_DC, OLED_RST, OLED_CS);

static bool gDisplayAvailable = false;

static const char *modeText(uint8_t mode)
{
    switch (mode)
    {
    case 0:
        return "STOP";
    case 1:
        return "MANUAL";
    case 2:
        return "AUTO";
    case 3:
        return "ANCHOR";
    default:
        return "UNKNOWN";
    }
}

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
    SPI.begin(36, -1, 35, OLED_CS); // SCK, MISO, MOSI, CS

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

void display_update(const StatusPacket &status, bool hasStatus, uint32_t buttonMask, bool linkAlive)
{
    (void)buttonMask;

    if (!gDisplayAvailable)
        return;

    display.clearDisplay();

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

    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(modeText(status.mode));

    display.setTextSize(1);
    display.setCursor(0, 32);

    if (status.mode == 1)
    {
        display.print("THR ");
        display.print(status.manualThrustPct);
        display.print("%");
    }
    else
    {
        display.print("SPD ");
        display.print(status.targetSpeedPct);
        display.print("%");
    }

    display.setCursor(0, 48);
    display.print("HDG ");
    display.print(status.headingDeg10 / 10);

    display.setCursor(0, 64);
    display.print("SAT ");
    display.print(status.satellites);

    display.setCursor(0, 80);
    display.print(linkAlive ? "LINK OK" : "LINK LOST");

    display.display();
}