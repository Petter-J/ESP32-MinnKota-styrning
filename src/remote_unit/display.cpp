#include "display.h"
#include "display_status.h"

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#define TFT_CS 8
#define TFT_DC 38
#define TFT_RST 39
#define TFT_SCLK 36
#define TFT_MOSI 35

static Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

static bool gDisplayAvailable = false;

static constexpr int W = 240;
static constexpr int H = 320;

static constexpr int HEADER_Y = 0;
static constexpr int HEADER_H = 50;

static constexpr int ROW1_Y = 55;
static constexpr int ROW1_H = 60;

static constexpr int ROW2_Y = 120;
static constexpr int ROW2_H = 60;

static constexpr int ROW3_Y = 185;
static constexpr int ROW3_H = 60;

static constexpr int FOOTER1_Y = 255;
static constexpr int FOOTER2_Y = 285;

static constexpr uint16_t COLOR_BG = ILI9341_BLACK;
static constexpr uint16_t COLOR_TEXT = ILI9341_WHITE;
static constexpr uint16_t COLOR_DIM = 0x8410;
static constexpr uint16_t COLOR_GOOD = ILI9341_GREEN;
static constexpr uint16_t COLOR_WARN = ILI9341_YELLOW;
static constexpr uint16_t COLOR_BAD = ILI9341_RED;
static constexpr uint16_t COLOR_MANUAL = ILI9341_YELLOW;
static constexpr uint16_t COLOR_AUTO = ILI9341_CYAN;
static constexpr uint16_t COLOR_ANCHOR = ILI9341_GREEN;
static constexpr uint16_t COLOR_STOP = ILI9341_RED;
static constexpr uint16_t COLOR_CAL = ILI9341_MAGENTA;

bool display_is_available()
{
    return gDisplayAvailable;
}

void display_set_brightness(uint8_t value)
{
    (void)value;
}

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

static uint16_t modeColor(uint8_t mode)
{
    switch (mode)
    {
    case 0:
        return COLOR_STOP;
    case 1:
        return COLOR_MANUAL;
    case 2:
        return COLOR_AUTO;
    case 3:
        return COLOR_ANCHOR;
    default:
        return COLOR_TEXT;
    }
}

static uint16_t headingDisplayDeg(uint16_t deg10)
{
    return deg10 / 10;
}

static void drawCenteredText(
    const char *text,
    int16_t centerX,
    int16_t y,
    uint8_t textSize,
    uint16_t color)
{
    int16_t x1, y1;
    uint16_t w, h;

    tft.setTextSize(textSize);
    tft.setTextColor(color, COLOR_BG);
    tft.getTextBounds(text, 0, y, &x1, &y1, &w, &h);

    tft.setCursor(centerX - (w / 2), y);
    tft.print(text);
}

static void clearRow1()
{
    tft.fillRect(0, ROW1_Y, W, ROW1_H, COLOR_BG);
}

static void clearRow2()
{
    tft.fillRect(0, ROW2_Y, W, ROW2_H, COLOR_BG);
}

static void clearRow3()
{
    tft.fillRect(0, ROW3_Y, W, ROW3_H, COLOR_BG);
}

static void clearMainRows()
{
    clearRow1();
    clearRow2();
    clearRow3();
}

static void drawHeader(uint8_t mode)
{
    tft.fillRect(0, HEADER_Y, W, HEADER_H, modeColor(mode));

    tft.setTextColor(ILI9341_BLACK, modeColor(mode));
    tft.setTextWrap(false);

    int16_t x1, y1;
    uint16_t tw, th;
    const char *txt = modeText(mode);

    tft.setTextSize(3);
    tft.getTextBounds(txt, 0, 8, &x1, &y1, &tw, &th);
    tft.setCursor((W - tw) / 2, 8);
    tft.print(txt);
}

static void drawSteerIndicator(int8_t steerState, int16_t centerX, int16_t y)
{
    tft.setTextSize(4);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    if (steerState < 0)
    {
        tft.setCursor(centerX - 110, y);
        tft.print("<--");
    }
    else if (steerState > 0)
    {
        tft.setCursor(centerX + 35, y);
        tft.print("-->");
    }
    else
    {
        drawCenteredText("|", centerX, y, 4, COLOR_TEXT);
    }
}

static void drawFooter(
    const StatusPacket &status,
    bool linkAlive,
    uint32_t buttonMask)
{
    (void)buttonMask;

    tft.fillRect(0, FOOTER1_Y, W, 50, COLOR_BG);

    tft.setTextSize(2);

    tft.setTextColor(COLOR_WARN, COLOR_BG);
    tft.setCursor(10, FOOTER1_Y);
    tft.print(status.counter);

    const bool gpsOk =
        linkAlive && ((status.flags & STATUS_FLAG_GPS_VALID) != 0);

    drawCenteredText("GPS", W / 2, FOOTER1_Y, 2, gpsOk ? COLOR_GOOD : COLOR_BAD);

    tft.setCursor(W - 75, FOOTER1_Y);
    tft.setTextColor(linkAlive ? COLOR_GOOD : COLOR_BAD, COLOR_BG);
    tft.print(linkAlive ? "LINK" : "LOST");

    tft.setTextColor(COLOR_WARN, COLOR_BG);

    tft.setCursor(35, FOOTER2_Y);
    tft.print("BH");
    tft.print(headingDisplayDeg(status.boatHeadingDeg10));

    tft.setCursor(190, FOOTER2_Y);
    tft.print("MH");
    tft.print(headingDisplayDeg(status.motorHeadingDeg10));
}

enum ScreenType : uint8_t
{
    SCREEN_NO_DATA,
    SCREEN_OTA,
    SCREEN_CAL,
    SCREEN_NORMAL
};

static ScreenType getScreenType(
    bool hasStatus,
    bool linkAlive,
    bool linkLostTooLong,
    const StatusPacket &status,
    bool calActive,
    bool calComplete)
{
    if (!hasStatus || linkLostTooLong)
        return SCREEN_NO_DATA;

    if ((status.flags & STATUS_FLAG_OTA_ACTIVE) != 0)
        return SCREEN_OTA;

    if (calActive || calComplete)
        return SCREEN_CAL;

    return SCREEN_NORMAL;
}

void display_begin()
{
    SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);

    tft.begin();
    tft.setSPISpeed(40000000);
    tft.setRotation(0);
    tft.fillScreen(COLOR_BG);
    tft.setTextWrap(false);

    gDisplayAvailable = true;

    drawHeader(0);
    drawCenteredText("REMOTE OK", W / 2, 105, 3, COLOR_GOOD);
}

void display_update(
    const StatusPacket &status,
    bool hasStatus,
    uint32_t buttonMask,
    bool linkAlive,
    bool calActive,
    bool calComplete,
    uint16_t calBucketMask,
    uint8_t calPhase,
    bool localBoatHeadingValid,
    float localBoatHeadingDeg)
{
    (void)localBoatHeadingValid;
    (void)localBoatHeadingDeg;

    if (!gDisplayAvailable)
        return;

    static bool firstDraw = true;
    static ScreenType lastScreenType = SCREEN_NO_DATA;

    static uint8_t lastMode = 255;
    static bool lastLinkAlive = false;
    static uint32_t lastButtonMask = 0;

    static uint8_t lastManualThrustPct = 255;
    static uint16_t lastTargetHeadingDeg10 = 65535;
    static uint16_t lastTargetSpeedCmps = 65535;
    static uint16_t lastGpsSpeedCmps = 65535;
    static uint16_t lastGpsCogDeg10 = 65535;
    static uint16_t lastBoatHeadingDeg10 = 65535;
    static uint16_t lastMotorHeadingDeg10 = 65535;
    static uint8_t lastMotorTiltUnsafe = 255;
    static int8_t lastSteerState = 99;

    static uint8_t lastSatellites = 255;
    static uint8_t lastSatellitesInView = 255;
    static uint8_t lastFlags = 255;
    static uint16_t lastCalBucketMask = 65535;
    static uint8_t lastCalPhase = 255;

    static uint32_t linkLostSinceMs = 0;

    if (linkAlive)
    {
        linkLostSinceMs = 0;
    }
    else if (linkLostSinceMs == 0)
    {
        linkLostSinceMs = millis();
    }

    const bool linkLostTooLong =
        (!linkAlive && linkLostSinceMs != 0 &&
         (millis() - linkLostSinceMs >= 5000));

    const ScreenType screenType =
        getScreenType(
            hasStatus,
            linkAlive,
            linkLostTooLong,
            status,
            calActive,
            calComplete);

    const bool screenChanged =
        firstDraw || (screenType != lastScreenType);

    const bool modeChanged =
        screenChanged ||
        (screenType == SCREEN_NORMAL && status.mode != lastMode);

    const bool footerChanged =
        screenChanged ||
        linkAlive != lastLinkAlive ||
        buttonMask != lastButtonMask ||
        status.satellites != lastSatellites ||
        status.satellitesInView != lastSatellitesInView ||
        status.flags != lastFlags ||
        headingDisplayDeg(status.boatHeadingDeg10) != headingDisplayDeg(lastBoatHeadingDeg10) ||
        headingDisplayDeg(status.motorHeadingDeg10) != headingDisplayDeg(lastMotorHeadingDeg10) ||
        status.counter != 255; // enkel: footer får uppdateras ofta

    if (firstDraw)
    {
        tft.fillScreen(COLOR_BG);
        firstDraw = false;
    }

    if (screenChanged)
    {
        tft.fillScreen(COLOR_BG);
    }

    if (screenType == SCREEN_NO_DATA)
    {
        if (screenChanged)
        {
            tft.fillRect(0, HEADER_Y, W, HEADER_H, COLOR_BAD);
            drawCenteredText("NO DATA", W / 2, 8, 3, ILI9341_BLACK);

            clearMainRows();
            drawCenteredText("WAITING FOR", W / 2, ROW1_Y + 8, 3, COLOR_TEXT);
            drawCenteredText("MAIN UNIT", W / 2, ROW2_Y + 8, 3, COLOR_TEXT);
        }

        lastScreenType = screenType;
        lastLinkAlive = linkAlive;
        return;
    }

    if (screenChanged || modeChanged)
    {
        if (screenType == SCREEN_OTA)
        {
            tft.fillRect(0, HEADER_Y, W, HEADER_H, COLOR_CAL);
            drawCenteredText("OTA", W / 2, 8, 3, ILI9341_BLACK);
        }
        else if (screenType == SCREEN_CAL)
        {
            tft.fillRect(0, HEADER_Y, W, HEADER_H, COLOR_CAL);
            drawCenteredText("CAL", W / 2, 8, 3, ILI9341_BLACK);
        }
        else
        {
            drawHeader(status.mode);
        }
    }

    if (screenType == SCREEN_OTA)
    {
        if (screenChanged)
        {
            clearMainRows();
            drawCenteredText("OTA", W / 2, ROW1_Y + 4, 4, COLOR_CAL);
            drawCenteredText("UPDATE MODE", W / 2, ROW2_Y + 10, 3, COLOR_TEXT);
            drawCenteredText("192.168.4.1", W / 2, ROW3_Y + 10, 3, COLOR_TEXT);
        }

        if (footerChanged)
            drawFooter(status, linkAlive, buttonMask);

        goto save_state;
    }

    if (screenType == SCREEN_CAL)
    {
        if (screenChanged || calBucketMask != lastCalBucketMask || calPhase != lastCalPhase)
        {
            clearRow1();

            char calLine1[32];

            if (calComplete)
            {
                snprintf(calLine1, sizeof(calLine1), "DONE");
            }
            else
            {
                uint8_t count = 0;

                for (uint8_t i = 0; i < 16; i++)
                {
                    if (calBucketMask & (1 << i))
                        count++;
                }

                const char *phaseText = "--";

                if (calPhase == 1)
                    phaseText = "CW";
                else if (calPhase == 2)
                    phaseText = "CCW";

                snprintf(calLine1, sizeof(calLine1), "%s %u/16", phaseText, count);
            }

            drawCenteredText(calLine1, W / 2, ROW1_Y + 2, 4, COLOR_CAL);
        }

        if (screenChanged || status.gpsSpeedCmps != lastGpsSpeedCmps)
        {
            clearRow2();

            char spdLine[32];
            snprintf(spdLine, sizeof(spdLine), "SPD %.1f M/S", status.gpsSpeedCmps / 100.0f);
            drawCenteredText(spdLine, W / 2, ROW2_Y + 8, 3, COLOR_TEXT);
        }

        if (screenChanged || status.gpsCogDeg10 != lastGpsCogDeg10)
        {
            clearRow3();

            char cogLine[32];
            snprintf(cogLine, sizeof(cogLine), "COG %u", status.gpsCogDeg10 / 10);
            drawCenteredText(cogLine, W / 2, ROW3_Y + 8, 3, COLOR_TEXT);
        }

        if (footerChanged)
            drawFooter(status, linkAlive, buttonMask);

        goto save_state;
    }

    if (modeChanged)
    {
        clearMainRows();
    }

    if (status.mode == 0)
    {
        if (modeChanged || status.motorTiltUnsafe != lastMotorTiltUnsafe)
        {
            clearRow1();

            if (status.motorTiltUnsafe)
                drawCenteredText("NOT READY", W / 2, ROW1_Y + 8, 3, COLOR_STOP);
            else
                drawCenteredText("SET MODE", W / 2, ROW1_Y + 8, 3, COLOR_GOOD);
        }

        if (modeChanged || status.motorTiltUnsafe != lastMotorTiltUnsafe)
        {
            clearRow2();

            if (status.motorTiltUnsafe)
                drawCenteredText("MOTOR UPPE", W / 2, ROW2_Y + 8, 3, COLOR_STOP);
            else
                drawCenteredText("MOTOR OK", W / 2, ROW2_Y + 8, 3, COLOR_GOOD);
        }

        if (modeChanged)
            clearRow3();
    }
    else if (status.mode == 1)
    {
        if (modeChanged || status.manualThrustPct != lastManualThrustPct)
        {
            clearRow1();

            char line1[32];
            snprintf(line1, sizeof(line1), "THR %u%%", status.manualThrustPct);
            drawCenteredText(line1, W / 2, ROW1_Y + 2, 4, COLOR_MANUAL);
        }

        if (modeChanged || status.steerState != lastSteerState)
        {
            clearRow2();
            drawSteerIndicator(status.steerState, W / 2, ROW2_Y + 2);
        }

        if (modeChanged || status.gpsSpeedCmps != lastGpsSpeedCmps)
        {
            clearRow3();

            float speedMps = status.gpsSpeedCmps / 100.0f;
            float speedKn = speedMps * 1.94384f;

            char line3[32];
            snprintf(line3, sizeof(line3), "KN %.2f | M/S %.1f", speedKn, speedMps);
            drawCenteredText(line3, W / 2, ROW3_Y + 10, 2, COLOR_TEXT);
        }
    }
    else if (status.mode == 2)
    {
        if (modeChanged ||
            status.targetHeadingDeg10 != lastTargetHeadingDeg10 ||
            status.boatHeadingDeg10 != lastBoatHeadingDeg10)
        {
            clearRow1();

            char line1[32];
            snprintf(line1, sizeof(line1),
                     "T%u | H%u",
                     status.targetHeadingDeg10 / 10,
                     status.boatHeadingDeg10 / 10);

            drawCenteredText(line1, W / 2, ROW1_Y + 8, 3, COLOR_AUTO);
        }

        if (modeChanged ||
            status.targetSpeedCmps != lastTargetSpeedCmps ||
            status.gpsSpeedCmps != lastGpsSpeedCmps)
        {
            clearRow2();

            const float targetMps = status.targetSpeedCmps / 100.0f;
            const float actualMps = status.gpsSpeedCmps / 100.0f;

            const float targetKn = targetMps * 1.94384f;
            const float actualKn = actualMps * 1.94384f;

            char line2[32];
            snprintf(line2, sizeof(line2), "K%.1f | K%.1f", targetKn, actualKn);
            drawCenteredText(line2, W / 2, ROW2_Y + 8, 3, COLOR_AUTO);
        }

        if (modeChanged || status.gpsSpeedCmps != lastGpsSpeedCmps)
        {
            clearRow3();

            const float actualMps = status.gpsSpeedCmps / 100.0f;

            char line3[32];
            snprintf(line3, sizeof(line3), "M/S %.2f", actualMps);
            drawCenteredText(line3, W / 2, ROW3_Y + 8, 3, COLOR_TEXT);
        }
    }
    else if (status.mode == 3)
    {
        if (modeChanged)
        {
            clearRow1();
            drawCenteredText("ANCHOR", W / 2, ROW1_Y + 2, 4, COLOR_ANCHOR);
        }

        if (modeChanged || status.targetHeadingDeg10 != lastTargetHeadingDeg10)
        {
            clearRow2();

            char line2[32];
            snprintf(line2, sizeof(line2), "HDG %u", status.targetHeadingDeg10 / 10);
            drawCenteredText(line2, W / 2, ROW2_Y + 8, 3, COLOR_TEXT);
        }

        if (modeChanged)
        {
            clearRow3();
            drawCenteredText("POSITION HOLD", W / 2, ROW3_Y + 10, 2, COLOR_DIM);
        }
    }
    else
    {
        if (modeChanged)
        {
            clearMainRows();
            drawCenteredText("UNKNOWN", W / 2, 100, 4, COLOR_WARN);
        }
    }

    if (footerChanged)
        drawFooter(status, linkAlive, buttonMask);

save_state:
    lastScreenType = screenType;
    lastLinkAlive = linkAlive;
    lastButtonMask = buttonMask;

    lastMode = status.mode;
    lastManualThrustPct = status.manualThrustPct;
    lastTargetHeadingDeg10 = status.targetHeadingDeg10;
    lastTargetSpeedCmps = status.targetSpeedCmps;
    lastGpsSpeedCmps = status.gpsSpeedCmps;
    lastGpsCogDeg10 = status.gpsCogDeg10;
    lastBoatHeadingDeg10 = status.boatHeadingDeg10;
    lastMotorHeadingDeg10 = status.motorHeadingDeg10;
    lastMotorTiltUnsafe = status.motorTiltUnsafe;
    lastSteerState = status.steerState;

    lastSatellites = status.satellites;
    lastSatellitesInView = status.satellitesInView;
    lastFlags = status.flags;
    lastCalBucketMask = calBucketMask;
    lastCalPhase = calPhase;
}