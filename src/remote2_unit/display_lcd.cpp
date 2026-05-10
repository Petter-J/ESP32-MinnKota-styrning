#include "remote2/display_lcd.h"

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// =====================================================
// LCD pins - Waveshare ESP32-S3-LCD-1.69
// =====================================================
static constexpr int LCD_CS   = 5;
static constexpr int LCD_DC   = 4;
static constexpr int LCD_RST  = 8;
static constexpr int LCD_BL   = 15;
static constexpr int LCD_SCLK = 6;
static constexpr int LCD_MOSI = 7;

// =====================================================
// LCD object
// =====================================================
static Adafruit_ST7789 tft(LCD_CS, LCD_DC, LCD_RST);

// =====================================================
// Colors
// =====================================================
static constexpr uint16_t COLOR_BG         = ST77XX_BLACK;
static constexpr uint16_t COLOR_TEXT       = ST77XX_WHITE;
static constexpr uint16_t COLOR_DIM        = 0x8410; // grå
static constexpr uint16_t COLOR_ACCENT     = ST77XX_CYAN;
static constexpr uint16_t COLOR_GOOD       = ST77XX_GREEN;
static constexpr uint16_t COLOR_WARN       = ST77XX_YELLOW;
static constexpr uint16_t COLOR_BAD        = ST77XX_RED;
static constexpr uint16_t COLOR_MANUAL     = ST77XX_YELLOW;
static constexpr uint16_t COLOR_AUTO       = ST77XX_CYAN;
static constexpr uint16_t COLOR_ANCHOR     = ST77XX_GREEN;
static constexpr uint16_t COLOR_STOP       = ST77XX_RED;
static constexpr uint16_t COLOR_CAL        = ST77XX_MAGENTA;

// =====================================================
// ROWS
// =====================================================
static constexpr int HEADER_Y = 0;
static constexpr int HEADER_H = 50;

static constexpr int ROW1_Y = 50;
static constexpr int ROW1_H = 60;

static constexpr int ROW2_Y = 110;
static constexpr int ROW2_H = 60;

static constexpr int ROW3_Y = 170;
static constexpr int ROW3_H = 60;



// =====================================================
// Helpers
// =====================================================

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

static uint16_t modeColor(uint8_t mode)
{
    switch (mode)
    {
        case 0: return COLOR_STOP;
        case 1: return COLOR_MANUAL;
        case 2: return COLOR_AUTO;
        case 3: return COLOR_ANCHOR;
        default: return COLOR_TEXT;
    }
}

static void clearMainRows()
{
    tft.fillRect(0, ROW1_Y, 240, ROW1_H, COLOR_BG);
    tft.fillRect(0, ROW2_Y, 240, ROW2_H, COLOR_BG);
    tft.fillRect(0, ROW3_Y, 240, ROW3_H, COLOR_BG);
}

static void clearRow1()
{
    tft.fillRect(0, ROW1_Y, 240, ROW1_H, COLOR_BG);
}

static void clearRow2()
{
    tft.fillRect(0, ROW2_Y, 240, ROW2_H, COLOR_BG);
}

static void clearRow3()
{
    tft.fillRect(0, ROW3_Y, 240, ROW3_H, COLOR_BG);
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

static uint16_t headingDisplayDeg(uint16_t deg10)
{
    return deg10 / 10;
}

static void drawCenteredText(const char* text, int16_t centerX, int16_t y, uint8_t textSize, uint16_t color)
{
    int16_t x1, y1;
    uint16_t w, h;

    tft.setTextSize(textSize);
    tft.setTextColor(color);
    tft.getTextBounds(text, 0, y, &x1, &y1, &w, &h);

    const int16_t x = centerX - (w / 2);
    tft.setCursor(x, y);
    tft.print(text);
}

static void drawSteerIndicator(int8_t steerState, int16_t centerX, int16_t y)
{
    tft.setTextSize(4);
    tft.setTextColor(COLOR_TEXT);

    if (steerState < 0)
    {
        tft.setCursor(centerX - 95, y);
        tft.print("<--");
    }
    else if (steerState > 0)
    {
        tft.setCursor(centerX + 25, y);
        tft.print("-->");
    }
    else
    {
        drawCenteredText("|", centerX, y, 4, COLOR_TEXT);
    }
}

static void drawHeader(uint8_t mode)
{
    tft.fillRect(0, HEADER_Y, 240, HEADER_H, modeColor(mode));

    tft.setTextWrap(false);
    drawCenteredText(modeText(mode), 120, 10, 4, ST77XX_BLACK);
}

static void drawFooter(const StatusPacket &status, bool linkAlive, uint32_t buttonMask)
{
    // Footer top row
    tft.fillRect(0, 230, 240, 25, COLOR_BG);

    

    tft.setTextSize(2);
    tft.setTextColor(COLOR_WARN);
    tft.setCursor(10, 230);
   // tft.print("S ");
    tft.print(status.counter);
    // tft.print("/");
    // tft.print(status.satellitesInView);

    const bool gpsOk = linkAlive && ((status.flags & STATUS_FLAG_GPS_VALID) != 0);

    drawCenteredText("GPS", 120, 230, 2, gpsOk ? COLOR_GOOD : COLOR_BAD);

    tft.setCursor(180, 230);
    tft.setTextColor(linkAlive ? COLOR_GOOD : COLOR_BAD);
    tft.print(linkAlive ? "LINK" : "LOST");

    tft.setTextSize(2);

    

    tft.fillRect(25, 255, 110, 25, COLOR_BG);

    tft.setTextColor(COLOR_WARN);
    tft.setCursor(45, 262);
    tft.print("BH");
    tft.print(headingDisplayDeg(status.boatHeadingDeg10));

    tft.fillRect(130, 255, 75, 25, COLOR_BG);

    tft.setCursor(140, 262);
    tft.print("MH");
    tft.print(headingDisplayDeg(status.motorHeadingDeg10));
}
// =====================================================
// Public API
// =====================================================
void display_lcd_begin()
{
    ledcSetup(0, 10000, 8);        // channel 0, 10kHz, 8-bit
    ledcAttachPin(LCD_BL, 0);
    ledcWrite(0, 60);            

    SPI.begin(LCD_SCLK, -1, LCD_MOSI, LCD_CS);

    tft.init(240, 280);
    tft.setSPISpeed(40000000);
    tft.setRotation(0);
    tft.fillScreen(COLOR_BG);
    tft.setTextWrap(false);
}

void display_lcd_update(
    const StatusPacket &status,
    bool hasStatus,
    uint32_t buttonMask,
    bool linkAlive,
    bool calActive,
    bool calComplete,
    uint16_t calBucketMask,
    uint8_t calPhase)
{
    static bool firstDraw = true;
    static ScreenType lastScreenType = SCREEN_NO_DATA;

    static uint8_t lastMode = 255;
    static bool lastHasStatus = false;
    static bool lastLinkAlive = false;
    static uint32_t lastButtonMask = 0;

    static uint8_t lastManualThrustPct = 255;
    static uint16_t lastTargetHeadingDeg10 = 65535;
    static uint16_t lastTargetSpeedCmps = 65535;
    static uint16_t lastGpsSpeedCmps = 65535;
    static uint16_t lastGpsCogDeg10 = 65535;
    static uint16_t lastBoatHeadingDeg10 = 65535;
    static uint8_t lastMotorTiltUnsafe = 255;
    static int8_t lastSteerState = 99;

    static uint8_t lastSatellites = 255;
    static uint8_t lastSatellitesInView = 255;
    static uint8_t lastFlags = 255;
    static uint16_t lastMotorHeadingDeg10 = 65535;
    static uint16_t lastCalBucketMask = 65535;
    static uint8_t lastCalPhase = 255;

    static uint32_t linkLostSinceMs = 0;

    if (linkAlive)
    {
        linkLostSinceMs = 0;
    }
    else
    {
        if (linkLostSinceMs == 0)
            linkLostSinceMs = millis();
    }

    const bool linkLostTooLong =
        (!linkAlive && linkLostSinceMs != 0 && (millis() - linkLostSinceMs >= 5000));

    const ScreenType screenType =
        getScreenType(hasStatus, linkAlive, linkLostTooLong, status, calActive, calComplete);

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
        headingDisplayDeg(status.motorHeadingDeg10) != headingDisplayDeg(lastMotorHeadingDeg10);

    if (firstDraw)
    {
        tft.fillScreen(COLOR_BG);
        firstDraw = false;
    }

    if (screenChanged)
    {
        tft.fillScreen(COLOR_BG);
    }

    // =========================
    // NO DATA
    // =========================
    if (screenType == SCREEN_NO_DATA)
    {
        if (screenChanged)
        {
            tft.fillRect(0, HEADER_Y, 240, HEADER_H, COLOR_BAD);
            drawCenteredText("NO DATA", 120, 10, 4, ST77XX_BLACK);

            clearMainRows();
            drawCenteredText("WAITING FOR", 120, ROW1_Y + 25, 3, COLOR_TEXT);
            drawCenteredText("MAIN UNIT", 120, ROW2_Y + 20, 3, COLOR_TEXT);
        }

        lastScreenType = screenType;
        lastHasStatus = hasStatus;
        lastLinkAlive = linkAlive;
        return;
    }

    // =========================
    // HEADER
    // =========================
    if (screenChanged || modeChanged)
    {
        if (screenType == SCREEN_OTA)
        {
            tft.fillRect(0, HEADER_Y, 240, HEADER_H, COLOR_CAL);
            drawCenteredText("OTA", 120, 10, 4, ST77XX_BLACK);
        }
        else if (screenType == SCREEN_CAL)
        {
            tft.fillRect(0, HEADER_Y, 240, HEADER_H, COLOR_CAL);
            drawCenteredText("CAL", 120, 10, 4, ST77XX_BLACK);
        }
        else
        {
            drawHeader(status.mode);
        }
    }

    // =========================
    // OTA
    // =========================
    if (screenType == SCREEN_OTA)
    {
        if (screenChanged)
        {
            clearMainRows();
            drawCenteredText("OTA", 120, ROW1_Y + 15, 4, COLOR_CAL);
            drawCenteredText("UPDATE MODE", 120, ROW2_Y + 20, 3, COLOR_TEXT);
            drawCenteredText("192.168.4.1", 120, ROW3_Y + 20, 3, COLOR_TEXT);
        }

        if (footerChanged)
            drawFooter(status, linkAlive, buttonMask);

        goto save_state;
    }

    // =========================
    // CAL
    // =========================
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

            drawCenteredText(calLine1, 120, ROW1_Y + 10, 5, COLOR_CAL);
        }

        if (screenChanged || status.gpsSpeedCmps != lastGpsSpeedCmps)
        {
            clearRow2();

            char spdLine[32];
            snprintf(spdLine, sizeof(spdLine), "SPD %.1f M/S", status.gpsSpeedCmps / 100.0f);
            drawCenteredText(spdLine, 120, ROW2_Y + 15, 3, COLOR_TEXT);
        }

        if (screenChanged || status.gpsCogDeg10 != lastGpsCogDeg10)
        {
            clearRow3();

            char cogLine[32];
            snprintf(cogLine, sizeof(cogLine), "COG %u", status.gpsCogDeg10 / 10);
            drawCenteredText(cogLine, 120, ROW3_Y + 15, 3, COLOR_TEXT);
        }

        if (footerChanged)
            drawFooter(status, linkAlive, buttonMask);

        goto save_state;
    }

    // =========================
    // NORMAL MODE
    // =========================

    if (modeChanged)
    {
        clearMainRows();
    }

    if (status.mode == 0) // STOP
    {
        if (modeChanged || status.motorTiltUnsafe != lastMotorTiltUnsafe)
        {
            clearRow1();

            if (status.motorTiltUnsafe)
                drawCenteredText("NOT READY", 120, ROW1_Y + 20, 3, COLOR_STOP);
            else
                drawCenteredText("SET MODE", 120, ROW1_Y + 20, 3, COLOR_GOOD);
        }

        if (modeChanged || status.motorTiltUnsafe != lastMotorTiltUnsafe)
        {
            clearRow2();

            if (status.motorTiltUnsafe)
                drawCenteredText("MOTOR UPPE", 120, ROW2_Y + 15, 3, COLOR_STOP);
            else
                drawCenteredText("MOTOR OK", 120, ROW2_Y + 15, 3, COLOR_GOOD);
        }

        if (modeChanged)
            clearRow3();
    }

    else if (status.mode == 1) // MANUAL
    {
        if (modeChanged || status.manualThrustPct != lastManualThrustPct)
        {
            clearRow1();

            char line1[32];
            snprintf(line1, sizeof(line1), "THR %u%%", status.manualThrustPct);
            drawCenteredText(line1, 120, ROW1_Y + 15, 4, COLOR_MANUAL);
        }

        if (modeChanged || status.steerState != lastSteerState)
        {
            clearRow2();
            drawSteerIndicator(status.steerState, 120, ROW2_Y + 15);
        }

        if (modeChanged || status.gpsSpeedCmps != lastGpsSpeedCmps)
        {
            clearRow3();

            float speedMps = status.gpsSpeedCmps / 100.0f;
            float speedKn = speedMps * 1.94384f;

            char line3[32];
            snprintf(line3, sizeof(line3), "KN %.2f | M/S %.1f", speedKn, speedMps);
            drawCenteredText(line3, 120, ROW3_Y + 18, 2, COLOR_TEXT);
        }
    }

    else if (status.mode == 2) // AUTO
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

            drawCenteredText(line1, 120, ROW1_Y + 15, 3, COLOR_AUTO);
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
            drawCenteredText(line2, 120, ROW2_Y + 15, 3, COLOR_AUTO);
        }

        if (modeChanged || status.gpsSpeedCmps != lastGpsSpeedCmps)
        {
            clearRow3();

            const float actualMps = status.gpsSpeedCmps / 100.0f;

            char line3[32];
            snprintf(line3, sizeof(line3), "M/S %.2f", actualMps);
            drawCenteredText(line3, 120, ROW3_Y + 18, 3, COLOR_TEXT);
        }
    }

    else if (status.mode == 3) // ANCHOR
    {
        if (modeChanged)
        {
            clearRow1();
            drawCenteredText("ANCHOR", 120, ROW1_Y + 8, 4, COLOR_ANCHOR);
        }

        if (modeChanged || status.targetHeadingDeg10 != lastTargetHeadingDeg10)
        {
            clearRow2();

            char line2[32];
            snprintf(line2, sizeof(line2), "HDG %u", status.targetHeadingDeg10 / 10);
            drawCenteredText(line2, 120, ROW2_Y + 15, 3, COLOR_TEXT);
        }

        if (modeChanged)
        {
            clearRow3();
            drawCenteredText("POSITION HOLD", 120, ROW3_Y + 20, 2, COLOR_DIM);
        }
    }

    else
    {
        if (modeChanged)
        {
            clearMainRows();
            drawCenteredText("UNKNOWN", 120, 90, 4, COLOR_WARN);
        }
    }

    if (footerChanged)
        drawFooter(status, linkAlive, buttonMask);

save_state:
    lastScreenType = screenType;
    lastHasStatus = hasStatus;
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