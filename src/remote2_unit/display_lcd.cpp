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

static void drawHeader(uint8_t mode, bool linkAlive)
{
    tft.fillRect(0, 0, 240, 40, modeColor(mode));

    tft.setTextWrap(false);
    drawCenteredText(modeText(mode), 120, 10, 3, ST77XX_BLACK);
}

static void drawFooter(const StatusPacket &status, bool linkAlive, uint32_t buttonMask)
{
    // Footer top row
    tft.fillRect(0, 220, 240, 30, COLOR_BG);

    // Footer bottom/debug row
    //tft.fillRect(0, 250, 240, 30, COLOR_BG);

    

    tft.setTextSize(2);
    tft.setTextColor(COLOR_DIM);
    tft.setCursor(10, 230);
    tft.print("S ");
    tft.print(status.satellites);
    tft.print("/");
    tft.print(status.satellitesInView);

    const bool gpsOk = linkAlive && ((status.flags & STATUS_FLAG_GPS_VALID) != 0);

    drawCenteredText("GPS", 120, 230, 2, gpsOk ? COLOR_GOOD : COLOR_BAD);

    tft.setCursor(180, 230);
    tft.setTextColor(linkAlive ? COLOR_GOOD : COLOR_BAD);
    tft.print(linkAlive ? "LINK" : "LOST");

    tft.setTextSize(2);

    tft.fillRect(20, 250, 60, 30, COLOR_BG);

    tft.setTextColor(COLOR_ACCENT);
    tft.setCursor(25, 260);
    tft.print("R");
    tft.print(status.counter);

    tft.fillRect(90, 250, 60, 30, COLOR_BG);

    tft.setTextColor(COLOR_WARN);
    tft.setCursor(90, 260);
    tft.print("BH");
    tft.print(headingDisplayDeg(status.headingDeg10));

    tft.fillRect(165, 250, 75, 30, COLOR_BG);

    tft.setCursor(165, 260);
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
    static bool lastHasStatus = false;
    static bool lastLinkAlive = false;
    static uint32_t lastButtonMask = 0;
    static uint8_t lastMode = 255;
    static uint8_t lastManualThrustPct = 255;
    static uint8_t lastTargetSpeedPct = 255;
    static uint16_t lastTargetHeadingDeg10 = 65535;
    static uint8_t lastSatellites = 255;
    static uint8_t lastFlags = 255;
    static int8_t lastSteerState = 99;
    static uint8_t lastCalFlags = 255;
    static uint16_t lastCalBucketMask = 65535;
    static uint8_t lastCalPhase = 255;
    static uint8_t lastSatellitesInView = 255;
    static uint8_t lastCounter = 255;
    static uint8_t lastMotorTiltUnsafe = 255;
    static uint16_t lastHeadingBucket = 65535;
    static uint16_t lastMotorHeadingBucket = 65535;

    const bool sameScreenData =
        !firstDraw &&
        (hasStatus == lastHasStatus) &&
        (linkAlive == lastLinkAlive) &&
        (buttonMask == lastButtonMask) &&
        (status.mode == lastMode) &&
        (status.manualThrustPct == lastManualThrustPct) &&
        (status.targetSpeedPct == lastTargetSpeedPct) &&
        (status.targetHeadingDeg10 == lastTargetHeadingDeg10) &&
        (status.satellites == lastSatellites) &&
        (status.flags == lastFlags) &&
        (status.steerState == lastSteerState) &&
        (status.calFlags == lastCalFlags) &&
        (status.calBucketMask == lastCalBucketMask) &&
        (status.satellitesInView == lastSatellitesInView) &&
        (status.counter == lastCounter) &&
        (status.motorTiltUnsafe == lastMotorTiltUnsafe) &&
        (headingDisplayDeg(status.headingDeg10) == lastHeadingBucket) &&
        (headingDisplayDeg(status.motorHeadingDeg10) == lastMotorHeadingBucket) &&
        (status.calPhase == lastCalPhase);

    if (sameScreenData)
    {
        return;
    }

    const bool doFullDraw = firstDraw;
    firstDraw = false;

    const bool modeChanged = (status.mode != lastMode);
    const bool statusChanged = (hasStatus != lastHasStatus);
    const bool motorTiltChanged = (status.motorTiltUnsafe != lastMotorTiltUnsafe);

    lastHasStatus = hasStatus;
    lastLinkAlive = linkAlive;
    lastButtonMask = buttonMask;
    lastMode = status.mode;
    lastManualThrustPct = status.manualThrustPct;
    lastTargetSpeedPct = status.targetSpeedPct;
    lastTargetHeadingDeg10 = status.targetHeadingDeg10;
    lastSatellites = status.satellites;
    lastSatellitesInView = status.satellitesInView;
    lastCounter = status.counter;
    lastFlags = status.flags;
    lastSteerState = status.steerState;
    lastCalFlags = status.calFlags;
    lastCalBucketMask = status.calBucketMask;
    lastCalPhase = status.calPhase;
    lastMotorTiltUnsafe = status.motorTiltUnsafe;
    lastHeadingBucket = headingDisplayDeg(status.headingDeg10);
    lastMotorHeadingBucket = headingDisplayDeg(status.motorHeadingDeg10);

    if (doFullDraw)
    {
        tft.fillScreen(COLOR_BG);
    }
    else
    {
        if (modeChanged || statusChanged || motorTiltChanged)
        {
            tft.fillRect(0, 40, 240, 180, COLOR_BG);
        }
    }

    if (!hasStatus)
    {
        tft.fillRect(0, 0, 240, 40, COLOR_BAD);

        drawCenteredText("NO DATA", 120, 10, 3, ST77XX_BLACK);

        drawCenteredText("WAITING FOR", 120, 90, 3, COLOR_TEXT);
        drawCenteredText("MAIN UNIT", 120, 130, 3, COLOR_TEXT);
        

        return;
    }

    const bool otaActive =
        (status.flags & STATUS_FLAG_OTA_ACTIVE) != 0;

    // Header
    if (otaActive)
    {
        tft.fillRect(0, 0, 240, 40, COLOR_CAL);
        drawCenteredText("OTA", 120, 10, 3, ST77XX_BLACK);
    }
    else if (calActive || calComplete)
    {
        tft.fillRect(0, 0, 240, 40, COLOR_CAL);
        drawCenteredText("CAL", 120, 10, 3, ST77XX_BLACK);
    }
    else
    {
        drawHeader(status.mode, linkAlive);
    }

    // OTA screen
    if (otaActive)
    {
        drawCenteredText("OTA", 120, 82, 5, COLOR_CAL);
        drawCenteredText("UPDATE MODE", 120, 155, 2, COLOR_TEXT);
        drawCenteredText("192.168.4.1", 120, 195, 2, COLOR_TEXT);

        drawFooter(status, linkAlive, buttonMask);
        return;
    }

    // CAL screen
    if (calActive || calComplete)
    {
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
                {
                    count++;
                }
            }

            const char *phaseText = "--";

            if (calPhase == 1)
                phaseText = "CW";
            else if (calPhase == 2)
                phaseText = "CCW";

            snprintf(calLine1, sizeof(calLine1), "%s %u/16", phaseText, count);
        }

        drawCenteredText(calLine1, 120, 82, 5, COLOR_CAL);

        char spdLine[32];
        snprintf(spdLine, sizeof(spdLine), "SPD %.1f", status.gpsSpeedCmps / 100.0f);
        drawCenteredText(spdLine, 120, 155, 3, COLOR_TEXT);

        char cogLine[32];
        snprintf(cogLine, sizeof(cogLine), "COG %u", status.gpsCogDeg10 / 10);
        drawCenteredText(cogLine, 120, 195, 3, COLOR_TEXT);

        drawFooter(status, linkAlive, buttonMask);
        return;
    }

    // Main content
    if (status.mode == 0) // STOP
    {
        drawCenteredText("STOP", 120, 90, 5, COLOR_STOP);

        if (status.motorTiltUnsafe)
        {
            drawCenteredText("MOTOR UP", 120, 160, 3, COLOR_STOP);
        }
        else
        {
            drawCenteredText("MOTOR OK", 120, 160, 3, COLOR_GOOD);
        }
    }
    else if (status.mode == 1) // MANUAL
    {
        char line1[32];
        snprintf(line1, sizeof(line1), "THR %u%%", status.manualThrustPct);

        tft.fillRect(0, 55, 240, 65, COLOR_BG);
        drawCenteredText(line1, 120, 72, 5, COLOR_MANUAL);

        tft.fillRect(0, 130, 240, 70, COLOR_BG);
        drawSteerIndicator(status.steerState, 120, 145);
    }
    else if (status.mode == 2) // AUTO
    {
        char line1[32];
        snprintf(line1, sizeof(line1), "SPD %u%%", status.targetSpeedPct);
        drawCenteredText(line1, 120, 62, 5, COLOR_AUTO);

        char line2[32];
        snprintf(line2, sizeof(line2), "HDG %u", status.targetHeadingDeg10 / 10);
        drawCenteredText(line2, 120, 145, 3, COLOR_TEXT);
    }
    else if (status.mode == 3) // ANCHOR
    {
        drawCenteredText("ANCHOR", 120, 62, 4, COLOR_ANCHOR);

        char line2[32];
        snprintf(line2, sizeof(line2), "HDG %u", status.targetHeadingDeg10 / 10);
        drawCenteredText(line2, 120, 135, 3, COLOR_TEXT);

        drawCenteredText("POSITION HOLD", 120, 182, 2, COLOR_DIM);
    }
    else
    {
        drawCenteredText("UNKNOWN", 120, 90, 4, COLOR_WARN);
    }

    drawFooter(status, linkAlive, buttonMask);
}