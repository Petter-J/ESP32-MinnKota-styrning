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

static void drawFooter(const StatusPacket &status, bool linkAlive)
{
    // 🔥 rensa footer innan vi ritar nytt
    tft.fillRect(0, 240, 240, 40, COLOR_BG);

    tft.drawFastHLine(0, 240, 240, COLOR_DIM);

    tft.setTextSize(2);
    tft.setTextColor(COLOR_DIM);
    tft.setCursor(10, 250);
    tft.print("SAT ");
    tft.print(status.satellites);

    drawCenteredText(
        (status.flags & STATUS_FLAG_GPS_VALID) ? "GPS OK" : "NO GPS",
        120, 250, 2,
        (status.flags & STATUS_FLAG_GPS_VALID) ? COLOR_GOOD : COLOR_BAD);

    tft.setCursor(180, 250);
    tft.setTextColor(linkAlive ? COLOR_GOOD : COLOR_BAD);
    tft.print(linkAlive ? "LINK" : "LOST");
}
// =====================================================
// Public API
// =====================================================
void display_lcd_begin()
{
    Serial.println("[LCD] begin");

    ledcSetup(0, 5000, 8);        // channel 0, 5kHz, 8-bit
    ledcAttachPin(LCD_BL, 0);
    ledcWrite(0, 120);            // 0–255 (120 ≈ 50%)

    SPI.begin(LCD_SCLK, -1, LCD_MOSI, LCD_CS);

    tft.init(240, 280);
    tft.setRotation(0);
    tft.fillScreen(COLOR_BG);
    tft.setTextWrap(false);

    drawCenteredText("REMOTE 2", 120, 90, 3, COLOR_TEXT);
    drawCenteredText("DISPLAY READY", 120, 140, 2, COLOR_ACCENT);

    Serial.println("[LCD] init done");
}

void display_lcd_update(const StatusPacket& status, bool hasStatus, uint32_t buttonMask, bool linkAlive)
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
        (status.steerState == lastSteerState);

    if (sameScreenData)
    {
        return;
    }

    const bool doFullDraw = firstDraw;
    firstDraw = false;

    lastHasStatus = hasStatus;
    lastLinkAlive = linkAlive;
    lastButtonMask = buttonMask;
    lastMode = status.mode;
    lastManualThrustPct = status.manualThrustPct;
    lastTargetSpeedPct = status.targetSpeedPct;
    lastTargetHeadingDeg10 = status.targetHeadingDeg10;
    lastSatellites = status.satellites;
    lastFlags = status.flags;
    lastSteerState = status.steerState;

    if (doFullDraw)
    {
        tft.fillScreen(COLOR_BG);
    }
    else
    {
        tft.fillRect(0, 36, 240, 204, COLOR_BG);
    }

    if (!hasStatus)
    {
        tft.fillRect(0, 0, 240, 36, COLOR_BAD);

        tft.setTextColor(ST77XX_BLACK);
        tft.setTextSize(2);
        tft.setCursor(10, 10);
        tft.print("NO DATA");

        drawCenteredText("WAITING FOR", 120, 90, 3, COLOR_TEXT);
        drawCenteredText("MAIN UNIT", 120, 130, 3, COLOR_TEXT);
        drawCenteredText(linkAlive ? "LINK OK" : "LINK LOST", 120, 190, 2, linkAlive ? COLOR_GOOD : COLOR_BAD);

        return;
    }

    drawHeader(status.mode, linkAlive);

    // Main content
    if (status.mode == 0) // STOP
    {
        drawCenteredText("STOP", 120, 90, 5, COLOR_STOP);
        drawCenteredText("SYSTEM IDLE", 120, 160, 2, COLOR_DIM);
    }
    else if (status.mode == 1) // MANUAL
    {
        char line1[32];
        snprintf(line1, sizeof(line1), "THR %u%%", status.manualThrustPct);
        drawCenteredText(line1, 120, 72, 5, COLOR_MANUAL);

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

    drawFooter(status, linkAlive);
}