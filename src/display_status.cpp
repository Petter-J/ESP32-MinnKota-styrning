#include "display_status.h"

static String modeText(SystemMode mode)
{
    switch (mode)
    {
    case SystemMode::MANUAL:
        return "MANUAL";
    case SystemMode::AUTO:
        return "AUTO";
    case SystemMode::ANCHOR:
        return "ANCHOR";
    case SystemMode::STOP:
        return "STOP";
    default:
        return "UNKNOWN";
    }
}

static String linkText(bool linkAlive)
{
    return linkAlive ? "LINK OK" : "LINK LOST";
}

static String manualArrowFromButtons(uint32_t buttonMask)
{
    const bool left = (buttonMask & buttonBit(ButtonId::STEER_LEFT)) != 0;
    const bool right = (buttonMask & buttonBit(ButtonId::STEER_RIGHT)) != 0;

    if (left && !right)
        return "<--";
    if (right && !left)
        return "-->";
    return "  |";
}

static String calProgressText(
    bool calComplete,
    uint16_t calBucketMask,
    uint8_t calPhase)
{
    if (calComplete)
    {
        return "DONE";
    }

    uint8_t count = 0;

    for (uint8_t i = 0; i < 16; i++)
    {
        if (calBucketMask & (1 << i))
        {
            count++;
        }
    }

    String phaseText = "--";

    if (calPhase == 1)
        phaseText = "CW";
    else if (calPhase == 2)
        phaseText = "CCW";

    return phaseText + " " + String(count) + "/16";
}

DisplayLines buildDisplayLines(
    const SystemState &sys,
    uint32_t buttonMask,
    bool linkAlive,
    bool calActive,
    bool calComplete,
    uint16_t calBucketMask,
    uint8_t calPhase)
{
    DisplayLines out;

    const bool showCal = calActive || calComplete;

    if (showCal)
    {
        out.line1 = "CAL";
        out.line2 = calProgressText(calComplete, calBucketMask, calPhase);
    }
    else
    {
        out.line1 = modeText(sys.mode);
    }

    switch (sys.mode)
    {
    case SystemMode::MANUAL:
    {
        if (!showCal)
        {
            out.line2 = "THR " + String((int)roundf(sys.manualThrustPct)) + "%";
        }

        out.line3 = manualArrowFromButtons(buttonMask);
        break;
    }

    case SystemMode::AUTO:
    {
        if (!showCal)
        {
            out.line2 = "SPD " + String((int)roundf(sys.targetSpeedPct)) + "%";
        }

        out.line3 = "HDG " + String((int)roundf(sys.targetHeadingDeg)) + (char)247;
        break;
    }

    case SystemMode::ANCHOR:
    {
        if (!showCal)
        {
            out.line2 = "SPD " + String((int)roundf(sys.targetSpeedPct)) + "%";
        }

        out.line3 = "HDG " + String((int)roundf(sys.targetHeadingDeg)) + (char)247;
        break;
    }

    case SystemMode::STOP:
    default:
    {
        if (!showCal)
        {
            out.line2 = "THR 0%";
        }

        out.line3 = "  |";
        break;
    }
    }

    out.line4 = linkText(linkAlive);
    return out;
}