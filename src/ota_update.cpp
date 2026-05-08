#include "ota_update.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

static WebServer otaServer(80);
static bool gOtaActive = false;

static const char *OTA_SSID = "BoatControl-OTA";
static const char *OTA_PASS = "12345678";

static String otaPage()
{
    String html;

    html += "<!doctype html><html><head>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<title>ESP32 OTA</title>";
    html += "</head><body>";

    html += "<h2>ESP32 OTA Update</h2>";

    html += "<p><b>AP SSID:</b> ";
    html += OTA_SSID;
    html += "</p>";

    html += "<p><b>AP IP:</b> ";
    html += WiFi.softAPIP().toString();
    html += "</p>";

    html += "<p><b>MAC:</b> ";
    html += WiFi.macAddress();
    html += "</p>";

    html += "<form method='POST' action='/update' enctype='multipart/form-data'>";
    html += "<input type='file' name='update'><br><br>";
    html += "<input type='submit' value='Upload'>";
    html += "</form>";

    html += "</body></html>";

    return html;
}

void ota_begin()
{
    if (gOtaActive)
        return;

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(OTA_SSID, OTA_PASS);

    otaServer.on("/", HTTP_GET, []()
                 { otaServer.send(200, "text/html", otaPage()); });

    otaServer.on(
        "/update",
        HTTP_POST,
        []()
        {
            otaServer.send(
                200,
                "text/plain",
                Update.hasError()
                    ? "Update failed"
                    : "Update OK. Rebooting...");

            delay(1500);
            ESP.restart();
        },
        []()
        {
            HTTPUpload &upload = otaServer.upload();

            if (upload.status == UPLOAD_FILE_START)
            {
                Update.begin(UPDATE_SIZE_UNKNOWN);
            }
            else if (upload.status == UPLOAD_FILE_WRITE)
            {
                Update.write(upload.buf, upload.currentSize);
            }
            else if (upload.status == UPLOAD_FILE_END)
            {
                Update.end(true);
            }
        });

    otaServer.begin();
    gOtaActive = true;
}

void ota_handle()
{
    if (!gOtaActive)
        return;

    otaServer.handleClient();
}

bool ota_is_active()
{
    return gOtaActive;
}