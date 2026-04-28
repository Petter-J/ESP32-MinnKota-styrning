#include "ota_update.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

static WebServer otaServer(80);

static const char *OTA_SSID = "BoatControl-OTA";
static const char *OTA_PASS = "12345678";

void ota_begin()
{
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(OTA_SSID, OTA_PASS);

    Serial.print("[OTA] AP IP: ");
    Serial.println(WiFi.softAPIP());

    otaServer.on("/", HTTP_GET, []()
                 { otaServer.send(200, "text/html",
                                  "<h2>ESP32 OTA Update</h2>"
                                  "<form method='POST' action='/update' enctype='multipart/form-data'>"
                                  "<input type='file' name='update'>"
                                  "<input type='submit' value='Upload'>"
                                  "</form>"); });

    otaServer.on("/update", HTTP_POST, []()
                 {
        otaServer.send(200, "text/plain", Update.hasError() ? "Update failed" : "Update OK. Rebooting...");
        delay(1000);
        ESP.restart(); }, []()
                 {
        HTTPUpload& upload = otaServer.upload();

        if (upload.status == UPLOAD_FILE_START)
        {
            Serial.printf("[OTA] Start: %s\n", upload.filename.c_str());
            Update.begin(UPDATE_SIZE_UNKNOWN);
        }
        else if (upload.status == UPLOAD_FILE_WRITE)
        {
            Update.write(upload.buf, upload.currentSize);
        }
        else if (upload.status == UPLOAD_FILE_END)
        {
            if (Update.end(true))
            {
                Serial.printf("[OTA] Success: %u bytes\n", upload.totalSize);
            }
            else
            {
                Serial.println("[OTA] Failed");
            }
        } });

    otaServer.begin();
    Serial.println("[OTA] server started");
}

void ota_handle()
{
    otaServer.handleClient();
}