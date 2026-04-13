#include "Backend_Manager.h"
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include "define.h"
#include "WiFi_Manager.h"

// External globals from various managers for serialization
extern float water_temp_c;
extern int g_co2Ppm;
extern float g_waterLevelPct;
extern float g_laserLevelPct;
extern float g_tankHealthPct;
extern float g_laserHealthPct;
extern float avg_temp_c;
extern float avg_humid_pct;

static bool backendActive = true;
static unsigned long lastPost = 0;
const unsigned long POST_INTERVAL = 60000; // 1 minute

bool isBackendConnected()
{
    return backendActive;
}

void backendSendStatus()
{
    lastPost = millis();

    if (!wifiConnected)
        return;

    WiFiClient client;
    client.setTimeout(15000);

    HTTPClient http;
    String url = CMS_SERVER_URL;
    Serial.printf("[BACKEND] HTTP POST to: %s\n", url.c_str());

    http.begin(client, url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-API-Key", CMS_API_KEY);
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);

    JsonDocument doc;
    doc["deviceId"] = g_deviceId;
    doc["version"] = FIRMWARE_VERSION;
    doc["uptime"] = millis() / 1000;
    doc["state"] = g_currentSystemState;

    JsonObject sensors = doc["sensors"].to<JsonObject>();
    sensors["airTemp"] = avg_temp_c;
    sensors["humidity"] = avg_humid_pct;
    sensors["waterTemp"] = water_temp_c;
    sensors["co2"] = g_co2Ppm;
    sensors["tankLevel"] = g_waterLevelPct;
    sensors["laserLevel"] = g_laserLevelPct;

    JsonObject diagnostics = doc["diag"].to<JsonObject>();
    diagnostics["tankHealth"] = g_tankHealthPct;
    diagnostics["laserHealth"] = g_laserHealthPct;
    diagnostics["rssi"] = WiFi.RSSI();
    diagnostics["heap"] = ESP.getFreeHeap();

    String payload;
    serializeJson(doc, payload);
    Serial.println("[JSON] " + payload);

    int httpCode = http.POST(payload);
    String response = http.getString();

    Serial.printf("[BACKEND] HTTP %d body:%d bytes\n", httpCode, response.length());
    if (httpCode >= 200 && httpCode < 400)
    {
        Serial.printf("[SUCCESS] HTTP %d | %s\n", httpCode, response.substring(0, 100));
        backendActive = true;
    }
    else
    {
        Serial.printf("[FAIL] HTTP %d: %s\n", httpCode, response.substring(0, 100));
        backendActive = false;
    }
    http.end();
}

void backendTask(void *parameter)
{
    while (g_currentSystemState != STATE_CONNECTED)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    Serial.println("Backend: Starting periodic JSON status sends every 60s.");


    for (;;)
    {
        if (wifiConnected)
        {
            unsigned long now = millis();
            if (now - lastPost > POST_INTERVAL)
            {
                backendSendStatus();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void backendInit() {} // Stateless

void backendSendStatus();
bool isBackendConnected();