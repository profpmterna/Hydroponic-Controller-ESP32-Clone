#include <Arduino.h>
#include <nvs_flash.h>
#include "define.h"
#include "LED_Manager.h"
#include "WiFi_Manager.h"
#include "NTP_Manager.h"
#include "OTA_Manager.h"
#include "RTC_Manager.h"
#include "Command_Manager.h"
#include "Backend_Manager.h"

#include "Thermal_Manager.h"
#include "Tank_Manager.h"
#include "ACWater_Manager.h"
#include "Circulation_Manager.h"
#include "LaserTOF_Manager.h"
#include "CO2_Manager.h"

#include <Preferences.h>
#include <LittleFS.h>
#include <esp_system.h>
#include <esp_efuse.h>

// Global definitions (define.h extern impl)
Preferences prefs;
QueueHandle_t stateQueue;              // FreeRTOS queue for state changes
SemaphoreHandle_t i2cMutex;            // Mutex for I2C bus synchronization
volatile int g_currentSystemState = 0; // Global variable for current system state (updated by LED task)
int ntpRetryCount = 0;
bool wifiConnected = false;
bool g_stressTestActive = false; // Initialize stress test flag
String g_deviceId = "";

#define LOG_FILE_PATH "/system_log.txt"
#define MAX_LOG_SIZE (128 * 1024) // 128KB limit for the log file

// RTC Memory Buffer to reduce Flash wear and preserve logs across resets/sleep
#define RAM_BUF_MAX_SIZE 2048
RTC_DATA_ATTR char rtcLogBuffer[RAM_BUF_MAX_SIZE];
RTC_DATA_ATTR size_t rtcLogIndex = 0;

/**
 * Appends a string to the system log file on LittleFS.
 * Implementation: Uses an RTC-backed buffer to preserve logs across resets.
 */
void logStatusToFile(const char *data, bool forceFlush = false)
{
  size_t dataLen = strlen(data);

  // Safety: If single log entry is larger than buffer, truncate it
  if (dataLen >= RAM_BUF_MAX_SIZE)
    dataLen = RAM_BUF_MAX_SIZE - 1;

  // Check if adding this data would overflow the buffer
  bool willOverflow = (rtcLogIndex + dataLen >= RAM_BUF_MAX_SIZE - 1);

  // Flush to Flash if forced or if an overflow is about to occur
  if ((forceFlush || willOverflow) && rtcLogIndex > 0)
  {
    File file = LittleFS.open(LOG_FILE_PATH, FILE_APPEND);
    if (file)
    {
      if (file.size() > MAX_LOG_SIZE)
      {
        file.close();
        file = LittleFS.open(LOG_FILE_PATH, FILE_WRITE);
      }

      if (file)
      {
        file.write((const uint8_t *)rtcLogBuffer, rtcLogIndex);
        file.close();
        rtcLogIndex = 0;
        rtcLogBuffer[0] = '\0';
      }
    }
  }

  // Store data in the RTC buffer if it fits
  if (rtcLogIndex + dataLen < RAM_BUF_MAX_SIZE - 1)
  {
    memcpy(rtcLogBuffer + rtcLogIndex, data, dataLen);
    rtcLogIndex += dataLen;
    rtcLogBuffer[rtcLogIndex] = '\0'; // Ensure null-termination
  }
}

void systemInfoTask(void *parameter)
{
  static int lastLoggedState = -1;
  static unsigned long lastFlashWrite = 0;
  const unsigned long FLASH_WRITE_INTERVAL = 600000; // 10 minutes

  // Wait for the system to reach a stable connected state
  while (g_currentSystemState != STATE_CONNECTED)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  Serial.println("System Info Task: Stable state reached. Periodic reporting active.");

  for (;;)
  {
    rtcUpdate();

    // Use String with reserve to minimize heap fragmentation while keeping logic simple
    String report = "";
    report.reserve(1024);

    // Calculate uptime
    unsigned long totalSeconds = millis() / 1000;
    int days = totalSeconds / 86400;
    int hours = (totalSeconds % 86400) / 3600;
    int mins = (totalSeconds % 3600) / 60;

    report += "\n--- System Status Report ---\n";
    char line[128];
    snprintf(line, sizeof(line), "Device ID:     %s\n", g_deviceId.c_str());
    report += line;
    snprintf(line, sizeof(line), "Firmware Ver:  %s\n", FIRMWARE_VERSION);
    report += line;
    snprintf(line, sizeof(line), "Uptime:        %dd %dh %dm\n", days, hours, mins);
    report += line;
    report += "Internal RTC:  " + rtcGetLocalTimeStr() + "\n";
    report += "WiFi Status:   ";
    report += (wifiConnected ? "Connected" : "Disconnected");
    report += "\n";
    // Add Thermal Monitoring Data
    if (dhtEnabled)
    {
      snprintf(line, sizeof(line), "Air Temp:      %.1f C, %.1f %% RH (SD: %.2f C)\n", avg_temp_c, avg_humid_pct, g_thermalStdDev);
      report += line;
      snprintf(line, sizeof(line), "Heat Index:    %.1f C\n", g_heatIndex);
      report += line;
      if (g_thermalStdDev > DHT_SD_THRESHOLD)
        report += "DHT Health:    WARNING (High Jitter)\n";
      else
        report += "DHT Health:    Good\n";
    }
    else
    {
      report += "Air Temp:      DHT Error (Disabled)\n";
    }

    if (ds18b20Enabled)
    {
      snprintf(line, sizeof(line), "Water Temp:    %.1f C (DS18B20)\n", water_temp_c);
      report += line;
    }
    else
    {
      report += "Water Temp:    DS18B20 Error\n";
    }

    if (co2Enabled)
    {
      snprintf(line, sizeof(line), "CO2 Level:     %d ppm %s (SD: %.1f)\n", g_co2Ppm, co2WarmedUp ? "" : "(Warming Up)", g_co2StdDev);
      report += line;
      if (g_co2StdDev > CO2_SD_THRESHOLD)
      {
        report += "CO2 Health:    WARNING (High Jitter)\n";
      }
      else
      {
        report += "CO2 Health:    Good\n";
      }

      snprintf(line, sizeof(line), "CO2 Int Temp:  %d C (Secondary)\n", g_co2Temp);
      report += line;
    }
    else if (g_co2Temp > 55)
    {
      snprintf(line, sizeof(line), "CO2 Level:     OVERHEAT (%d C) - Disabled\n", g_co2Temp);
      report += line;
    }
    else
    {
      snprintf(line, sizeof(line), "CO2 Level:     Sensor Error (Disabled)\n");
      report += line;
    }

    if (tankSensorEnabled)
    {
      snprintf(line, sizeof(line), "Water Level:   %.1f %% (%.1f L) [%.1f cm, SD: %.2f cm]\n", g_waterLevelPct, g_waterVolumeL, g_waterDistanceCm, g_tankStdDev);
      report += line;

      // Determine status string based on both Jitter (Precision) and Signal Quality (Health)
      const char *healthStatus = "Good";
      if (g_tankHealthPct < 60.0f)
        healthStatus = "Poor Signal";
      else if (g_tankHealthPct < 90.0f)
        healthStatus = "Degraded";
      else if (g_tankStdDev > TANK_SD_THRESHOLD)
        healthStatus = "Jittery";

      snprintf(line, sizeof(line), "Sensor Health: %.0f %% (%s)\n", g_tankHealthPct, healthStatus);
      report += line;

      if (g_tankDryRunRisk)
      {
        snprintf(line, sizeof(line), "TANK ALERT:    CRITICAL LOW %.1f%% - DRY RUN RISK FOR MOTOR!\n", g_waterLevelPct);
        report += line;
      }
      snprintf(line, sizeof(line), "Circulation:   %s\n", g_circPumpRunning ? "RUNNING" : "STOPPED");
      report += line;
    }
    else
    {
      snprintf(line, sizeof(line), "Water Level:   Sensor Error (Disabled)\n");
      report += line;
    }

    if (laserEnabled)
    {
      snprintf(line, sizeof(line), "Laser Level:   %.1f %% [%.1f cm, SD: %.2f cm]\n", g_laserLevelPct, g_laserDistanceCm, g_laserStdDev);
      report += line;

      const char *lStatus = "Good";
      if (g_laserHealthPct < 60.0f)
        lStatus = "Poor Signal";
      else if (g_laserHealthPct < 90.0f)
        lStatus = "Degraded";
      else if (g_laserStdDev > TOF_SD_THRESHOLD)
        lStatus = "Jittery";

      snprintf(line, sizeof(line), "Laser Health: %.0f %% (%s)\n", g_laserHealthPct, lStatus);
      report += line;

      // Stability Diagnostics for 400kHz validation
      if (g_laserI2cTotal > 0)
      {
        float errorRate = (g_laserI2cErrors * 100.0f) / g_laserI2cTotal;
        snprintf(line, sizeof(line), "I2C Error Rate: %.3f %% (%u failures)\n", errorRate, g_laserI2cErrors);
        report += line;
      }
    }

    snprintf(line, sizeof(line), "AC Pumped:     %.1f L (Today)\n", g_acWaterPumpedToday);
    report += line;

    if (wifiConnected)
    {
      report += "IP Address:    " + WiFi.localIP().toString() + "\n";
      snprintf(line, sizeof(line), "Signal (RSSI): %d dBm\n", WiFi.RSSI());
      report += line;
    }

    snprintf(line, sizeof(line), "Free Heap:     %u KB\nFree PSRAM:    %u KB\nState:         %d\nOTA In Prog:   %s\n",
             ESP.getFreeHeap() / 1024, ESP.getFreePsram() / 1024, g_currentSystemState, isOtaInProgress() ? "Yes" : "No");
    report += line;
    report += "----------------------------\n";

    Serial.print(report);

    bool criticalChange = (g_currentSystemState != lastLoggedState);
    bool intervalReached = (millis() - lastFlashWrite >= FLASH_WRITE_INTERVAL);

    logStatusToFile(report.c_str(), criticalChange || intervalReached);

    if (criticalChange || intervalReached)
      lastFlashWrite = millis();
    lastLoggedState = g_currentSystemState;

    vTaskDelay(pdMS_TO_TICKS(60000));
  }
}

void setup()
{
  Serial.begin(115200);
  delay(2000); // 2sec boot delay is sufficient for most serial monitors
  Serial.println("ESP32-S3 WiFi + RGB LED + NTP + OTA Starting...");
  // Load or generate unique device ID from eFuse MAC (Purely Decimal)
  prefs.begin("device", false);
  g_deviceId = prefs.getString("device-id", "");
  prefs.end();

  // Validate that the stored ID follows the format "COSYFARM-<decimal>"
  bool isValid = g_deviceId.startsWith("COSYFARM-");
  if (isValid) {
    for (size_t i = 9; i < g_deviceId.length(); i++) {
      if (!isDigit(g_deviceId[i])) {
        isValid = false;
        break;
      }
    }
  }

  if (!isValid)
  {
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    uint64_t mac_decimal = 0;
    for (int i = 0; i < 6; i++)
    {
      mac_decimal = (mac_decimal << 8) | mac[i];
    }
    g_deviceId = "COSYFARM-" + String(mac_decimal);
    Serial.printf("Generated Device ID: %s\n", g_deviceId.c_str());
    prefs.begin("device", false);
    prefs.putString("device-id", g_deviceId);
    prefs.end();
  }

  rtcInit(); // Initialize RTC and load Geo-Cache

  // Handle RTC Memory persistence logic
  esp_reset_reason_t reason = esp_reset_reason();
  if (reason == ESP_RST_POWERON || reason == ESP_RST_BROWNOUT)
  {
    // Fresh boot: zero out the index to prevent reading garbage
    rtcLogIndex = 0;
    rtcLogBuffer[0] = '\0';
    Serial.println("RTC Memory: Initialized fresh log buffer.");
  }
  else
  {
    // Software reset or Deep Sleep wake: preserve index and log status
    Serial.printf("RTC Memory: Preserved %u bytes of log data. (Reset Reason: %d)\n", rtcLogIndex, reason);
  }

  // Initialize LittleFS
  if (!LittleFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting LittleFS");
  }
  else
  {
    Serial.println("LittleFS mounted successfully");
  }

  // Print ESP32 Chip and Memory Information
  Serial.println("\n--- Hardware Information ---");
  Serial.printf("Device ID:     %s\n", g_deviceId.c_str());
  Serial.printf("Chip Model:    %s\n", ESP.getChipModel());
  Serial.printf("Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("Cores:         %d\n", ESP.getChipCores());
  Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Flash Size:    %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("Heap Size:     %d KB\n", ESP.getHeapSize() / 1024);
  Serial.printf("Free Heap:     %d KB\n", ESP.getFreeHeap() / 1024);
  Serial.printf("PSRAM Size:    %d KB\n", ESP.getPsramSize() / 1024);
  Serial.printf("Free PSRAM:    %d KB\n", ESP.getFreePsram() / 1024);
  Serial.println("---------------------------\n");

  // Create the state queue
  stateQueue = xQueueCreate(5, sizeof(int)); // Queue can hold 5 integer state messages
  i2cMutex = xSemaphoreCreateMutex();        // Create the I2C mutex

  ledInit();  // Now safe: task starts after queue is ready
  wifiInit(); // Safe: uses queue to signal connecting state

  xTaskCreate(
      wifiMonitorTask,
      "WiFiMonitor",
      4096, // Reduced from 8KB to free up heap for SSL
      NULL,
      1,
      NULL);

  // Create a FreeRTOS task for printing system information every minute
  xTaskCreate(
      systemInfoTask, // Function to be executed by the task
      "SystemInfo",   // Name of the task
      4096,           // 4KB is sufficient for report generation
      NULL,           // Parameter to pass to the task
      1,              // Priority of the task
      NULL);          // Task handle

#if HW_ENABLE_MHZ19
  co2Init();
  xTaskCreate(
      co2Task,
      "CO2Sensor",
      3072, // Reduced from 4KB
      NULL,
      1,
      NULL);
#endif

#if HW_ENABLE_TOF
  laserInit();
  xTaskCreate(
      laserTask,
      "LaserLevel",
      3072, // Reduced from 4KB
      NULL,
      1,
      NULL);
#endif

#if HW_ENABLE_AC_PUMP
  xTaskCreate(
      acWaterTask,
      "ACWaterPump",
      2048,
      NULL,
      1,
      NULL);
#endif

#if HW_ENABLE_ULTRASONIC || HW_ENABLE_DS18B20
  tankInit();
  xTaskCreate(
      tankTask,
      "TankLevel",
      3072, // Reduced from 4KB
      NULL,
      1,
      NULL);
#endif

#if HW_ENABLE_CIRC_PUMP
  xTaskCreate(
      circTask,
      "Circulation",
      2048,
      NULL,
      1,
      NULL);
#endif

#if HW_ENABLE_DHT22
  thermalInit();
  xTaskCreate(
      thermalTask,
      "Thermal",
      3072, // Reduced from 4KB
      NULL,
      1,
      NULL);
#endif

#if HW_ENABLE_BACKEND
  xTaskCreate(
      backendTask,
      "Backend",
      16384, // Increased stack size for SSL/WSS handshake stability
      NULL,
      2, // Increased priority to ensure smooth SSL/WSS handshakes
      NULL);
#endif
} // Close setup()

void loop()
{
  // Logic is handled by FreeRTOS tasks; this remains empty to satisfy the Arduino framework.
}
