#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <WiFi.h>
#include <Preferences.h>
#include "OTA_Manager.h"
#include "LED_Manager.h" // for ledSetColor if direct

// Globals (definitions for externs in define.h)
String localOtaVersion = "";
time_t lastOtaCheck = 0;
bool otaInProgress = false;
TaskHandle_t otaTaskHandle = NULL;
String targetOtaMd5 = ""; // Stores the MD5 hash of the pending update (if provided)

bool isOtaInProgress()
{
  return otaInProgress;
}

void otaCheckAfterNtp()
{
#if !HW_ENABLE_OTA
  Serial.println(F("OTA: Disabled by hardware flag."));
  return;
#endif

  if (g_currentSystemState != STATE_CONNECTED)
    return;

  Serial.println(F("OTA: Starting version check after NTP..."));
  int newState;

  newState = STATE_OTA_CHECK;
  xQueueSend(stateQueue, &newState, 0);

  // Load local version from prefs
  if (!prefs.begin("ota", false))
  {
    Serial.println(F("OTA: Failed to open prefs"));
    newState = STATE_CONNECTED;
    xQueueSend(stateQueue, &newState, 0);
    return;
  }

  // Check if keys exist before reading to avoid NVS "NOT_FOUND" error logs.
  localOtaVersion = prefs.isKey("ota-version") ? prefs.getString("ota-version") : "0.0.0";
  lastOtaCheck = prefs.isKey("last-check") ? prefs.getLong("last-check") : 0;

  // Initialize local version if default (first boot)
  if (localOtaVersion == "0.0.0")
  {
    localOtaVersion = FIRMWARE_VERSION;
    prefs.putString("ota-version", localOtaVersion);
    Serial.printf("OTA: Initialized local version to %s\n", localOtaVersion.c_str());
  }
  prefs.end();

  Serial.printf("OTA: Local version: %s, Last check: %ld\n", localOtaVersion.c_str(), lastOtaCheck);

  // Check if need to check (daily)
  if (g_epochTime > 0 && (g_epochTime - lastOtaCheck > OTA_CHECK_INTERVAL))
  {
    HTTPClient http;
    http.begin(OTA_VERSION_URL);
    http.addHeader("User-Agent", "ESP32-OTA");
    int code = http.GET();

    if (code == HTTP_CODE_OK)
    {
      String payload = http.getString();
      http.end();
      payload.trim();

      // Expecting format "version:md5" or just "version"
      String remoteVersion = payload;
      targetOtaMd5 = "";

      int separatorIndex = payload.indexOf(':');
      if (separatorIndex != -1)
      {
        remoteVersion = payload.substring(0, separatorIndex);
        targetOtaMd5 = payload.substring(separatorIndex + 1);
        targetOtaMd5.trim();
      }

      Serial.printf("OTA: Remote version: %s\n", remoteVersion.c_str());
      if (targetOtaMd5.length() > 0)
        Serial.printf("OTA: Expected MD5: %s\n", targetOtaMd5.c_str());

      if (remoteVersion.length() > 0 && remoteVersion != localOtaVersion)
      {
        // Simple string compare; assume semantic versioning
        Serial.println(F("OTA: New version found! Starting update..."));
        // Save new version
        prefs.begin("ota", false);
        prefs.putString("ota-version", remoteVersion);
        prefs.end();
        otaInProgress = true; // Set flag before starting task
        newState = STATE_OTA_UPDATE;
        xQueueSend(stateQueue, &newState, 0);
        xTaskCreate(otaUpdateTask, "OTAUpdate", 12288, NULL, 2, &otaTaskHandle); // Increased stack for 4KB buffer
        return;
      }
    }
    else
    {
      // Log an error if fetching the version failed.
      Serial.printf("OTA: Version fetch failed: %d\n", code);
      http.end();
    }
  }
  else
  {
    // Log if no OTA check is needed (either recently checked or NTP time is not yet available).
    Serial.println(F("OTA: No check needed (recent or no time)"));
  }

  // Save the current epoch time as the last OTA check time.
  // This happens regardless of whether an update was found or not,
  // to ensure the interval is respected.
  prefs.begin("ota", false);
  prefs.putLong("last-check", g_epochTime);
  prefs.end();

  // Revert to connected state if no update was initiated.
  newState = STATE_CONNECTED;
  xQueueSend(stateQueue, &newState, 0);
  Serial.println(F("OTA: Check complete - up to date"));
}

// FreeRTOS task for performing the actual firmware update.
// This runs in a separate task to avoid blocking the main loop.
void otaUpdateTask(void *parameter)
{
  Serial.println(F("OTA Task: Starting firmware download..."));

  HTTPClient http;
  http.begin(OTA_FIRMWARE_URL);
  http.addHeader("User-Agent", "ESP32-OTA");

  // Fetch the firmware binary.
  int code = http.GET();
  if (code != HTTP_CODE_OK)
  {
    Serial.printf("OTA: Firmware fetch failed: %d\n", code);
    http.end();
    // Reset OTA flag, set error state, and delete the task.
    otaInProgress = false;
    int newState = STATE_ERROR;
    xQueueSend(stateQueue, &newState, 0);
    vTaskDelete(NULL);
    return;
  }

  int contentLength = http.getSize();
  Serial.printf("OTA: Firmware size: %d bytes\n", contentLength); // Log firmware size.

  // Get the stream to read the firmware data.
  WiFiClient *stream = http.getStreamPtr();

  // VALIDITY CHECK 1: Set expected MD5 if available.
  // If the calculated MD5 doesn't match this at the end, Update.end() will return false.
  if (targetOtaMd5.length() > 0)
  {
    Update.setMD5(targetOtaMd5.c_str());
  }

  // Begin the OTA update process.
  // VALIDITY CHECK 2: Using actual contentLength instead of unknown size.
  size_t updateSize = (contentLength > 0) ? contentLength : UPDATE_SIZE_UNKNOWN;
  if (!Update.begin(updateSize))
  {
    Serial.println(F("OTA: Update begin failed"));
    http.end();
    // Reset OTA flag, set error state, and delete the task.
    int newState = STATE_ERROR;
    xQueueSend(stateQueue, &newState, 0);
    otaInProgress = false;
    vTaskDelete(NULL);
    return;
  }

  uint32_t written = 0; // Keep track of bytes written.
  // Larger buffer (4KB) significantly improves flash write performance
  uint8_t buff[4096];
  size_t len;

  // Read data from the stream and write it to the Update partition.
  while (http.connected() && (len = stream->available()) && (contentLength == -1 || written < (uint32_t)contentLength))
  {
    size_t read_len = min(len, sizeof(buff));
    int read = stream->read(buff, read_len);
    if (read > 0)
    {
      size_t w = Update.write(buff, read);
      written += w;
    }
    yield(); // Allow other tasks to run.
  }

  http.end(); // Close HTTP connection.

  // VALIDITY CHECK 3: Finalize update and verify integrity.
  // Update.end() checks if written bytes match begin() size AND validates the MD5.
  // The 'true' parameter for evenIfRemaining is only needed if size was unknown.
  bool success = (contentLength > 0) ? Update.end() : Update.end(true);

  if (success)
  {
    // Double check that we actually wrote the number of bytes the server promised
    if (contentLength > 0 && written != (uint32_t)contentLength)
    {
      Serial.printf("OTA: Size mismatch! Expected %d, got %u\n", contentLength, written);
      otaInProgress = false; // Clear flag
      int newState = STATE_ERROR;
      xQueueSend(stateQueue, &newState, 0);
      vTaskDelete(NULL);
      return;
    }
    Serial.printf("OTA: Success! Written %u bytes, restarting...\n", written);
    // If successful, restart the ESP32 to boot into the new firmware.
  }
  else
  {
    // If update failed, log the error.
    Serial.printf("OTA: Update failed! Error %d: %s (written %u)\n", Update.getError(), Update.errorString(), written);
    // Reset OTA flag, set error state, and delete the task.
    int newState = STATE_ERROR;
    xQueueSend(stateQueue, &newState, 0);
    otaInProgress = false;
    vTaskDelete(NULL);
    return;
  }

  // If update was successful and device is about to restart, set otaInProgress to false.
  otaInProgress = false;

  // Set the verification flag so the next boot knows it needs to verify WiFi connection.
  prefs.begin("ota", false);
  prefs.putBool("pending-verify", true);
  prefs.end();

  ESP.restart();
}
