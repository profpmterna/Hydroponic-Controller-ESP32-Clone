#include "Thermal_Manager.h"
#include <DHT.h>
#include <algorithm>
#include <vector>

DHT dht(PIN_DHT22, DHT22);

float avg_temp_c = 0.0f;
float avg_humid_pct = 0.0f;
float g_heatIndex = 0.0f;
float g_thermalStdDev = 0.0f;
bool dhtEnabled = false;     // Tracks sensor health

static int dhtConsecutiveFails = 0;
const int MAX_DHT_FAILS = 5; // Disable after 10 seconds of failure (5 samples * 2s)

// Buffer for Jitter Analysis
#define DHT_SD_SAMPLES 10
static float dht_temp_samples[DHT_SD_SAMPLES];
static float dht_humid_samples[DHT_SD_SAMPLES];
static int dht_sd_idx = 0;
static bool dht_sd_filled = false;

static unsigned long lastRecoveryAttempt = 0;
const unsigned long RECOVERY_INTERVAL_MS = 600000; // 10 minutes (600,000 ms)

// EMA Filter Constants
#define THERMAL_EMA_ALPHA 0.05f // Reduced from 0.2 for higher stability

void thermalInit()
{
#if !HW_ENABLE_DHT22
  dhtEnabled = false;
  Serial.println(F("Thermal: DHT22 disabled by hardware flag."));
  return;
#endif
  dht.begin();
  dhtEnabled = true;
  Serial.printf("Thermal Manager: DHT22 on GPIO%d initialized\n", PIN_DHT22);

  lastRecoveryAttempt = millis();
}

void thermalReset()
{
  dhtConsecutiveFails = 0;
  dht_sd_filled = false;
  dht_sd_idx = 0;
  dhtEnabled = true;
  Serial.println("Thermal: Flags reset.");
}

void thermalUpdate()
{
  unsigned long now = millis();
  // Check if it's time to try re-initializing failed sensors
  bool isRecoveryTime = (now - lastRecoveryAttempt >= RECOVERY_INTERVAL_MS);
  bool dhtRecoveryActive = isRecoveryTime && !dhtEnabled;

  if (isRecoveryTime) {
    lastRecoveryAttempt = now;
    if (dhtRecoveryActive) {
      Serial.println("Thermal: 10-minute recovery attempt for DHT22...");
    }
  }

  // DHT22 air temp/humidity
  if (dhtEnabled || dhtRecoveryActive) 
  {
    dht.read();
    float temp = dht.readTemperature();
    float humid = dht.readHumidity();

    if (isnan(temp) || isnan(humid))
    {
      if (dhtEnabled) {
        dhtConsecutiveFails++;
        Serial.printf("Thermal: DHT read failed (%d/%d)\n", dhtConsecutiveFails, MAX_DHT_FAILS);

        if (dhtConsecutiveFails >= MAX_DHT_FAILS)
        {
          dhtEnabled = false;
          Serial.println("Thermal: CRITICAL - DHT sensor marked as failed. Disabling for 10 minutes.");
        }
      } else if (dhtRecoveryActive) {
        Serial.println("Thermal: DHT22 recovery read failed. Sensor still unresponsive.");
      }
    }
    else
    {
      // Success: Reset failure counter and re-enable if it was disabled
      if (!dhtEnabled) Serial.println("Thermal: DHT22 Self-Healed!");
      dhtConsecutiveFails = 0;
      dhtEnabled = true;
      
      // --- Step 1: Populate Buffers ---
      dht_temp_samples[dht_sd_idx] = temp;
      dht_humid_samples[dht_sd_idx] = humid;
      dht_sd_idx = (dht_sd_idx + 1) % DHT_SD_SAMPLES;
      if (dht_sd_idx == 0) dht_sd_filled = true;

      // --- Jitter Analysis (Standard Deviation) ---
      int sd_count = dht_sd_filled ? DHT_SD_SAMPLES : dht_sd_idx;
      float sd_sum = 0;
      for (int i = 0; i < sd_count; i++) sd_sum += dht_temp_samples[i];
      float sd_mean = sd_sum / sd_count;

      float sumSqDiff = 0;
      for (int i = 0; i < sd_count; i++) sumSqDiff += sq(dht_temp_samples[i] - sd_mean);
      g_thermalStdDev = sqrt(sumSqDiff / sd_count);

      if (sd_count == DHT_SD_SAMPLES && g_thermalStdDev > DHT_SD_THRESHOLD) {
          Serial.printf("Thermal: WARNING - DHT22 Jitter detected (SD: %.2f C)\n", g_thermalStdDev);
      }

      // --- Step 3: Slew Rate Limiting ---
      float slewTemp = temp;
      float slewHumid = humid;
      if (avg_temp_c > 0) {
          float dT = temp - avg_temp_c;
          if (abs(dT) > DHT_MAX_SLEW_TEMP) 
              slewTemp = avg_temp_c + (dT > 0 ? DHT_MAX_SLEW_TEMP : -DHT_MAX_SLEW_TEMP);
          
          float dH = humid - avg_humid_pct;
          if (abs(dH) > DHT_MAX_SLEW_HUMID)
              slewHumid = avg_humid_pct + (dH > 0 ? DHT_MAX_SLEW_HUMID : -DHT_MAX_SLEW_HUMID);
      }

      // --- Step 4: Final EMA Filtering ---
      if (avg_temp_c == 0.0f) {
          avg_temp_c = slewTemp;
          avg_humid_pct = slewHumid;
      } else {
          avg_temp_c = (THERMAL_EMA_ALPHA * slewTemp) + ((1.0f - THERMAL_EMA_ALPHA) * avg_temp_c);
          avg_humid_pct = (THERMAL_EMA_ALPHA * slewHumid) + ((1.0f - THERMAL_EMA_ALPHA) * avg_humid_pct);
      }

      // Calculate Heat Index (Feels Like temperature) using the filtered values
      g_heatIndex = dht.computeHeatIndex(avg_temp_c, avg_humid_pct, false);
    }
  }
}

void thermalTask(void *parameter)
{
#if !HW_ENABLE_DHT22
  Serial.println(F("Thermal: Disabled by hardware flag. Task terminating."));
  vTaskDelete(NULL);
#endif

  Serial.println("Thermal Task: 0.5Hz (2s) sampling started");
  for (;;)
  {
    // Always call thermalUpdate so the internal 10-minute timer can be checked
    thermalUpdate();
    
    yield();
    vTaskDelay(pdMS_TO_TICKS(2000)); 
  }
}
