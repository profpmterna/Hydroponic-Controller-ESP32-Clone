#include "Tank_Manager.h"
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <vector>
#include <algorithm>

float g_waterLevelPct = 0.0f;
float g_waterDistanceCm = 0.0f;
float g_waterVolumeL = 0.0f;
bool tankSensorEnabled = false;
bool g_tankDryRunRisk = false;
float g_tankStdDev = 0.0f;      // Track sensor noise/precision
float g_tankHealthPct = 100.0f; // 100% means all pings in burst succeeded

OneWire oneWire(PIN_DS18B20);
DallasTemperature waterSensors(&oneWire);

float water_temp_c = 0.0f;
bool ds18b20Enabled = false;
static int ds18b20ConsecutiveFails = 0;
const int MAX_DS18B20_FAILS = 5;

// Buffer for Ultrasonic Jitter Analysis
#define TANK_AVG_SAMPLES 5
float tank_samples[TANK_AVG_SAMPLES];
int tank_sample_idx = 0;
bool tank_samples_filled = false;

static int tankConsecutiveFails = 0;
const int MAX_TANK_FAILS = 5;
static unsigned long lastTankRecovery = 0;
const unsigned long TANK_RECOVERY_INTERVAL = 600000; // 10 minutes

// Constants for filtering
#define TANK_MEDIAN_SAMPLES 5    // Streamlined for faster cycles
#define TANK_EMA_SLOW 0.15f      // Very smooth for stable water
#define TANK_EMA_FAST 0.80f      // Very responsive for pumping/filling
#define TANK_JUMP_THRESHOLD 1.5f // Change in cm to trigger fast response
#define WATER_TEMP_EMA_ALPHA 0.2f
static float filteredDistance = -1.0f;
static unsigned long healthDropStartTime = 0;
static unsigned long lastWipeAlertTime = 0;

void tankInit()
{
#if HW_ENABLE_ULTRASONIC
    pinMode(PIN_TANK_TRIG, OUTPUT);
    pinMode(PIN_TANK_ECHO, INPUT_PULLDOWN);
    digitalWrite(PIN_TANK_TRIG, LOW);
    tankSensorEnabled = true;
    Serial.println("Tank Manager: HC-SR04 Initialized");
#else
    tankSensorEnabled = false;
    Serial.println("Tank Manager: HC-SR04 masked (Disabled)");
#endif

#if HW_ENABLE_DS18B20
    waterSensors.begin();
    ds18b20Enabled = true;
    Serial.printf("Tank Manager: DS18B20 on GPIO%d, devices: %d\n", PIN_DS18B20, waterSensors.getDeviceCount());
#else
    ds18b20Enabled = false;
    Serial.println("Tank Manager: DS18B20 masked (Disabled)");
#endif

    lastTankRecovery = millis();
}

void tankReset()
{
    tankConsecutiveFails = 0;
    ds18b20ConsecutiveFails = 0;
    tankSensorEnabled = true;
    ds18b20Enabled = true;
    g_tankHealthPct = 100.0f;
    tank_samples_filled = false;
    tank_sample_idx = 0;
    Serial.println("Tank: Sensor flags reset.");
}

void tankUpdate()
{
    unsigned long now = millis();
    bool isRecoveryTick = (now - lastTankRecovery >= TANK_RECOVERY_INTERVAL);

    // Define specific recovery flags for each sensor
    bool tankRecoveryActive = isRecoveryTick && !tankSensorEnabled;
    bool dsRecoveryActive = isRecoveryTick && !ds18b20Enabled;

    if (isRecoveryTick)
    {
        lastTankRecovery = now; // Reset global timer every 10 minutes
    }

    if (tankRecoveryActive)
    {
        Serial.println("Tank: 10-minute recovery attempt for HC-SR04...");
    }
    if (dsRecoveryActive)
    {
        Serial.println("Tank: 10-minute recovery attempt for DS18B20...");
        waterSensors.begin();
        Serial.printf("Tank: DS18B20 recovery found %d devices\n", waterSensors.getDeviceCount());
    }

    // --- HC-SR04 Ultrasonic Update ---
    if (tankSensorEnabled || tankRecoveryActive)
    {
        std::vector<float> validReadings;

        // Calculate speed of sound once per burst instead of inside the loop
        float speedOfSound = (avg_temp_c > 0) ? (331.3f + (0.606f * avg_temp_c)) / 10000.0f : 0.0343f;

        // Take a burst of samples for the Median Filter
        for (int i = 0; i < TANK_MEDIAN_SAMPLES; i++)
        {
            // Latch-up Prevention: If Echo is already HIGH, pulseIn will fail immediately.
            if (digitalRead(PIN_TANK_ECHO) == HIGH)
            {
                pinMode(PIN_TANK_ECHO, OUTPUT);
                digitalWrite(PIN_TANK_ECHO, LOW);
                delayMicroseconds(100);
                pinMode(PIN_TANK_ECHO, INPUT_PULLDOWN);
                delayMicroseconds(10);
            }

            digitalWrite(PIN_TANK_TRIG, LOW);
            delayMicroseconds(2);
            digitalWrite(PIN_TANK_TRIG, HIGH);
            delayMicroseconds(10);
            digitalWrite(PIN_TANK_TRIG, LOW);

            // Use a 30ms timeout (approx 5 meters)
            long duration = pulseIn(PIN_TANK_ECHO, HIGH, 30000);

            if (duration > 0)
            {
                float d = (duration * speedOfSound) / 2.0f;
                // Only accept physically plausible readings (2cm to 400cm)
                if (d >= 2.0f && d <= 400.0f)
                    validReadings.push_back(d);
            }

            vTaskDelay(pdMS_TO_TICKS(60));
        }

        float distance = -1.0f;
        float burstHealth = (validReadings.size() * 100.0f) / TANK_MEDIAN_SAMPLES;

        // Require at least 3 valid pings for a reliable median
        if (validReadings.size() >= 3)
        {
            std::sort(validReadings.begin(), validReadings.end());
            float medianDist = validReadings[validReadings.size() / 2];
            float burstSpread = validReadings.back() - validReadings.front();

            // Use the median if the spread is within reason (10cm default for stability)
            if (burstSpread < 10.0f)
            {
                distance = medianDist;
            }
            else
            {
                Serial.printf("Tank: Burst Rejected - High Spread (%.2f cm)\n", burstSpread);
            }
        }

        // --- Signal Quality (Health) Calculation ---
        Serial.printf("TANK DEBUG: burst_valid=%d/%d (%.0f%% health)\n", (int)validReadings.size(), TANK_MEDIAN_SAMPLES, burstHealth);
        g_tankHealthPct = (TANK_HEALTH_EMA_ALPHA * burstHealth) + ((1.0f - TANK_HEALTH_EMA_ALPHA) * g_tankHealthPct); // LPF for stability

        // --- Sensor Wipe Alert Logic ---
        if (g_tankHealthPct < TANK_HEALTH_THRESHOLD)
        {
            if (healthDropStartTime == 0)
            {
                healthDropStartTime = now;
            }
            else if (now - healthDropStartTime >= TANK_HEALTH_WIPE_MS)
            {
                // Health has been low for more than an hour. Alert every 15 mins.
                if (now - lastWipeAlertTime >= 900000UL || lastWipeAlertTime == 0)
                {
                    Serial.println("Tank: ALERT - Signal quality has been below 70% for over an hour. Please wipe the sensor face!");
                    lastWipeAlertTime = now;
                }
            }
        }
        else if (g_tankHealthPct >= (TANK_HEALTH_THRESHOLD + 5.0f))
        {
            // Hysteresis: Only reset the degradation timer if health recovers significantly (e.g., 75%)
            // This prevents "flickering" where a single lucky ping resets an hour-long countdown.
            healthDropStartTime = 0;
            lastWipeAlertTime = 0;
        }

        // --- Persistence Check Logic ---
        if (distance <= 0 || distance > 400.0f)
        {
            // This median-filtered burst failed
            if (tankSensorEnabled)
            {
                tankConsecutiveFails++;
                Serial.printf("Tank: Persistence Check - Failure %d/%d\n", tankConsecutiveFails, MAX_TANK_FAILS);

                if (tankConsecutiveFails >= MAX_TANK_FAILS)
                {
                    tankSensorEnabled = false;
                    Serial.println("Tank: CRITICAL - 5 consecutive median-filtered failures. Disabling sensor.");
                }
            }
            else if (tankRecoveryActive)
            {
                Serial.println("Tank: Recovery attempt failed. Sensor remains disabled.");
            }
        }
        else
        {
            // Success: Any valid median reading resets the persistence counter
            if (!tankSensorEnabled)
                Serial.println("Tank: Sensor Self-Healed!");
            tankSensorEnabled = true;
            tankConsecutiveFails = 0;

            // --- Filtering Stage 1: Slew Rate Limit (Bypassed for testing) ---
            // This filter is temporarily disabled to allow the system to reach the actual
            // distance more quickly during sensor validation.
            float processedDist = distance;
            // if (g_waterDistanceCm > 0)
            // {
            //     float delta = distance - g_waterDistanceCm;
            //     if (abs(delta) > TANK_MAX_SLEW_CM)
            //     {
            //         processedDist = g_waterDistanceCm + (delta > 0 ? TANK_MAX_SLEW_CM : -TANK_MAX_SLEW_CM);
            //     }
            // }

            // --- Filtering Stage 2: Adaptive EMA ---
            if (g_waterDistanceCm <= 0)
            {
                g_waterDistanceCm = processedDist; // Seed the filter
            }
            else
            {
                // Increase alpha if level is moving rapidly (pumping/filling)
                float diff = abs(processedDist - g_waterDistanceCm);
                float currentAlpha = (diff > TANK_JUMP_THRESHOLD) ? TANK_EMA_FAST : TANK_EMA_SLOW;
                g_waterDistanceCm = (currentAlpha * processedDist) + (1.0f - currentAlpha) * g_waterDistanceCm;
            }

            // --- Moving Average Stage 2: Rolling SMA Buffer ---
            // This buffer is used for Standard Deviation (Jitter) analysis
            tank_samples[tank_sample_idx] = g_waterDistanceCm;
            tank_sample_idx = (tank_sample_idx + 1) % TANK_AVG_SAMPLES;
            if (tank_sample_idx == 0)
                tank_samples_filled = true;

            int count = tank_samples_filled ? TANK_AVG_SAMPLES : tank_sample_idx;

            // 1. Calculate Mean (Average)
            float sum = 0;
            for (int i = 0; i < count; i++)
                sum += tank_samples[i];
            float avgDist = sum / count;

            // 2. Calculate Standard Deviation (Precision Check)
            float sumSqDiff = 0;
            for (int i = 0; i < count; i++)
            {
                sumSqDiff += sq(tank_samples[i] - avgDist);
            }
            g_tankStdDev = sqrt(sumSqDiff / count);

            // 3. Reliability Diagnostic
            if (count == TANK_AVG_SAMPLES && g_tankStdDev > TANK_SD_THRESHOLD)
            {
                Serial.printf("Tank: WARNING - High Jitter (SD: %.2f cm). Sensor face may be dirty, wet, or too close to water.\n", g_tankStdDev);
            }

            // --- Usable Water Percentage ---
            // 0% = 18cm distance (12cm water), 100% = 5cm distance (25cm water)
            float pct = (TANK_EMPTY_DIST_CM - avgDist) / (TANK_EMPTY_DIST_CM - TANK_FULL_DIST_CM) * 100.0f;
            g_waterLevelPct = constrain(pct, 0.0f, 100.0f);

            // --- Physical Volume Calculation ---
            // Volume is calculated based on absolute height from bottom (30cm total)
            float currentHeightCm = TANK_HEIGHT_CM - avgDist;
            g_waterVolumeL = constrain(currentHeightCm * (TANK_TOTAL_VOLUME_L / TANK_HEIGHT_CM), 0.0f, TANK_TOTAL_VOLUME_L);

            // --- Dry Run Protection Flag (with Safety Buffer) ---
            g_tankDryRunRisk = (currentHeightCm <= (TANK_MOTOR_SUCTION_CM + TANK_SAFETY_BUFFER_CM));
        }
    }

    // --- DS18B20 Water Temperature Update ---
    if (ds18b20Enabled || dsRecoveryActive)
    {
        waterSensors.requestTemperatures();
        float wtemp = waterSensors.getTempCByIndex(0);

        if (isnan(wtemp) || wtemp == DEVICE_DISCONNECTED_C)
        {
            if (ds18b20Enabled)
            {
                ds18b20ConsecutiveFails++;
                Serial.printf("Tank: DS18B20 read failed (%d/%d)\n", ds18b20ConsecutiveFails, MAX_DS18B20_FAILS);
                if (ds18b20ConsecutiveFails >= MAX_DS18B20_FAILS)
                {
                    ds18b20Enabled = false;
                    Serial.println("Tank: CRITICAL - DS18B20 marked as failed.");
                }
            }
            else if (dsRecoveryActive)
            {
                Serial.println("Tank: DS18B20 recovery failed.");
            }
        }
        else
        {
            if (!ds18b20Enabled)
                Serial.println("Tank: DS18B20 Self-Healed!");
            ds18b20ConsecutiveFails = 0;
            ds18b20Enabled = true;

            // --- EMA Filtering for Water Temp Stability ---
            if (water_temp_c <= 0.0f)
            {
                water_temp_c = wtemp; // Initial seed
            }
            else
            {
                water_temp_c = (WATER_TEMP_EMA_ALPHA * wtemp) + ((1.0f - WATER_TEMP_EMA_ALPHA) * water_temp_c);
            }
        }
    }
}

void tankTask(void *parameter)
{
#if !HW_ENABLE_ULTRASONIC && !HW_ENABLE_DS18B20
    Serial.println(F("Tank: Both sensors disabled by hardware flag. Task terminating."));
    vTaskDelete(NULL);
#endif

    Serial.println("Tank Task: Monitoring water level every 5s");
    for (;;)
    {
        tankUpdate();
        uint32_t delayMs = g_stressTestActive ? 100 : 5000;
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}