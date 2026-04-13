#include "LaserTOF_Manager.h"
#include "define.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <vector>
#include <algorithm>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

float g_laserDistanceCm = 0.0f;
float g_laserLevelPct = 0.0f;
float g_laserStdDev = 0.0f;
float g_laserHealthPct = 100.0f;
bool laserEnabled = false;
bool g_laserReflectionFlag = false;
float g_laserCorrectedDistCm = 0.0f;
uint32_t g_laserI2cTotal = 0;
uint32_t g_laserI2cErrors = 0;
static int laserConsecutiveFails = 0;
const int MAX_LASER_FAILS = 10;

// Buffer for Jitter Analysis
#define TOF_AVG_SAMPLES 10
static float tof_samples[TOF_AVG_SAMPLES];
static int tof_sample_idx = 0;
static bool tof_samples_filled = false;

const float TOF_EMA_SLOW = 0.10f;
const float TOF_EMA_FAST = 0.70f;

void laserInit()
{
#if !HW_ENABLE_TOF
    laserEnabled = false;
    Serial.println(F("Laser TOF: Disabled by hardware flag."));
    return;
#endif
    // Pre-initialization: Ensure pins are driven high to help the I2C bus start cleanly.
    pinMode(PIN_TOF_SDA, INPUT_PULLUP);
    pinMode(PIN_TOF_SCL, INPUT_PULLUP);
    delay(50);

    // Initialize I2C with explicit pins and 400kHz (Fast Mode).
    if (!Wire.begin(PIN_TOF_SDA, PIN_TOF_SCL, 400000))
    {
        Serial.println(F("Laser TOF: I2C hardware initialization failed"));
        laserEnabled = false;
        return;
    }

    // 50ms is the "sweet spot" for ESP32. It covers long clock-stretching and 
    // WiFi-induced latency without hanging the task too long on hardware failure.
    Wire.setTimeOut(50);

    // Check for sensor presence at the default address (0x29) before calling the library.
    // This prevents a flood of internal library I2C error logs if the sensor is missing.
    Wire.beginTransmission(0x29);
    if (Wire.endTransmission() != 0)
    {
        Serial.println(F("Laser TOF: VL53L0X sensor not detected at address 0x29. Checking wiring..."));
        laserEnabled = false;
        return;
    }

    if (!lox.begin(0x29, false, &Wire))
    {
        Serial.println(F("Laser TOF: Failed to boot VL53L0X"));
        laserEnabled = false;
    }
    else
    {
        Serial.println(F("Laser TOF: VL53L0X Initialized"));
        laserEnabled = true;
        // Long range mode for better performance in larger tanks
        lox.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
    }
}

void laserReset()
{
    laserConsecutiveFails = 0;
    laserEnabled = true;
    tof_samples_filled = false;
    g_laserI2cTotal = 0;
    g_laserI2cErrors = 0;
    tof_sample_idx = 0;
    g_laserReflectionFlag = false;
    Serial.println("Laser: Sensor flags reset.");
}

void laserUpdate()
{
    if (!laserEnabled)
        return;

    // Periodically verify the bus is still alive before a burst.
    // This helps catch cases where WiFi interference hung the sensor.
    bool busReady = false;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_laserI2cTotal++;
        Wire.beginTransmission(0x29);
        busReady = (Wire.endTransmission() == 0);
        if (!busReady) g_laserI2cErrors++;
        xSemaphoreGive(i2cMutex);
    } else {
        // The mutex is stuck or held by another high-priority task for too long.
        Serial.println(F("Laser TOF: Mutex Timeout during bus check!"));
        laserConsecutiveFails++;
        if (laserConsecutiveFails >= MAX_LASER_FAILS) 
            laserEnabled = false;
        return;
    }

    if (!busReady)
    {
        laserConsecutiveFails++;
        if (laserConsecutiveFails >= MAX_LASER_FAILS)
        {
            laserEnabled = false;
        }
        return;
    }

    std::vector<float> validReadings;
    VL53L0X_RangingMeasurementData_t measure;

    // Take a burst of samples for the Median Filter
    for (int i = 0; i < TOF_MEDIAN_SAMPLES; i++)
    {
        bool validReading = false;
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            g_laserI2cTotal++;
            lox.rangingTest(&measure, false);
            validReading = (measure.RangeStatus == 0);
            if (measure.RangeStatus == 4) g_laserI2cErrors++; // Status 4 is typically a comms error
            xSemaphoreGive(i2cMutex);
        } else {
            // If we can't get the bus during the burst, we just skip this specific sample
            Serial.println(F("Laser TOF: Mutex Timeout during burst sample!"));
            continue; 
        }

        // Status 0 indicates a high-quality, valid return signal
        if (validReading)
        {
            float distCm = measure.RangeMilliMeter / 10.0f;
            if (distCm > 0)
            {
                validReadings.push_back(distCm);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(60)); // Increased delay for water surface settling to reduce reflections
    }

    // --- Signal Quality (Health) Calculation ---
    float burstHealth = (validReadings.size() * 100.0f) / TOF_MEDIAN_SAMPLES;
    g_laserHealthPct = (0.2f * burstHealth) + (0.8f * g_laserHealthPct); // LPF for stability

    // Ensure we have enough samples to determine a meaningful median
    if (validReadings.size() >= (TOF_MEDIAN_SAMPLES / 2))
    {
        std::sort(validReadings.begin(), validReadings.end());

        float burstMin = validReadings.front();
        float burstMax = validReadings.back();
        float burstSpread = burstMax - burstMin;

        // Ripple Rejection: If the spread is too high, the surface is too unstable
        if (burstSpread > TOF_MAX_SPREAD_CM)
        {
            Serial.printf("Laser: Burst rejected - High ripple noise (Spread: %.2f cm)\n", burstSpread);
            laserConsecutiveFails++;
            return;
        }

        float medianDist = validReadings[validReadings.size() / 2];

        // Water surface stability: Reflection & Refractive index correction
        g_laserReflectionFlag = false;
        if (medianDist < TOF_REFLECTION_THRESHOLD_CM)
        {
            g_laserReflectionFlag = true;
            Serial.printf("Laser: Reflection interference detected (%.1f cm), burst rejected\n", medianDist);
            g_laserHealthPct = max(0.0f, g_laserHealthPct - 10.0f); // Penalty for reflection event
            laserConsecutiveFails++;
            return;
        }

        // Apply refractive index correction for water (apparent depth = real / n)
        g_laserCorrectedDistCm = medianDist / WATER_REFRACTIVE_INDEX;
        Serial.printf("Laser: Refractive corrected: raw %.1f -> corrected %.1f cm (n=%.2f)\n", medianDist, g_laserCorrectedDistCm, WATER_REFRACTIVE_INDEX);

        laserConsecutiveFails = 0;

        // --- Filtering Stage: Adaptive EMA --- (use corrected dist)
        if (g_laserDistanceCm <= 0)
        {
            g_laserDistanceCm = g_laserCorrectedDistCm; // Seed with corrected
        }
        else
        {
            float diff = abs(g_laserCorrectedDistCm - g_laserDistanceCm);
            float currentAlpha = (diff > TOF_JUMP_THRESHOLD) ? TOF_EMA_FAST : TOF_EMA_SLOW;
            g_laserDistanceCm = (currentAlpha * g_laserCorrectedDistCm) + (1.0f - currentAlpha) * g_laserDistanceCm;
        }

        // --- Jitter Analysis (Standard Deviation) ---
        tof_samples[tof_sample_idx] = g_laserDistanceCm;
        tof_sample_idx = (tof_sample_idx + 1) % TOF_AVG_SAMPLES;
        if (tof_sample_idx == 0)
            tof_samples_filled = true;

        int count = tof_samples_filled ? TOF_AVG_SAMPLES : tof_sample_idx;
        float sum = 0;
        for (int i = 0; i < count; i++)
            sum += tof_samples[i];
        float avgDist = sum / count;

        float sumSqDiff = 0;
        for (int i = 0; i < count; i++)
            sumSqDiff += sq(tof_samples[i] - avgDist);
        g_laserStdDev = sqrt(sumSqDiff / count);

        // --- Percentage Calculation ---
        float pct = (TANK_EMPTY_DIST_CM - g_laserDistanceCm) / (TANK_EMPTY_DIST_CM - TANK_FULL_DIST_CM) * 100.0f;
        g_laserLevelPct = constrain(pct, 0.0f, 100.0f);
    }
    else
    {
        laserConsecutiveFails++; // Increment only if all retries failed
        if (laserConsecutiveFails >= MAX_LASER_FAILS)
        {
            laserEnabled = false;
            Serial.println("Laser: CRITICAL - VL53L0X persistent failure.");
        }
    }
}

void laserTask(void *parameter)
{
#if !HW_ENABLE_TOF
    Serial.println(F("Laser TOF: Disabled by hardware flag. Task terminating."));
    vTaskDelete(NULL);
#endif

    Serial.println("Laser Task: Parallel water level monitoring started");
    for (;;)
    {
        laserUpdate();
        uint32_t delayMs = g_stressTestActive ? 100 : 5000;
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}
