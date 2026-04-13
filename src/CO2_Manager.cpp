#include "CO2_Manager.h"
#include "define.h"
#include <HardwareSerial.h>

int g_co2Ppm = 0;
int g_co2Temp = 0;
float g_co2StdDev = 0.0f;
bool co2Enabled = false;
bool co2WarmedUp = false;

static HardwareSerial mhzSerial(2); // Use UART2

static int co2ConsecutiveFails = 0;
const int MAX_CO2_FAILS = 5;
static int co2JitterHits = 0;
static unsigned long lastCo2Recovery = 0;
const unsigned long CO2_RECOVERY_INTERVAL = 600000; // 10 minutes
const int CO2_MAX_SAFE_TEMP = 55;                   // Safety threshold in Celsius

const int CO2_AVG_SAMPLES = 20;
int co2_samples[CO2_AVG_SAMPLES];
int co2_sample_idx = 0;
bool co2_samples_filled = false;

// Adaptive EMA Constants
#define CO2_SLOW_ALPHA 0.15f   // Very smooth for stable environments
#define CO2_FAST_ALPHA 0.80f   // Very responsive for large changes
#define CO2_JUMP_THRESHOLD 250 // PPM difference to trigger fast response
#define CO2_MAX_SLEW_PPM 500   // Max allowable PPM change per 5s sample

void co2Init()
{
#if !HW_ENABLE_MHZ19
    co2Enabled = false;
    Serial.println("CO2: Disabled by hardware flag.");
    return;
#endif
    mhzSerial.begin(9600, SERIAL_8N1, PIN_MHZ_RX, PIN_MHZ_TX);
    mhzSerial.setTimeout(150); // Prevent long task blocks if sensor is unplugged
    co2Enabled = true;
    lastCo2Recovery = millis();
    Serial.println("CO2 Manager: MH-Z19E Initialized on UART2");
}

void co2Reset()
{
    co2ConsecutiveFails = 0;
    co2Enabled = true;
    co2_samples_filled = false;
    co2_sample_idx = 0;
    Serial.println("CO2: Sensor flags reset.");
}

void co2PerformBurnIn()
{
    Serial.println("CO2: Initiating automated burn-in/reset sequence...");
    co2WarmedUp = false; // Force re-warmup period
    co2JitterHits = 0;
    // We don't call calibrateZero() automatically as it requires 400ppm environment
    // but we can trigger a sensor reset command.
    Serial.println("CO2: Reset complete. Sensor in re-stabilization phase.");
}

void co2Update()
{
    unsigned long now = millis();
    bool isRecoveryTick = (now - lastCo2Recovery >= CO2_RECOVERY_INTERVAL);
    bool co2RecoveryActive = isRecoveryTick && !co2Enabled;

    if (isRecoveryTick)
    {
        lastCo2Recovery = now;
    }

    if (co2Enabled || co2RecoveryActive)
    {
        // Raw command to read CO2: {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79}
        byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
        mhzSerial.write(cmd, 9);
        
        byte response[9];
        memset(response, 0, 9);
        int bytesRead = mhzSerial.readBytes(response, 9);

        // Checksum validation
        byte check = 0;
        for (int i = 1; i < 8; i++) check += response[i];
        check = 0xFF - check + 0x01;

        if (bytesRead != 9 || response[0] != 0xFF || response[1] != 0x86 || response[8] != check)
        {
            if (co2Enabled)
            {
                co2ConsecutiveFails++;
                Serial.printf("CO2: Communication failed! (%d/%d)\n", co2ConsecutiveFails, MAX_CO2_FAILS);
                if (co2ConsecutiveFails >= MAX_CO2_FAILS)
                {
                    co2Enabled = false;
                    Serial.println("CO2: CRITICAL - Sensor disabled for recovery period.");
                }
            }
        }
        else
        {
            int currentCo2 = (int)response[2] * 256 + (int)response[3];
            g_co2Temp = (int)response[4] - 40;

            // Check for valid range if communication was successful
            if ((currentCo2 < 300 || currentCo2 > 5000) && !(currentCo2 == 0 && millis() < 30000))
            {
                // Sensor is responding but providing invalid data
                return;
            }

            // Success
            if (!co2Enabled)
                Serial.println("CO2: Sensor Self-Healed!");
            co2Enabled = true;
            co2ConsecutiveFails = 0;

            // Check warmup status (Library provides this based on time since boot)
            // Usually ~3 minutes for stability
            if (!co2WarmedUp && millis() > 180000)
            {
                co2WarmedUp = true;
                Serial.println("CO2: Sensor warm-up complete.");
            }

            // Safety Check: Overheat Protection
            if (g_co2Temp > CO2_MAX_SAFE_TEMP)
            {
                co2Enabled = false;
                Serial.printf("CO2: CRITICAL - Sensor disabled due to high internal temperature (%d C)\n", g_co2Temp);
                co2ConsecutiveFails = MAX_CO2_FAILS; // Ensure it stays disabled until recovery interval
                return;
            }

            // 1. Slew Rate Limit (Pre-filtering)
            // Prevent impossible jumps from affecting the filters immediately.
            int processedCo2 = currentCo2;
            if (g_co2Ppm > 0 && co2WarmedUp)
            {
                int delta = currentCo2 - g_co2Ppm;
                if (abs(delta) > CO2_MAX_SLEW_PPM)
                {
                    processedCo2 = g_co2Ppm + (delta > 0 ? CO2_MAX_SLEW_PPM : -CO2_MAX_SLEW_PPM);
                    Serial.printf("CO2: Slew limit active. Raw: %d, Clamped to: %d\n", currentCo2, processedCo2);
                }
            }

            // 2. Update buffer for Jitter/SD Analysis using the slewed value
            co2_samples[co2_sample_idx] = processedCo2;
            co2_sample_idx = (co2_sample_idx + 1) % CO2_AVG_SAMPLES;
            if (co2_sample_idx == 0)
                co2_samples_filled = true;

            // 3. Calculate EMA for the reported global value
            if (g_co2Ppm == 0)
            {
                g_co2Ppm = processedCo2; // Seed the filter
            }
            else
            {
                // Calculate the delta and choose alpha dynamically
                float diff = abs(processedCo2 - g_co2Ppm);
                float currentAlpha = (diff > CO2_JUMP_THRESHOLD) ? CO2_FAST_ALPHA : CO2_SLOW_ALPHA;

                g_co2Ppm = (int)((currentAlpha * processedCo2) + ((1.0f - currentAlpha) * g_co2Ppm));
            }

            // 4. Calculate Mean and Standard Deviation for Health Check
            long sum = 0;
            int count = co2_samples_filled ? CO2_AVG_SAMPLES : co2_sample_idx;
            for (int i = 0; i < count; i++)
                sum += co2_samples[i];
            float avg = (float)sum / count;
            float sumSqDiff = 0;
            for (int i = 0; i < count; i++)
            {
                sumSqDiff += sq((float)co2_samples[i] - avg);
            }
            g_co2StdDev = sqrt(sumSqDiff / count);

            if (count == CO2_AVG_SAMPLES && g_co2StdDev > CO2_SD_THRESHOLD)
            {
                co2JitterHits++;
                Serial.printf("CO2: WARNING - High Jitter detected (SD: %.2f PPM, Hits: %d)\n", g_co2StdDev, co2JitterHits);

                // If jitter persists for 12 consecutive samples (1 minute), perform burn-in
                if (co2JitterHits >= 12)
                {
                    co2PerformBurnIn();
                }
            }
            else
            {
                if (co2JitterHits > 0)
                    co2JitterHits--; // Slow decay of hit counter
            }
        }
    }
}

void co2Task(void *parameter)
{
#if !HW_ENABLE_MHZ19
    Serial.println(F("CO2: Disabled by hardware flag. Task terminating."));
    vTaskDelete(NULL);
#endif

    Serial.println("CO2 Task: Monitoring CO2 levels started");
    // Initial delay to allow UART stability
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (;;)
    {
        co2Update();
        uint32_t delayMs = g_stressTestActive ? 100 : 5000;
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}