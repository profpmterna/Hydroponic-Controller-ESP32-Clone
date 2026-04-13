#include "ACWater_Manager.h"
#include <Arduino.h>

bool g_acPumpRunning = false;
float g_acWaterPumpedToday = 0.0f;
static unsigned long pumpStartTime = 0;
static unsigned long floatTriggerStartTime = 0;
static bool faultDetected = false;

enum ACState
{
    IDLE,
    PUMPING,
    COOLDOWN,
    FAULT
};
static ACState currentACState = IDLE;

void acWaterInit()
{
#if !HW_ENABLE_AC_PUMP
    Serial.println("ACWater: Disabled by hardware flag.");
    return;
#endif
    pinMode(PIN_AC_FLOAT, INPUT_PULLUP);
    pinMode(PIN_AC_PUMP, OUTPUT);
    digitalWrite(PIN_AC_PUMP, LOW);
    Serial.println("ACWater Manager: Initialized (GPIO8 Float Active-Low, GPIO9 Pump)");
}

void acWaterUpdate()
{
    unsigned long now = millis();
    bool floatTriggered = (digitalRead(PIN_AC_FLOAT) == LOW);

    switch (currentACState)
    {
    case IDLE:
        if (floatTriggered)
        {
            if (floatTriggerStartTime == 0)
            {
                floatTriggerStartTime = now; // Mark the first time we saw the trigger
            }
            else if (now - floatTriggerStartTime >= AC_FLOAT_DEBOUNCE_MS)
            {
                // Signal has been stable for the required duration
                Serial.println("ACWater: Tank Full (Stable). Starting pump...");
                digitalWrite(PIN_AC_PUMP, HIGH);
                pumpStartTime = now;
                g_acPumpRunning = true;
                currentACState = PUMPING;
                floatTriggerStartTime = 0;
            }
        }
        else
        {
            floatTriggerStartTime = 0; // Reset timer if signal bounces back to HIGH
        }
        break;

    case PUMPING:
        // --- Dry Run Prevention (Early Exit) ---
        // If the float clears (becomes HIGH) during pumping, the tank is empty.
        // We stop immediately to prevent the pump from running dry.
        if (digitalRead(PIN_AC_FLOAT) == HIGH)
        {
            if (floatTriggerStartTime == 0)
                floatTriggerStartTime = now;
            else if (now - floatTriggerStartTime >= AC_EMPTY_DEBOUNCE_MS)
            {
                digitalWrite(PIN_AC_PUMP, LOW);
                g_acPumpRunning = false;
                Serial.println("ACWater: Tank empty detected early. Stopping pump (Dry Run Protection).");
                g_acWaterPumpedToday += 2.0f; // Count the successful cycle
                currentACState = COOLDOWN;
                pumpStartTime = now; // Set start time for cooldown tracking
                floatTriggerStartTime = 0;
                break; // Exit state processing for this tick
            }
        }
        else
        {
            floatTriggerStartTime = 0;
        }

        // --- Watchdog Timeout (Max Runtime Safety) ---
        // If we reach this timeout, it means the float NEVER went HIGH.
        if (now - pumpStartTime >= AC_PUMP_RUN_TIME_MS)
        {
            digitalWrite(PIN_AC_PUMP, LOW);
            g_acPumpRunning = false;
            floatTriggerStartTime = 0; // Initialize timer for recovery debounce
            currentACState = FAULT;
            Serial.println("ACWater: CRITICAL FAULT - Pump Timeout! Tank failed to drain within max runtime.");
        }
        break;

    case COOLDOWN:
        // 10 second delay to prevent rapid cycling from sensor noise
        if (now - pumpStartTime >= 10000)
        {
            currentACState = IDLE;
        }
        break;

    case FAULT:
        // --- Automatic Recovery ---
        // Stay in fault unless float clears (becomes HIGH) for a stable duration.
        if (!floatTriggered)
        {
            if (floatTriggerStartTime == 0) floatTriggerStartTime = now;
            else if (now - floatTriggerStartTime >= AC_FLOAT_DEBOUNCE_MS)
            {
                Serial.println("ACWater: Fault cleared automatically (Stable HIGH signal detected).");
                currentACState = IDLE;
                floatTriggerStartTime = 0;
            }
        }
        else
        {
            floatTriggerStartTime = 0; // Reset timer if signal bounces back to LOW
        }
        break;
    }
}

void acWaterResetDaily()
{
    g_acWaterPumpedToday = 0.0f;
    Serial.println("ACWater: Daily totalizer reset.");
}

void acWaterTask(void *parameter)
{
#if !HW_ENABLE_AC_PUMP
    Serial.println(F("ACWater: Disabled by hardware flag. Task terminating."));
    vTaskDelete(NULL);
#endif

    acWaterInit();
    for (;;)
    {
        acWaterUpdate();
        // Higher frequency check (200ms) to ensure responsive float detection
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}