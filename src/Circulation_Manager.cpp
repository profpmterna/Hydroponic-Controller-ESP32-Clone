#include "Circulation_Manager.h"
#include "Tank_Manager.h"
#include <Arduino.h>
#include <time.h>

bool g_circPumpRunning = false;
bool g_circPumpEnabled = true; // Manual software override

static unsigned long manualCircUntil = 0;

void triggerManualCirc()
{
    manualCircUntil = millis() + (CIRC_PUMP_DURATION_MINS * 60000UL);
    Serial.printf("Circulation: Manual %d-minute cycle triggered.\n", CIRC_PUMP_DURATION_MINS);
}

void circInit()
{
#if !HW_ENABLE_CIRC_PUMP
    g_circPumpEnabled = false;
    Serial.println("Circulation: Disabled by hardware flag.");
    return;
#endif
    pinMode(PIN_CIRC_PUMP, OUTPUT);
    digitalWrite(PIN_CIRC_PUMP, LOW);
    Serial.println("Circulation Manager: Initialized (GPIO15 Pump)");
}

void circUpdate()
{
    // Safety Interlock Logic:
    // The pump should ONLY run if:
    // 1. Manual enable is true (user wants it on).
    // 2. Tank sensor is healthy (if disabled, we can't verify safety).
    // 3. No dry run risk (water level is above suction + safety buffer).
    // 4. Time matches the hourly schedule OR a manual trigger is active.

    struct tm timeinfo;
    bool timeValid = getLocalTime(&timeinfo);
    
    // Check if we are within the scheduled window (Start of hour)
    // If time is not yet synced via NTP, we treat it as unscheduled to be safe.
    bool isScheduled = timeValid && (timeinfo.tm_min < CIRC_PUMP_DURATION_MINS);

    // Check if manual override is active
    bool isManualActive = (manualCircUntil > 0 && millis() < manualCircUntil);
    
    // Clean up manual trigger if time has expired
    if (manualCircUntil > 0 && millis() >= manualCircUntil) {
        manualCircUntil = 0;
    }

    // Safety Interlocks:
    // 1. Level: Sensor must be healthy and water must be above suction + buffer.
    // 2. Temp: Sensor must be healthy and water must be below max safe threshold.
    bool isLevelSafe = tankSensorEnabled && !g_tankDryRunRisk;
    bool isTempSafe = ds18b20Enabled && (water_temp_c < CIRC_PUMP_MAX_TEMP);
    bool shouldRun = g_circPumpEnabled && isLevelSafe && isTempSafe && (isScheduled || isManualActive);

    if (shouldRun)
    {
        if (!g_circPumpRunning)
        {
            digitalWrite(PIN_CIRC_PUMP, HIGH);
            g_circPumpRunning = true;
            Serial.println("Circulation: Safety interlock PASSED. Pump ON.");
        }
    }
    else
    {
        if (g_circPumpRunning)
        {
            digitalWrite(PIN_CIRC_PUMP, LOW);
            g_circPumpRunning = false;

            if (!g_circPumpEnabled)
                Serial.println("Circulation: Manual STOP.");
            else if (!tankSensorEnabled)
                Serial.println("Circulation: Tank sensor fail STOP.");
            else if (!ds18b20Enabled)
                Serial.println("Circulation: Water temp sensor fail STOP.");
            else if (g_tankDryRunRisk)
                Serial.println("Circulation: CRITICAL LOW WATER STOP.");
            else if (water_temp_c >= CIRC_PUMP_MAX_TEMP)
                Serial.printf("Circulation: HIGH TEMPERATURE STOP (%.1f C).\n", water_temp_c);
            else if (isManualActive && (!isLevelSafe || !isTempSafe))
                Serial.println("Circulation: Manual cycle aborted due to safety risk.");
            else if (!isScheduled)
                Serial.println("Circulation: Hourly cycle complete. Entering standby.");
        }
    }
}

void circTask(void *parameter)
{
#if !HW_ENABLE_CIRC_PUMP
    Serial.println(F("Circulation: Disabled by hardware flag. Task terminating."));
    vTaskDelete(NULL);
#endif

    circInit();
    for (;;)
    {
        circUpdate();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check safety every 1 second
    }
}