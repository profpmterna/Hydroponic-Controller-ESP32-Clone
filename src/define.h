#ifndef DEFINE_H
#define DEFINE_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h> // Required for QueueHandle_t
#include <WiFi.h>
#include <Preferences.h>
// No need <String.h>, Arduino.h covers

// Global variables, declared as extern to be defined in a single .cpp file (e.g., NTP_Manager.cpp or main.cpp).
// Geographic latitude.
extern String g_lat;
// Geographic longitude.
extern String g_lon;
// Current epoch time (seconds since Jan 1, 1970).
extern time_t g_epochTime;
// Current UTC time string.
extern String g_utcTime;
// Current local time string.
extern String g_localTime;
// Timezone name (e.g., "Europe/London").
extern String g_timezone;

// Current operational state of the device.
// Replaced by a queue for inter-task communication and a global for current status.
extern QueueHandle_t stateQueue;
extern volatile int g_currentSystemState;

// Define various states for the device's operation.
#define STATE_NTP_SYNC 0
#define STATE_AP 1
#define STATE_CONNECTING 2
#define STATE_CONNECTED 3
#define STATE_ERROR 4
#define STATE_OTA_CHECK 5
#define STATE_OTA_UPDATE 6
// NVS Preferences object.
extern Preferences prefs;
// Flag indicating if WiFi is currently connected.
extern bool wifiConnected;
// Global flag for Safe Mode status
// Counter for NTP synchronization retries.
extern int ntpRetryCount;
// Local firmware version string.
extern String localOtaVersion;
// Timestamp of the last OTA check.

extern time_t lastOtaCheck;
extern String g_deviceId;

// Default WiFi credentials, used if no saved credentials are found.

#define DEFAULT_SSID "COSYFARM"
#define DEFAULT_PASS "12345678"

// The current version of the firmware. This is used to compare against the remote version
// to decide if an update is required.
#define FIRMWARE_VERSION "1.0.0"

#define OTA_VERSION_URL "https://raw.githubusercontent.com/PrathameshMestry/CosyFarm-ESP32/main/version.txt"
#define OTA_FIRMWARE_URL "https://raw.githubusercontent.com/PrathameshMestry/CosyFarm-ESP32/main/firmware.bin"
#define OTA_CHECK_INTERVAL 86400UL // How often to check for updates (in seconds, e.g., 24 hours)

// Pin definitions for the RGB LED.
#define PIN_R 2
#define PIN_G 1
#define PIN_B 3

// PWM (Pulse Width Modulation) settings for the RGB LED.
#define PWM_FREQ 100000
#define PWM_RES 8
#define PWM_CH_R 0 // PWM channel for Red LED
#define PWM_CH_G 1 // PWM channel for Green LED
#define PWM_CH_B 2 // PWM channel for Blue LED

// NTP (Network Time Protocol) settings.
#define NTP_TIMEOUT_MS 3000

#define PIN_DHT22 6
#define PIN_DS18B20 7

extern float avg_temp_c;
extern float avg_humid_pct;
extern float g_heatIndex;
extern float g_thermalStdDev;
#define DHT_SD_THRESHOLD 1.0f    // Jitter threshold for DHT instability
#define DHT_MAX_SLEW_TEMP 2.0f   // Max degrees change per 2s
#define DHT_MAX_SLEW_HUMID 5.0f  // Max humidity change per 2s
extern bool dhtEnabled;
extern bool ds18b20Enabled;

// CO2 Management
#define PIN_MHZ_RX 17            // MH-Z19 RX Pin (Connect to Sensor TX)
#define PIN_MHZ_TX 18            // MH-Z19 TX Pin (Connect to Sensor RX)
extern int g_co2Ppm;
extern int g_co2Temp;
extern float g_co2StdDev;
extern bool co2Enabled;
extern bool co2WarmedUp;
#define CO2_SD_THRESHOLD 150.0f  // Jitter threshold for MH-Z19E instability

// Tank Management Settings
#define PIN_TANK_TRIG 10
#define PIN_TANK_ECHO 11

// Laser TOF Management (VL53L0X)
#define PIN_TOF_SDA 4
#define PIN_TOF_SCL 5
extern float g_laserDistanceCm;
extern float g_laserLevelPct;
extern float g_laserStdDev;
extern bool laserEnabled;
#define TOF_SD_THRESHOLD 1.5f      // Jitter threshold for Laser instability
#define TOF_JUMP_THRESHOLD 1.2f    // Change in cm to trigger fast EMA response
#define TOF_MEDIAN_SAMPLES 7       // Samples for median burst
#define TOF_MAX_SPREAD_CM 2.0f     // Max difference between min/max in a burst

// Water surface LaserTOF stability (reflections, refraction)
#define WATER_REFRACTIVE_INDEX 1.33f
#define TOF_REFLECTION_THRESHOLD_CM 8.0f  // Reject median < this as reflection
#define TOF_WATER_COR

// Calibration for a 30cm Tank
#define TANK_HEIGHT_CM 30.0f      // Physical height of the tank
#define TANK_MOTOR_SUCTION_CM 12.0f // Height of suction mouth from bottom
#define TANK_SAFETY_BUFFER_CM 2.0f  // Safety buffer above suction to trigger warning early

#define TANK_FULL_DIST_CM 2.0f    // 100% Usable (2cm from sensor = 28cm water) - Range extended for testing
#define TANK_EMPTY_DIST_CM 30.0f   // 0% = tank empty (sensor at 30cm height)
#define TANK_TOTAL_VOLUME_L 150.0f // Total capacity in Liters

#define TANK_SD_THRESHOLD 2.5f    // Standard Deviation threshold for jitter detection
#define TANK_MAX_SLEW_CM 10.0f    // Max allowable distance change per 5s sample
#define TANK_HEALTH_THRESHOLD 70.0f // Trigger alert if health is below this
#define TANK_HEALTH_EMA_ALPHA 0.1f  // Heavy LPF for health stability (0.1 = 10% new, 90% old)
#define TANK_HEALTH_WIPE_MS 3600000UL // Duration required to trigger alert (1 hour)

extern float g_waterLevelPct;
extern float g_waterDistanceCm;
extern float g_waterVolumeL;
extern bool tankSensorEnabled;
extern bool g_tankDryRunRisk;
extern float g_tankHealthPct;

// Circulation Management
#define PIN_CIRC_PUMP 15
#define CIRC_PUMP_DURATION_MINS 5 // Run for the first 5 minutes of every hour
#define CIRC_PUMP_MAX_TEMP 40.0f  // Max safe water temperature for pump operation
extern bool g_circPumpRunning;
extern bool g_circPumpEnabled;

// AC Condensate Management
#define PIN_AC_FLOAT 8
#define PIN_AC_PUMP 9
#define AC_FLOAT_DEBOUNCE_MS 3000UL // 3 sec
#define AC_EMPTY_DEBOUNCE_MS 1000UL // 1 sec debounce for empty detection
#define AC_PUMP_RUN_TIME_MS 90000UL // 90 seconds max runtime watchdog
extern float g_acWaterPumpedToday; // Ensure this is visible to main.cpp

#endif // DEFINE_H