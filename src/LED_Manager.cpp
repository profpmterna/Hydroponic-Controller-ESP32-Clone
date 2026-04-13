#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "LED_Manager.h"
#include "define.h" // Include define.h for state definitions
#include "Backend_Manager.h"

// Variables for managing LED blinking.
// unsigned long lastBlink = 0; // No longer needed for fading states
// bool blinkState = false; // No longer needed for fading states

// Blink variables
bool blinkState = false;
unsigned long lastBlinkToggle = 0;
const unsigned long BLINK_INTERVAL = 500;      // 1Hz blink
const unsigned long FAST_BLINK_INTERVAL = 200; // 2.5Hz for errors/NTP
static int currentLedState = 0;                // Internal state for the LED task

// Dedicated task to handle LED blinking independently of blocking network calls
void ledTask(void *parameter);
static TaskHandle_t ledTaskHandle = NULL;

// Initializes the LED pins for PWM control.
void ledInit()
{
  // Configure PWM channels for Red, Green, and Blue LEDs.
  // ledcSetup(channel, frequency, resolution_bits)
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_G, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES);

  // Attach PWM channels to their respective GPIO pins.
  // ledcAttachPin(pin, channel)
  ledcAttachPin(PIN_R, PWM_CH_R);
  ledcAttachPin(PIN_G, PWM_CH_G);
  ledcAttachPin(PIN_B, PWM_CH_B);

  // Set initial LED color to off (0,0,0).
  ledSetColor(0, 0, 0);

  // Start the LED task
  xTaskCreate(ledTask, "LED_Task", 2048, NULL, 2, &ledTaskHandle);
}

// Sets the RGB color of the LED.
// r, g, b: 8-bit values (0-255) for Red, Green, and Blue intensity.
void ledSetColor(uint8_t r, uint8_t g, uint8_t b)
{
  // Write the analog value (PWM duty cycle) to each LED channel.
  // The resolution is 8 bits, so values range from 0 to 255.
  ledcWrite(PWM_CH_R, r);
  ledcWrite(PWM_CH_G, g);
  ledcWrite(PWM_CH_B, b);
}

// Blink state machine - hard on/off toggle

// Controls LED blinking patterns based on the current device state.
// state: The current operational state of the device (from define.h).
// now: The current time in milliseconds (from millis()).
void ledBlink(int state, unsigned long now)
{
  static int lastState = -1;
  if (state != lastState)
  {
    lastState = state;
    blinkState = false;
    lastBlinkToggle = now - BLINK_INTERVAL; // Force immediate toggle
  }

  unsigned long blinkInt = (state == STATE_NTP_SYNC || state == STATE_ERROR) ? FAST_BLINK_INTERVAL : BLINK_INTERVAL;
  if (now - lastBlinkToggle >= blinkInt)
  {
    blinkState = !blinkState;
    lastBlinkToggle = now;
  }

  switch (state)
  {
  case STATE_NTP_SYNC:
    // FIXED: Was toggling Red/Green. Now blinks Green/Off for consistency.
    ledSetColor(0, blinkState ? 255 : 0, 0);
    break;
  case STATE_AP:
    ledSetColor(blinkState ? 255 : 0, blinkState ? 255 : 0, 0); // Yellow blink
    break;
  case STATE_CONNECTING:
    ledSetColor(0, 0, blinkState ? 255 : 0); // Blue blink
    break;
  case STATE_CONNECTED:
    if (isBackendConnected()) 
    {
      ledSetColor(0, 255, 0); // Solid green when backend is online
    }
    else
    {
      ledSetColor(blinkState ? 255 : 0, blinkState ? 128 : 0, 0); // Yellow/Orange blink = WiFi OK, Portal Offline
    }
    break;
  case STATE_ERROR:
    ledSetColor(blinkState ? 255 : 0, 0, 0); // Red blink
    break;
  case STATE_OTA_CHECK:
    ledSetColor(blinkState ? 255 : 0, 0, blinkState ? 255 : 0); // Magenta blink
    break;
  case STATE_OTA_UPDATE:
    ledSetColor(blinkState ? 255 : 0, blinkState ? 255 : 0, blinkState ? 255 : 0); // White blink
    break;
  default:
    ledSetColor(0, 0, 0);
    break;
  }
}

/**
 * Dedicated task for LED updates to ensure consistency.
 */
void ledTask(void *parameter)
{
  int receivedState;
  const TickType_t xResolution = pdMS_TO_TICKS(20); // 20ms resolution for smoother blinking

  for (;;)
  {
    // Block on the queue. If a new state arrives, we react instantly.
    // If not, we wake up every 20ms to handle the blink timing.
    if (xQueueReceive(stateQueue, &receivedState, xResolution) == pdTRUE)
    {
      currentLedState = receivedState;      // Update internal state
      g_currentSystemState = receivedState; // Update the global authoritative state
    }

    ledBlink(currentLedState, millis());
  }
}
