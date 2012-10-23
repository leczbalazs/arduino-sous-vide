#include <stdio.h>

#include <EEPROMex.h>
#include <LiquidCrystal.h>
#include <OneButton.h>
#include <PID_v1.h>
#include <Thermistor.h>

// TODOs
// - implement PID tuning menu
// - implement saving/loading PID parameters
// - balance resistor value setting menu
// - temperature calibration menu
// - PID auto-tune
// - refactor constants to express tuning as in http://freshmealssolutions.com/downloads/PID-tuning-guide_R2_V006.pdf
// - anti-windup setting (% of proportional band)
// - deviation stats (min/max/avg/std-dev/90 percentile)
// - factory reset
// - manual relay on/off
// - debug
// - only start timer when temperature reaches the target temp 


// Uncomment to include debugging code
//#define DEBUG 1

#define SERIAL_SPEED 115200

// Configuration data address in the EEPROM
#define CONFIG_ADDRESS 0
// Configuration data states
#define CONFIG_VALID 1
#define CONFIG_INVALID 2

// Pin assignments
#define PIN_PLUS_BUTTON 2
#define PIN_MINUS_BUTTON 3
#define PIN_START_BUTTON 4
#define PIN_STOP_BUTTON 5
#define PIN_FUNCTION_BUTTON 6
#define PIN_THERMISTOR A0
#define PIN_RELAY A1
#define PIN_LCD_RS 7
#define PIN_LCD_EN 8
#define PIN_LCD_D4 9
#define PIN_LCD_D5 10
#define PIN_LCD_D6 11
#define PIN_LCD_D7 12
#define PIN_STATUS_LED 13

// Default target temperature
#define DEFAULT_TARGET_TEMP 55.0 // [C]

// Interval between two updates of the current temperature value
#define UPDATE_INTERVAL 1000 // [ms]

// Number of samples to take between two temperature updates
// This many samples are kept in the moving window
#define N_SAMPLES 100 // max: 255
// The length of the moving window for smoothing temperature measurements
#define MOVING_WINDOW_SIZE 10000 // [ms]
// The interval between two temperature measurements
#define SAMPLE_INTERVAL (MOVING_WINDOW_SIZE / N_SAMPLES) // [ms]

// Control window size (for turning the PID output value into a slow PWM signal)
#define WINDOW_SIZE 5000 // [ms]

// Minimum time the relay is allowed to turn on
// There's no point in turning on a mechanical relay for less than 200 ms
#define MIN_RELAY_DURATION 200 // [ms]

// Target temperature boundaries
#define MIN_TARGET_TEMP 20.0 // [C]
#define MAX_TARGET_TEMP 90.0 // [C]

// Temperature setting granularity
#define TEMP_STEP 0.5 // [C]

// Boundaries for determining whether temperature readings are sane
#define MIN_TEMP 0.1 // [C] -- We are not expecting ice
#define MAX_TEMP 99.9 // [C] -- We are not expecting superheated water or steam

// Maximum minutes on the timer
#define MAX_TIME 999 // [minutes]
#define MINUTE 60000 // [ms]

// Possible states
#define STATE_INIT 0
#define STATE_MAIN 1
#define STATE_MENU 2
#define STATE_CALIBRATION 3
#define STATE_PID_SETTING 4

// Possible events
#define EVENT_CLICK 0
#define EVENT_DOUBLECLICK 1
#define EVENT_LONGPRESS 2

// What the PLUS/MINUS buttons should be adjusting in STATE_MAIN
#define ADJUST_TEMP 0
#define ADJUST_TIMER 1

// Number of main menu items
#define N_MENU_ITEMS 2

// Logging
#define LOG(x) Serial.print(millis()); Serial.print(": "); Serial.println(x)
#define LOG2(x, y) Serial.print(millis()); Serial.print(": "); Serial.print(x); Serial.println(y)


//
// Global state variables
//
unsigned char state = STATE_INIT; // Current system state
boolean running = false; // Whether the controller/timer is running
unsigned long now; // Current time (time elapsed since power-on) [ms]
unsigned char adjustTarget = ADJUST_TEMP; // What the PLUS/MINUS buttons are adjusting
unsigned long lastSampleTimestamp; // The timestamp of the last temperature sample that was taken [ms]
unsigned long lastUpdateTimestamp; // The timestamp of the last temperature update [ms]
double currentTemp; // Current temperature [C]
double targetTemp = DEFAULT_TARGET_TEMP; // Target temperature [C]
int timer = -1; // Current number of minutes on the timer (negative values mean the timer is disabled) [min]
unsigned long windowStartTime;
double onDuration; // Time to keep the relay ON [s]
boolean relayState; // Current state of the relay [false: off, true: on]
boolean updateNeeded; // Whether a display update is required
unsigned long lastTick; // Timestamp of the last timer decrement [ms]

double samples[N_SAMPLES]; // Temperature samples (circular buffer)
unsigned char currentSample = 0; // The index of the latest temperature sample in the circular buffer

unsigned char selectedMenuItem = 0; // The index of the currently selected menu item

//
// Global objects
//
// Temperature sensor
Thermistor thermistor(PIN_THERMISTOR); // NCT thermistor

// PID Controller
// This gives +/-0.3C on my crock pot
PID pid(&currentTemp, &onDuration, &targetTemp, 2000.0, 1.0, 100.0, DIRECT); // PID controller

// LCD Display
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

// Buttons
#define BUTTON_PLUS 0
#define BUTTON_MINUS 1
#define BUTTON_START 2
#define BUTTON_STOP 3
#define BUTTON_FUNCTION 4
OneButton plusButton(PIN_PLUS_BUTTON, true, REPEAT);
OneButton minusButton(PIN_MINUS_BUTTON, true, REPEAT);
OneButton startButton(PIN_START_BUTTON, true, NO_REPEAT);
OneButton stopButton(PIN_STOP_BUTTON, true, NO_REPEAT);
OneButton functionButton(PIN_FUNCTION_BUTTON, true, NO_REPEAT);


void relayOn() {
  if (!relayState) {
    updateNeeded = true;
  }
  relayState = true;
  digitalWrite(PIN_RELAY, HIGH);
  digitalWrite(PIN_STATUS_LED, HIGH);
}


void relayOff() {
  if (relayState) {
    updateNeeded = true;
  }
  relayState = false;
  digitalWrite(PIN_RELAY, LOW);
  digitalWrite(PIN_STATUS_LED, LOW);
}


// Main loop
void loop() {
  now = millis();

  // Decrement the timer
  if (timer > 0 && running) {
    if (now > (lastTick + MINUTE)) {
      lastTick += MINUTE;
      timer--;
      updateNeeded = true;
      if (timer == 0) {
        // Reached the preset time
        running = false;
        relayOff();
        timer = -1;
      }
    } 
  }

  // Measure the temperature at every sample interval
  if (now > lastSampleTimestamp + SAMPLE_INTERVAL
      || now < lastSampleTimestamp) { // millis() wrapped around
    sampleTemperature();
  }
  
  // Update the temperature and the PID controller
  if (now > lastUpdateTimestamp + UPDATE_INTERVAL
      || now < lastUpdateTimestamp) { // millis() wrapped around
    updateTemperature();
    LOG2("Current temperature (moving average) [C]: ", currentTemp);
    LOG2("Error [C]: ", targetTemp - currentTemp);
    pid.Compute();
    LOG2("Relay ON duration calculated by PID controller [ms]: ", onDuration);
    if (onDuration < MIN_RELAY_DURATION) {
      onDuration = 0;
      LOG2("Corrected relay ON duration [ms]: ", onDuration);
    }
    if (onDuration > WINDOW_SIZE - MIN_RELAY_DURATION) {
      onDuration = WINDOW_SIZE;
      LOG2("Corrected relay ON duration [ms]: ", onDuration);
    }
    
    // Force a minimum output, to keep the temperature from falling too quickly.
    // This has to be slightly more than what's required to keep the temperature constant.
    onDuration += 1200.0;
    
    // Dirty hack to keep the heating element warm and avoid a large under-shoot
    if (currentTemp > targetTemp + 0.05) {
      onDuration = 200.0;
    }
    updateNeeded = true; 
  }

  // Shift the control window if needed  
  if (now > windowStartTime + WINDOW_SIZE) {
    windowStartTime += WINDOW_SIZE;
  }

  // Switch the relay on/off
  if (running) {
    // Set the relay state according to the PID output
    if (now <= windowStartTime + onDuration) {
      relayOn();
    } else {
      relayOff();
    }
  }
  
  // Poll button states
  plusButton.tick();
  minusButton.tick();
  startButton.tick();
  stopButton.tick();
  functionButton.tick();

  // Update the display if needed
  if (updateNeeded) {
    updateDisplay();
    updateNeeded = false;
  }
}

