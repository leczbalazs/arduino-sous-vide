#include <stdio.h>

#include <EEPROMex.h>
#include <LiquidCrystal.h>
#include <OneButton.h>
#include <PID_v1.h>
#include <Thermistor.h>

// Uncomment to include debugging code
//#define DEBUG 1

#define SERIAL_SPEED 115200

// Logging
#define LOG(x) Serial.print(millis()); Serial.print(": "); Serial.println(x)
#define LOG2(x, y) Serial.print(millis()); Serial.print(": "); Serial.print(x); Serial.println(y)

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

// Default target temperature
#define DEFAULT_TARGET_TEMP 55.0 // [C]

// Interval between temperature measurements
#define SAMPLE_INTERVAL 250 // [ms]

// Control window size (for turning the PID output value into a slow PWM signal)
#define WINDOW_SIZE 5000 // [ms]

// Target temperature boundaries
#define MIN_TARGET_TEMP 30.0 // [C]
#define MAX_TARGET_TEMP 90.0 // [C]

// Temperature setting granularity
#define TEMP_STEP 0.5 // [C]

// Boundaries for determining if temperature readings are sane
#define MIN_TEMP -2.0 // [C] -- We are not expecting ice
#define MAX_TEMP 102.0 // [C] -- We are not expecting superheated water or steam

// Possible states
#define STATE_INIT 0
#define STATE_STOPPED 1
#define STATE_RUNNING 2
// TODO: add states for
// - temperature calibration
// - PID settings (auto-calibration?)

// Possible events
#define EVENT_CLICK 0
#define EVENT_DOUBLECLICK 1
#define EVENT_LONGPRESS 2

// Global variables
unsigned char state = STATE_INIT; // Current system state
unsigned long lastSampleTimestamp; // The timestamp of the last temperature sample that was taken [ms]
double currentTemp; // Current temperature [C]
double targetTemp = DEFAULT_TARGET_TEMP; // Target temperature [C]
unsigned long windowStartTime;
double onDuration; // Time to keep the relay ON [s]
boolean outputState; // Current state of the relay [false: off, true: on]

// Temperature sensor
Thermistor thermistor(PIN_THERMISTOR); // NCT thermistor

// PID Controller
// TODO: move PID parameters to EEPROM and make them modifiable via the config menu
PID pid(&currentTemp, &onDuration, &targetTemp, 2, 5, 1, DIRECT); // PID controller

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

#define BUTTONS 5
OneButton buttons[BUTTONS] = {
  plusButton, minusButton, startButton, stopButton, functionButton
};


// Display related functions
void updateDisplay() {
  // TODO 
}

void saveConfig() {
  // TODO
}

void loadConfig() {
}

void increaseTargetTemp() {
  if (targetTemp < MAX_TARGET_TEMP - TEMP_STEP) {
    targetTemp += TEMP_STEP;
  }
  updateDisplay();
}

void decreaseTargetTemp() {
  if (targetTemp > MIN_TARGET_TEMP + TEMP_STEP) {
    targetTemp -= TEMP_STEP;
  }
  updateDisplay();
}

void handleEvent(char button, char event) {
  switch (state) {
  case STATE_INIT:
    // Don't accept any keypresses during initialization
    break;
  case STATE_STOPPED:
    switch (button) {
    case BUTTON_PLUS:
      increaseTargetTemp();
      break;
    case BUTTON_MINUS:
      decreaseTargetTemp();
      break;
    }
    break;
  }
}


void plusButtonClicked() {
  handleEvent(BUTTON_PLUS, EVENT_CLICK);
}


void minusButtonClicked() {
  handleEvent(BUTTON_MINUS, EVENT_CLICK);
}


void setup() {
  Serial.begin(SERIAL_SPEED);

  // Set pin modes
  // TODO
  
  // Save startup time
  // TODO
  
  // These calibration points are from the thermistor's factory data sheet
  // TODO: load calibration values from EEPROM
  //thermistor.calibrate(25.0, 50.0, 80.0, 10000.0, 3600.55, 1255.5);

  // Results from manual calibration
  thermistor.calibrate(21.2, 59.2, 81.0, 12434.0, 2645.0, 1180.0);
  
  // Set up the display with 16 characters and 2 lines
  lcd.begin(16, 2);
  
  // Set up the PID controller
  pid.SetMode(AUTOMATIC);

  // Attach button handler functions
  plusButton.attachClick(plusButtonClicked);
  minusButton.attachClick(minusButtonClicked);
  // TODO: Attach the rest of the button click handlers

  // Display initialization message on LCD
  // TODO

  // TODO: move this state transition to loop()
  state = STATE_STOPPED;
}  


void loop() {
  unsigned long now = millis();
  
  // Measure the temperature at every sample interval
  if (now > lastSampleTimestamp + SAMPLE_INTERVAL
      || now < lastSampleTimestamp) { // millis() wrapped around
    LOG("Meuasring temperature");
    double reading = thermistor.getTemperatureCelsius();
    lastSampleTimestamp = now;

#ifdef DEBUG
      LOG("ADC value: ");
      LOG(thermistor.getADC());
      LOG("Resistance: ");
      LOG(thermistor.getR());
      LOG("Temperature: ");
      LOG(reading);
#endif

    if (reading > MIN_TEMP && reading < MAX_TEMP) {
      currentTemp = reading;
      pid.Compute();
      LOG2(currentTemp, " C");
    } else {
      LOG2("Temperature outside of accepted range; rejecting value ", reading);
      // TODO: log rejected reading
    }
  }

  // Shift the control window if needed  
  if (now > windowStartTime + WINDOW_SIZE) {
    windowStartTime += WINDOW_SIZE;
  }
  // Set the relay state according to the PID output
  if (now < windowStartTime + onDuration) {
    digitalWrite(PIN_RELAY, HIGH);
  } else {
    digitalWrite(PIN_RELAY,LOW);
  }
  
  // Poll button states
  for (int button = 0; button < BUTTONS; button++) {
    buttons[button].tick();
  }
}

