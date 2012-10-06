#include <stdio.h>

#include <EEPROMex.h>
#include <LiquidCrystal.h>
#include <OneButton.h>
#include <PID_v1.h>
#include <Thermistor.h>

// Uncomment to include debugging code
//#define DEBUG 1

// TODO: Implement timer

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

// Interval between temperature measurements
#define SAMPLE_INTERVAL 250 // [ms]

// Control window size (for turning the PID output value into a slow PWM signal)
#define WINDOW_SIZE 5000 // [ms]

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
#define STATE_STOPPED 1
#define STATE_RUNNING 2
// TODO: add states for
// - balance resistor value setting
// - temperature calibration
// - PID settings (auto-calibration?)
// - factory reset?
// - manual output on/off?
// - error

// Possible events
#define EVENT_CLICK 0
#define EVENT_DOUBLECLICK 1
#define EVENT_LONGPRESS 2

// What the PLUS/MINUS buttons should be adjusting
#define ADJUST_TEMP 0
#define ADJUST_TIMER 1

// Logging
#define LOG(x) Serial.print(millis()); Serial.print(": "); Serial.println(x)
#define LOG2(x, y) Serial.print(millis()); Serial.print(": "); Serial.print(x); Serial.println(y)


//
// Global state variables
//
unsigned char state = STATE_INIT; // Current system state
unsigned long now; // Current time (time elapsed since power-on) [ms]
unsigned char adjustTarget = ADJUST_TEMP; // What the PLUS/MINUS buttons are adjusting
unsigned long lastSampleTimestamp; // The timestamp of the last temperature sample that was taken [ms]
double currentTemp; // Current temperature [C]
double targetTemp = DEFAULT_TARGET_TEMP; // Target temperature [C]
int timer = -1; // Current number of minutes on the timer (negative values mean the timer is disabled) [min]
unsigned long windowStartTime;
double onDuration; // Time to keep the relay ON [s]
boolean relayState; // Current state of the relay [false: off, true: on]
boolean updateNeeded; // Whether a display update is required
unsigned long lastTick; // Timestamp of the last timer decrement [ms]

//
// Global objects
//
// Temperature sensor
Thermistor thermistor(PIN_THERMISTOR); // NCT thermistor

// PID Controller
// TODO: move PID parameters to EEPROM and make them modifiable via the config menu
PID pid(&currentTemp, &onDuration, &targetTemp, 4.0, 5.0, 0.5, DIRECT); // PID controller

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


// Format a temperature value (format: ##.#C)
// We need this manual implementation as float formatting is not included in libavr used by Arduino
// TODO: check if it's possible to override compilation flags and make it actually include the *printf()
// version that supports floats 
void formatTemp(char* output, double temperature) {
  unsigned int temp_integer =  (int)temperature;
  unsigned int temp_fractional = (((unsigned int)(temperature * 1000) % 1000) + 50) / 100;
  if (temp_fractional == 10) {
    temp_integer++;
    temp_fractional = 0;
  }

  sprintf(output, "%2d.%1dC", temp_integer, temp_fractional);
}


// Display related functions
void updateDisplay() {
  // Fake display on serial
  Serial.println("/================\\");
  Serial.print("|C:"); // Current temperature
  char temp[6];
  formatTemp(temp, currentTemp);
  Serial.print(temp);
  Serial.print(" Out: "); // Output state
  Serial.print(relayState ? "ON " : "OFF");
  Serial.println("|");
  Serial.print("|T:"); // Target temperature
  formatTemp(temp, targetTemp);
  Serial.print(temp);
  Serial.print((adjustTarget == ADJUST_TEMP) ? "<" : ">"); // Adjust target selector
  if (timer >= 0) {
    sprintf(temp, "%3d", timer); // Remaining minutes on timer
    Serial.print(temp);
  } else {
    Serial.print("___");
  }
  switch (state) {
    case STATE_INIT:
      Serial.print(" INIT");
      break;
    case STATE_STOPPED:
      Serial.print(" STOP");
      break;
    case STATE_RUNNING:
      Serial.print("  RUN");
      break;
  }
  Serial.println("|");
  Serial.println("\\================/");
  // TODO: print output to LCD
}


// Configuration data layout in EEPROM:
//
// - Config state (char)
// - Rbalance (unsigned long)
// - T1 (double)
// - T2 (double)
// - T3 (double)
// - R1 (double)
// - R2 (double)
// - R3 (double)

// Save configuration data to EEPROM
void saveConfig() {
  LOG2("Saving configuration values to EEPROM at address ", CONFIG_ADDRESS);
  // Mark the config as invalid
  EEPROM.write(CONFIG_ADDRESS, CONFIG_INVALID);

  unsigned int address = CONFIG_ADDRESS + 1;  
  
  // Save thermistor calibration data
  {
    EEPROM.updateLong(address, thermistor.getRbalance());
    address += sizeof(long);
    EEPROM.updateDouble(address, thermistor.getT1());
    address += sizeof(double);
    EEPROM.updateDouble(address, thermistor.getT2());
    address += sizeof(double);
    EEPROM.updateDouble(address, thermistor.getT3());
    address += sizeof(double);
    EEPROM.updateDouble(address, thermistor.getR1());
    address += sizeof(double);
    EEPROM.updateDouble(address, thermistor.getR2());
    address += sizeof(double);
    EEPROM.updateDouble(address, thermistor.getR3());
    address += sizeof(double);
  }
  
  // Mark the config as valid
  EEPROM.write(CONFIG_ADDRESS, CONFIG_VALID);
  LOG("Successfully saved config values");
}


// Load configuration data from EEPROM
void loadConfig() {
  int configState = EEPROM.read(CONFIG_ADDRESS);
  if (configState != CONFIG_VALID) {
    LOG2("No valid configuration found at EEPROM address ", CONFIG_ADDRESS);
    LOG("Using default values and saving config to EEPROM");

    // These calibration points are from the thermistor's factory data sheet
    // Manufacturer: Betatherm Part number: 10K3A542I
    //thermistor.calibrate(25.0, 50.0, 80.0, 10000.0, 3600.55, 1255.5);
    // Results from manual calibration
    thermistor.calibrate(21.2, 59.2, 81.0, 12434.0, 2645.0, 1180.0);
    saveConfig();
    return;
  }

  LOG2("Loading configuration values from EEPROM address ", CONFIG_ADDRESS);
  
  unsigned int address = CONFIG_ADDRESS + 1;

  // Calibrate thermistor
  {
    unsigned long Rbalance;
    double T1, T2, T3;
    double R1, R2, R3;
  
    Rbalance = EEPROM.readLong(address);
    address += sizeof(long);
    T1 = EEPROM.readDouble(address);
    address += sizeof(double);
    T2 = EEPROM.readDouble(address);
    address += sizeof(double);
    T3 = EEPROM.readDouble(address);
    address += sizeof(double);
    R1 = EEPROM.readDouble(address);
    address += sizeof(double);
    R2 = EEPROM.readDouble(address);
    address += sizeof(double);
    R3 = EEPROM.readDouble(address);
    address += sizeof(double);
    
    LOG("Loaded config values from EEPROM");
    LOG2("Rbalance: ", Rbalance);
    LOG2("T1: ", T1);
    LOG2("T2: ", T2);
    LOG2("T3: ", T3);
    LOG2("R1: ", R1);
    LOG2("R2: ", R2);
    LOG2("R3: ", R3);
    
    thermistor.setRbalance(Rbalance);
    thermistor.calibrate(T1, T2, T3, R1, R2, R3);
  }
}


void increaseTargetTemp() {
  if (targetTemp < MAX_TARGET_TEMP - TEMP_STEP) {
    targetTemp += TEMP_STEP;
  }
}


void decreaseTargetTemp() {
  if (targetTemp > MIN_TARGET_TEMP + TEMP_STEP) {
    targetTemp -= TEMP_STEP;
  }
}


void increaseTimer() {
  if (timer < MAX_TIME) {
    timer++;
    lastTick = now;
    if (timer < 1) {
      timer = 1;
    }
  }
}


void decreaseTimer() {
  if (timer > 0) {
    timer--;
    lastTick = now;
  }
  if (timer == 0) {
    timer = -1;
  }
}


// Handle button events
void handleEvent(char button, char event) {
  if (state == STATE_STOPPED
      || state == STATE_RUNNING) {
    switch (button) {
    case BUTTON_PLUS:
      if (adjustTarget == ADJUST_TEMP) {
        increaseTargetTemp();
      } else {
        increaseTimer();
      }
      updateNeeded = true;
      break;
    case BUTTON_MINUS:
      if (adjustTarget == ADJUST_TEMP) {
        decreaseTargetTemp();
      } else {
        decreaseTimer();
      }
      updateNeeded = true;
      break;
    case BUTTON_FUNCTION:
      if (adjustTarget == ADJUST_TIMER) {
        adjustTarget = ADJUST_TEMP;
      } else {
        adjustTarget = ADJUST_TIMER;
      }
      updateNeeded = true;
      break;
    }
  }
  
  if (state == STATE_STOPPED) {
    switch (button) {
      case BUTTON_START:
      state = STATE_RUNNING;
      lastTick = now;
      updateNeeded = true;
      break;
    }
  } else if (state == STATE_RUNNING) {
    switch (button) {
      case BUTTON_STOP:
      relayOff();
      state = STATE_STOPPED;
      updateNeeded = true;
      break;
    }
  }
}


// Button event handler functions
void plusButtonClicked() {
  LOG("PLUS button pressed");
  handleEvent(BUTTON_PLUS, EVENT_CLICK);
}

void minusButtonClicked() {
  LOG("MINUS button clicked");
  handleEvent(BUTTON_MINUS, EVENT_CLICK);
}

void startButtonClicked() {
  LOG("START button clicked");
  handleEvent(BUTTON_START, EVENT_CLICK);
}

void stopButtonClicked() {
  LOG("STOP button clicked");
  handleEvent(BUTTON_STOP, EVENT_CLICK);
}

void functionButtonClicked() {
  LOG("FUNCTION button clicked");
  handleEvent(BUTTON_FUNCTION, EVENT_CLICK);
}

void functionButtonLongPressed() {
  LOG("FUNCTION button long pressed");
  handleEvent(BUTTON_FUNCTION, EVENT_LONGPRESS);
}


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


void setup() {
  // Set serial speed
  Serial.begin(SERIAL_SPEED);
  LOG("Initializing"); 

  // Set pin modes
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);

  // Make sure the output is off by default
  relayOff();

  // Set up the display with 16 characters and 2 lines
  lcd.begin(16, 2);
  // Display initialization message on LCD
  // TODO

  // Load calibration and configuration data from EEPROM
  loadConfig();

  // Set up the PID controller
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, WINDOW_SIZE);

  // Attach button handler functions
  plusButton.attachClick(plusButtonClicked);
  minusButton.attachClick(minusButtonClicked);
  startButton.attachClick(startButtonClicked);
  stopButton.attachClick(stopButtonClicked);
  functionButton.attachClick(functionButtonClicked);
  functionButton.attachLongPress(functionButtonLongPressed);

  // TODO: move this state transition to loop()
  // We should take a few temperature samples to make sure
  // it's stable before we transition from STATE_INIT to STATE_STOPPED
  state = STATE_STOPPED;
  
  updateDisplay();
}  


// Main loop
void loop() {
  now = millis();

  // Decrement the timer
  if (timer > 0 && state == STATE_RUNNING) {
    if (now > (lastTick + MINUTE)) {
      lastTick += MINUTE;
      timer--;
      updateNeeded = true;
      if (timer == 0) {
        // Reached the preset time
        state = STATE_STOPPED;
        timer = -1;
      }
    } 
  }

  // Measure the temperature at every sample interval
  if (now > lastSampleTimestamp + SAMPLE_INTERVAL
      || now < lastSampleTimestamp) { // millis() wrapped around
    double reading = thermistor.getTemperatureCelsius();
    lastSampleTimestamp = now;

#ifdef DEBUG
    LOG2("ADC value: ", thermistor.getADC());
    LOG2("Resistance: ", thermistor.getR());
    LOG2("Temperature: ", reading);
#endif

    if (reading > MIN_TEMP && reading < MAX_TEMP) {
      currentTemp = reading;
      pid.Compute();
      LOG2("Current temperature [C]: ", currentTemp);
      LOG2("Relay ON duration [ms]: ", onDuration);
    } else {
      LOG2("Temperature outside of accepted range; rejecting value ", reading);
      // TODO: move to an error state after multiple rejected readings
    }
    updateNeeded = true; 
  }

  // Shift the control window if needed  
  if (now > windowStartTime + WINDOW_SIZE) {
    windowStartTime += WINDOW_SIZE;
  }
  if (state == STATE_RUNNING) {
    // Set the relay state according to the PID output
    if (now < windowStartTime + onDuration) {
      relayOn();
    } else {
      relayOff();
    }
  }
  
  // Update the display if needed
  if (updateNeeded) {
    updateDisplay();
    updateNeeded = false;
  }

  // Poll button states
  plusButton.tick();
  minusButton.tick();
  startButton.tick();
  stopButton.tick();
  functionButton.tick();
}

