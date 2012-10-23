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


void displayInitMessage() {
  lcd.clear();
  lcd.print("Initializing...");
}


void displayStatus() {
  lcd.setCursor(0,0);  
  lcd.print("Current: ");
  char temp[8];
  formatTemp(temp, currentTemp);
  lcd.print(temp);
  
  lcd.setCursor(0, 1);
  lcd.print(" Target: ");
  formatTemp(temp, targetTemp);
  lcd.print(temp);
  if (adjustTarget == ADJUST_TEMP) {
    lcd.print(" <-");
  } else {
    lcd.print("   ");
  }
  
  lcd.setCursor(0, 2);
  lcd.print("  Timer:  ");
  if (timer >= 0) {
    sprintf(temp, "%3dm", timer); // Remaining minutes on timer
    lcd.print(temp);
  } else {
    lcd.print("___m");
  }
  if (adjustTarget == ADJUST_TIMER) {
    lcd.print(" <-");
  } else {
    lcd.print("   ");
  }

  lcd.setCursor(0, 3);
  lcd.print("State: ");
  if (running) {
    lcd.print("RUN  ");
  } else {
    lcd.print("STOP ");
  }
  
  lcd.print("Out: ");
  lcd.print(relayState ? "ON " : "OFF");
}


void displayMenu() {
  lcd.home();
  lcd.print((selectedMenuItem == 0) ? ">" : " ");
  lcd.print("Thermistor calibr.");
  lcd.setCursor(0, 1);
  lcd.print((selectedMenuItem == 1) ? ">" : " ");
  lcd.print("PID configuration");
  lcd.setCursor(0, 2);
}


void displayCalibration() {
  lcd.home();
  lcd.print("Calibration (TODO)"); 
}


void displayPidSetting() {
  lcd.home();
  lcd.print("PID setting. (TODO)"); 
}


// Display related functions
void updateDisplay() {
  switch (state) {
  case STATE_INIT:
    displayInitMessage();
    break;
  case STATE_MAIN:
    displayStatus();
    break;
  case STATE_MENU:
    displayMenu();
    break;
  case STATE_CALIBRATION:
    displayCalibration();
    break;
  case STATE_PID_SETTING:
    displayPidSetting();
    break;
  }
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
  if (targetTemp <= MAX_TARGET_TEMP - TEMP_STEP) {
    targetTemp += TEMP_STEP;
  }
}


void decreaseTargetTemp() {
  if (targetTemp >= MIN_TARGET_TEMP + TEMP_STEP) {
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
  switch (state) {
  case STATE_MAIN:
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
      if (event == EVENT_CLICK) {
        if (adjustTarget == ADJUST_TIMER) {
          adjustTarget = ADJUST_TEMP;
        } else {
          adjustTarget = ADJUST_TIMER;
        }
      } else if (event == EVENT_LONGPRESS) {
        state = STATE_MENU;
        selectedMenuItem = 0;
        lcd.clear();
      }
      updateNeeded = true;
      break;
    
    case BUTTON_START:
      if (!running) {
        running = true;
        lastTick = now;
        updateNeeded = true;
      }
      break;
     
    case BUTTON_STOP:
      if (running) {
        relayOff();
        running = false;
        updateNeeded = true;
      }
    }
    break;
    
  case STATE_MENU:
    switch (button) {
    case BUTTON_PLUS:
      selectedMenuItem++;
      selectedMenuItem %= N_MENU_ITEMS;
      updateNeeded = true;
      break;
    
    case BUTTON_MINUS:
      if (selectedMenuItem == 0) {
        selectedMenuItem = N_MENU_ITEMS - 1;
      } else {
        selectedMenuItem--;
      }
      updateNeeded = true;      
      break;
    
    case BUTTON_FUNCTION:
      // NO-OP
      break;
    
    case BUTTON_START:
      switch (selectedMenuItem) {
      case 0:
        state = STATE_CALIBRATION;
        updateNeeded = true;
        lcd.clear();
        break;
      case 1:
        state = STATE_PID_SETTING;
        updateNeeded = true;
        lcd.clear();      
        break;        
      }
      break;
     
    case BUTTON_STOP:
      state = STATE_MAIN;
      updateNeeded = true;
      lcd.clear();  
      break;
    }
    break;    
  
  case STATE_CALIBRATION:
    switch (button) {
    case BUTTON_PLUS:
      // TODO
      break;
    
    case BUTTON_MINUS:
      // TODO
      break;
    
    case BUTTON_FUNCTION:
      // TODO
      break;
    
    case BUTTON_START:
      // TODO
      break;
     
    case BUTTON_STOP:
      state = STATE_MENU;
      updateNeeded = true;
      lcd.clear();  
      break;
    }
    break;

  case STATE_PID_SETTING:
    switch (button) {
    case BUTTON_PLUS:
      // TODO
      break;
    
    case BUTTON_MINUS:
      // TODO
      break;
    
    case BUTTON_FUNCTION:
      // TODO
      break;
    
    case BUTTON_START:
      // TODO
      break;
     
    case BUTTON_STOP:
      state = STATE_MENU;
      updateNeeded = true;
      lcd.clear();  
      break;
    }
    break;
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


void sampleTemperature() {
  double reading = thermistor.getTemperatureCelsius();

#ifdef DEBUG
  LOG2("ADC value: ", thermistor.getADC());
  LOG2("Resistance: ", thermistor.getR());
  LOG2("Temperature: ", reading);
#endif

  if (reading > MIN_TEMP && reading < MAX_TEMP) {
    samples[currentSample] = reading;
    currentSample = (currentSample + 1) % N_SAMPLES;
    lastSampleTimestamp = now;
  } else {
    LOG2("Temperature outside of accepted range; rejecting value ", reading);
    // TODO: move to an error state after multiple rejected readings
  }
}


void updateTemperature() {
  double newTemp = 0.0;
  int n = 0;
  for (int i = 0; i < N_SAMPLES; i++) {
    if (samples[i] > 0.0) {
      newTemp += samples[i];
      n++;
    }
  }
  
  if (n > 0) {
    currentTemp = newTemp / n;
    lastUpdateTimestamp = now;
  }
}


void setup() {
  // Set serial speed
  Serial.begin(SERIAL_SPEED);
  LOG("Initializing"); 

  // Set pin modes
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);

  // Make sure the relay is off by default
  relayOff();

  // Set up the display with 16 characters and 2 lines
  lcd.begin(20, 4);
  // Display initialization message on LCD
  updateDisplay();

  // Load calibration and configuration data from EEPROM
  loadConfig();

  // Set up the PID controller
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, WINDOW_SIZE);
  pid.SetSampleTime(UPDATE_INTERVAL);

  // Attach button handler functions
  plusButton.attachClick(plusButtonClicked);
  minusButton.attachClick(minusButtonClicked);
  startButton.attachClick(startButtonClicked);
  stopButton.attachClick(stopButtonClicked);
  functionButton.attachClick(functionButtonClicked);
  functionButton.attachLongPress(functionButtonLongPressed);

  // Take one temperature sample
  sampleTemperature();
  updateTemperature();
  
  state = STATE_MAIN;
  lcd.clear();
  updateDisplay();
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

