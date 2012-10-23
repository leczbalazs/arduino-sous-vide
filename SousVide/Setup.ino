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
