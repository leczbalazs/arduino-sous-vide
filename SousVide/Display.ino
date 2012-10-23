void displayInit() {
  lcd.clear();
  lcd.print("Initializing...");
}


void displayMain() {
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
    displayInit();
    break;
  case STATE_MAIN:
    displayMain();
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
