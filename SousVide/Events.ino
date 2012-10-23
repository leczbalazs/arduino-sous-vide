
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

