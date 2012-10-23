
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
