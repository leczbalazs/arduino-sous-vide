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
