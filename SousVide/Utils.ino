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
