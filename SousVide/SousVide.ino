#include <OneButton.h>
#include <PID_v1.h>
#include <Thermistor.h>


Thermistor thermistor(A1, 10000.0);

void setup() {
  Serial.begin(9600);
  thermistor.init();
  thermistor.calibrate(25.0, 50.0, 80.0, 10000.0, 3600.55, 1255.5);
  Serial.println("Calibration results:");
  Serial.print("A = ");
  Serial.println(thermistor.getA(), 6);
  Serial.print("B = ");
  Serial.println(thermistor.getB(), 6);
  Serial.print("C = ");
  Serial.println(thermistor.getC(), 6);

}

void loop() {
}
