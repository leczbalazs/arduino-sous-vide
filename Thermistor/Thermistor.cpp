#include <math.h>
#include "Thermistor.h"

#define T0 273.15

/*
Calculated:
A = 0.0011304084062576293
B = 0.0002339279413223266
C = 0.0000000883788013458

Fitted:
A = 0.0011304106122615558
B = 0.0002339275532379136
C = 0.0000000883810102069
*/

Thermistor::Thermistor(int pin, float R1) {
    _pin = pin;
    _R1 = R1;
}

void Thermistor::init() {
}

void Thermistor::calibrate(
	    float T1, float T2, float T3,
	    float R1, float R2, float R3) {
      float L1 = log(R1);
      float L2 = log(R2);
      float L3 = log(R3);

      float Y1 = 1.0 / (T1 + T0);
      float Y2 = 1.0 / (T2 + T0);
      float Y3 = 1.0 / (T3 + T0);
      float g2 = (Y2 - Y1) / (L2 - L1);
      float g3 = (Y3 - Y1) / (L3 - L1);

      _C = ((g3 - g2) / (L3 - L2)) * 1.0 / (L1 + L2 + L3);
      _B = g2 - _C * (L1 * L1 + L1 * L2 + L2 * L2);
      _A = Y1 - (_B + L1 * L1 * _C) * L1; 
}
