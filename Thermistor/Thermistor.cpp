// Basic NCT thermistor handling class
//
// Expected circuit:
//
// Vcc--Rbalance--+--Thermistor--GND
//                |
//             ADC input

#include "Thermistor.h"

#if ARDUINO >= 100
#include <Arduino.h> 
#else
#include <WProgram.h> 
#endif

#include <math.h>

#define T0 273.15 // 0 °Celsius = 273.15 Kelvin

Thermistor::Thermistor(unsigned char pin) {
    init(pin, 10000, 0.0);
}

Thermistor::Thermistor(
    unsigned char pin,
    unsigned long Rbalance,
    float offset) {
    init(pin, Rbalance, offset);
}

void Thermistor::init(
    unsigned char pin,
    unsigned long Rbalance,
    float offset) {
    _pin = pin;
    _Rbalance = Rbalance;
    _offset = offset;
    // Arduino library does not disconnect the digital pin buffer from the physical
    // pin for analogRead(), so we're doing it here ourselves
    DIDR0 |= _BV(_pin - A0);
}

float Thermistor::getTemperatureCelsius() {
    _ADC = analogRead(_pin);
    float m = _ADC / 1023.0;
    _R = _Rbalance * (1.0 / m - 1.0);
    float logR = log(_R);
    float T = 1.0 / (_A + _B * logR + _C * logR * logR * logR);
    return T - T0 + _offset;
}

void Thermistor::calibrate(
        float T1, float T2, float T3,
        float R1, float R2, float R3) {
    // Store calibration data points
   _T1 = T1;
   _T2 = T2;
   _T3 = T3;
   _R1 = R1;
   _R2 = R2;
   _R3 = R3;

    // Calculate Steinhart-Hart coefficients
    // Based on http://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
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
