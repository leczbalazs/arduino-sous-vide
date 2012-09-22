#ifndef Thermistor_h
#define Thermistor_h

class Thermistor {
public:
    Thermistor(int pin, float R1);
    void init();

    // Thermistor calibration
    void calibrate(
        float T1, float T2, float T3, // Temperature in Celsius
        float R1, float R2, float R3); // Measured resistance in Ohms

    float getA() { return _A; };
    float getB() { return _B; };
    float getC() { return _C; };

private:
    // Input pin
    int _pin;
    
    // Pull-up resistor value
    float _R1;

    // Steinhart-Hart coefficients for NTC thermistors
    float _A, _B, _C;
};

#endif
