#ifndef Thermistor_h
#define Thermistor_h

class Thermistor {
public:
    Thermistor(unsigned char pin);
    Thermistor(
        unsigned char pin,
		unsigned long Rbalance,
		float offset);

    void init(
        unsigned char pin,
		unsigned long Rbalance,
		float offset);

    // Thermistor calibration
    void calibrate(
        float T1, float T2, float T3, // Temperature in Celsius
        float R1, float R2, float R3); // Measured resistance in Ohms

    float getA() { return _A; }
    float getB() { return _B; }
    float getC() { return _C; }

	void setCoeffs(float A, float B, float C) {
	    _A = A;
		_B = B;
		_C = C;
    }

	void setRbalance(unsigned long Rbalance) { _Rbalance = Rbalance; }
	unsigned long getRbalance() { return _Rbalance; }

	void setOffset(float offset) { _offset = offset; }
	float getOffset() { return _offset; }

	unsigned int getADC() { return _ADC; }
    float getTemperatureCelsius();
	float getR() { return _R; }

private:
    // Input pin
    unsigned char _pin;

    // Pull-up resistor value
    unsigned long _Rbalance;

	// Raw ADC value
	unsigned int _ADC;

	// Measured resistance
    float _R;

	// Offset for correction
	float _offset;

    // Steinhart-Hart coefficients
	// Equation: 1/T = A + B * ln(R) + C * (ln(R))^3; T[Kelvin], R[Ohm]
    float _A, _B, _C;
};

#endif
