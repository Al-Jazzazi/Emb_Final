#include <Arduino.h>
#include "arduinoFFT.h"

const uint16_t samples = 64; \
const double signalFrequency = 1000;
const double samplingFrequency = 5000;
const uint8_t amplitude = 100;

double vReal[samples];
double vImag[samples];

// Initialize FFT object
arduinoFFT FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency);

// Moving average filter
void moving_avg(double* vReal) {
    uint8_t filter_size = 5;  
    double movAvg = 0.0;     

    for (uint16_t i = filter_size; i < samples; i++) {
        double sum = 0.0;
        for (uint8_t j = 0; j < filter_size; j++) {
            sum += vReal[i - j];  
        }
        movAvg = sum / filter_size;
        vReal[i] = movAvg;  
    }
}

double getDominantFrequency(double* vReal) {
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(FFT_FORWARD);
    FFT.ComplexToMagnitude();

    double maxMagnitude = 0;
    uint16_t indexOfMax = 0;
    for (uint16_t i = 1; i < samples / 2; i++) { 
        if (vReal[i] > maxMagnitude) {
            maxMagnitude = vReal[i];
            indexOfMax = i;
        }
    }
    double dominantFreq = (indexOfMax * samplingFrequency) / samples;
    return dominantFreq;
}

void setup() {
    Serial.begin(115200);  // Start Serial Monitor to print frequency
}

void loop() {
    /* Build raw data */
    double ratio = twoPi * signalFrequency / samplingFrequency; 
    for (uint16_t i = 0; i < samples; i++) {
        vReal[i] = int8_t(amplitude * sin(i * ratio) / 2.0); // Simulate sine wave data
        vImag[i] = 0.0; // Imaginary part must be zeroed
    }

    moving_avg(vReal);  

    double freq = getDominantFrequency(vReal);  

    Serial.print("Dominant Frequency: ");
    Serial.print(freq);
    Serial.println(" Hz");

    delay(1000); // Wait before next reading
}
