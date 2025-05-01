#include <Arduino.h>
#include "arduinoFFT.h"

/*
Embedded Challenge Spring 2025 - Frequency Analysis

Tasks:
1. Indicator
2. Data collection
3. Moving average filter
4. FFT to find dominant frequency
*/

const uint16_t samples = 64; // Number of samples (must be power of 2)
const double signalFrequency = 1000; // Simulated signal frequency in Hz
const double samplingFrequency = 5000; // Sampling rate in Hz
const uint8_t amplitude = 100; // Signal amplitude

double vReal[samples];
double vImag[samples];

// Initialize FFT object
arduinoFFT FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency);

// Apply moving average filter to smooth the data
void moving_avg(double* vReal) {
    uint8_t filter_size = 5;
    for (uint16_t i = filter_size; i < samples; i++) {
        double sum = 0.0;
        for (uint8_t j = 0; j < filter_size; j++) {
            sum += vReal[i - j];
        }
        vReal[i] = sum / filter_size;
    }
}

// Perform FFT and return the dominant frequency
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
    return (indexOfMax * samplingFrequency) / samples;
}

void setup() {
    Serial.begin(115200); // Start serial communication
}

void loop() {
    // Generate simulated sine wave data
    double ratio = twoPi * signalFrequency / samplingFrequency;
    for (uint16_t i = 0; i < samples; i++) {
        vReal[i] = int8_t(amplitude * sin(i * ratio) / 2.0);
        vImag[i] = 0.0;
    }

    moving_avg(vReal); // Smooth the data

    double freq = getDominantFrequency(vReal); // Find the dominant frequency

    Serial.print("Dominant Frequency: ");
    Serial.print(freq);
    Serial.println(" Hz");

    delay(1000); // Wait 1 second
}
