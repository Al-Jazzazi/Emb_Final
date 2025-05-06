#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>
#include "arduinoFFT.h"
#define N_SAMPLES 128
#define SAMPLE_DURATION 3000  // Collect data over 3 seconds

// Initialize accelerometer (I2C address 0x53 or 0x1D, using 0x7B default)
Adafruit_ADXL343 accel = Adafruit_ADXL343(123);
// FFT setup
const uint16_t samples = N_SAMPLES;
const double samplingFrequency = (double)N_SAMPLES / 3.0; // ~42.67 Hz
// Arrays to hold the real and imaginary parts of the sampled signal
double vReal[samples];
double vImag[samples];
// Create FFT object
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);
//Moving average filter to smooth out the data and reduce noise.Uses a window of 5 samples for averaging.

void moving_avg(double* data) {
    uint8_t filter_size = 5;
    for (uint16_t i = filter_size; i < samples; i++) {
        double sum = 0.0;
        for (uint8_t j = 0; j < filter_size; j++) {
            sum += data[i - j];
        }
        data[i] = sum / filter_size;
    }
}
// Runs FFT on the smoothed data to find the dominant frequency.
double getDominantFrequency(double* data, ArduinoFFT<double>& fft) {
    // Apply Hamming window to reduce spectral leakage
    fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    fft.compute(FFTDirection::Forward);
    fft.complexToMagnitude();
    // Find the frequency bin with the highest magnitude
    double maxMagnitude = 0;
    uint16_t indexOfMax = 0;
    for (uint16_t i = 1; i < samples / 2; i++) { // Ignore DC component at i=0
        if (vReal[i] > maxMagnitude) {
            maxMagnitude = vReal[i];
            indexOfMax = i;
        }
    }
    // Convert index to frequency
    double dominantFreq = (indexOfMax * samplingFrequency) / samples;
    return dominantFreq;
}
//Initializes the accelerometer and sets the range to ±2G for better sensitivity.
void task2_setup() {
    if (!accel.begin()) {
        // Stop the program if the accelerometer is not detected
        while (1);
    }
    accel.setRange(ADXL343_RANGE_2_G); // ±2G range
}
// Collects 3 seconds of acceleration data.Calculates the magnitude of the acceleration vector (x, y, z).
void collectData() {
    int delayTime = SAMPLE_DURATION / N_SAMPLES;
    for (int i = 0; i < N_SAMPLES; i++) {
        sensors_event_t event;
        accel.getEvent(&event);

        // Calculate magnitude of acceleration vector
        double x = event.acceleration.x;
        double y = event.acceleration.y;
        double z = event.acceleration.z;
        vReal[i] = sqrt(x * x + y * y + z * z);
        vImag[i] = 0.0;

        delay(delayTime);
    }
}
void setup() {
    task2_setup();
    pinMode(LED_BUILTIN, OUTPUT);  // Use built-in LED for feedback
}

void loop() {
    collectData();        // Task 2: collect data
    moving_avg(vReal);    // Task 3: clean data
    double freq = getDominantFrequency(vReal, FFT);  // Task 4: analyze data
    // Task 1: indicators based on frequency range
    if (freq >= 3 && freq <= 5) {
        // Tremor detected: light LED and play low tone
        digitalWrite(LED_BUILTIN, HIGH);
        tone(5, 1000);  // 1000 Hz tone on pin 5
    } else if (freq > 5 && freq <= 7) {
        // Dyskinesia detected: light LED and play higher tone
        digitalWrite(LED_BUILTIN, HIGH);
        tone(5, 1500);  // 1500 Hz tone
    } else {
        // No significant tremor/dyskinesia detected
        digitalWrite(LED_BUILTIN, LOW);
        noTone(5);
    }
    delay(1000); // Short pause before next sample set
}
