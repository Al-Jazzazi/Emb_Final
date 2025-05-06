#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>
#include "arduinoFFT.h"

#define N_SAMPLES 128
#define SAMPLE_DURATION 3000

Adafruit_ADXL343 accel = Adafruit_ADXL343(123);
double rawData[N_SAMPLES]; 
double vReal[N_SAMPLES];
double vImag[N_SAMPLES];

const double samplingFrequency = (double)N_SAMPLES / 3.0; 
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, N_SAMPLES, samplingFrequency);

// Smooth the data using a moving average
void moving_avg(double *data) {
    uint8_t filter_size = 5;
    for (uint16_t i = filter_size; i < N_SAMPLES; i++) {
        double sum = 0;
        for (uint8_t j = 0; j < filter_size; j++) {
            sum += data[i - j];
        }
        data[i] = sum / filter_size;
    }
}

// Run FFT and return the dominant frequency
double getDominantFrequency(double *data, ArduinoFFT<double> &fft) {
    fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    fft.compute(FFTDirection::Forward);
    fft.complexToMagnitude();

    double maxMag = 0;
    uint16_t index = 0;
    for (uint16_t i = 1; i < N_SAMPLES / 2; i++) {
        if (vReal[i] > maxMag) {
            maxMag = vReal[i];
            index = i;
        }
    }
    return (index * samplingFrequency) / N_SAMPLES;
}

// Light LED and play sound based on frequency
void indicator(double freq) {
    if (freq >= 3 && freq <= 5) {
        digitalWrite(LED_BUILTIN, HIGH);
        tone(5, 1000);
    } else if (freq > 5 && freq <= 7) {
        digitalWrite(LED_BUILTIN, HIGH);
        tone(5, 1500);
    } else {
        digitalWrite(LED_BUILTIN, LOW);
        noTone(5);
    }
}

// Initialize accelerometer
void task2_setup() {
    if (!accel.begin()) {
        while (1);
    }
    accel.setRange(ADXL343_RANGE_2_G);
    pinMode(LED_BUILTIN, OUTPUT);
}

// Collect 3 seconds of acceleration data (x-axis only)
double *collectData() {
    int delayTime = SAMPLE_DURATION / N_SAMPLES;
    for (int i = 0; i < N_SAMPLES; i++) {
        sensors_event_t event;
        accel.getEvent(&event);
        rawData[i] = event.acceleration.x;
        delay(delayTime);
    }
    return rawData;
}

void setup() {
    Serial.begin(115200);
    task2_setup();
}

void loop() {
    double *data = collectData();

    // Copy data for FFT
    for (int i = 0; i < N_SAMPLES; i++) {
        vReal[i] = data[i];
        vImag[i] = 0;
    }

    moving_avg(vReal);
    double freq = getDominantFrequency(vReal, FFT);

    Serial.print("Dominant Frequency: ");
    Serial.print(freq);
