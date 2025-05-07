
#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include "arduinoFFT.h"

#define SAMPLES 128                 // Number of samples for each axis
#define SAMPLING_INTERVAL 3000      // Sampling interval in milliseconds (3 seconds)
#define SAMPLING_FREQUENCY 50      // Sampling frequency (50 Hz)
#define AMPLITUDE 100              // Signal amplitude


// Data arrays for storing accelerometer readings
float xData[SAMPLES];
float yData[SAMPLES];
float zData[SAMPLES];
double vImag[SAMPLES];  // Imaginary part for FFT
double vReal[SAMPLES];  // Real part for FFT

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// Variables to track the sampling
unsigned long lastSampleTime = 0;
unsigned long startSampleTime = 0;
int sampleIndex = 0;


void indicator(double freq) {
  CircuitPlayground.clearPixels();
  if (freq >= 3 && freq <= 5) {
      digitalWrite(LED_BUILTIN, HIGH);
      tone(5, 1000);
      for (uint8_t i = 0; i < 10; i++) {

        CircuitPlayground.setPixelColor(i, 255, 0, 0); // R, G, B

      }
  } else if (freq > 5 && freq <= 7) {
      digitalWrite(LED_BUILTIN, HIGH);
      tone(5, 1500);
      for (uint8_t i = 0; i < 10; i++) {
        CircuitPlayground.setPixelColor(i, 0, 0, 255); // R, G, B
      }
 
    }
  
  }
double getDominantFrequency(double* vReal) {
    // Apply window function (Hamming window)
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    // Perform FFT
    FFT.compute(FFT_FORWARD);
    // Convert complex values to magnitudes
    FFT.complexToMagnitude();

    double maxMagnitude = 0;
    uint16_t indexOfMax = 0;

    // Find the index of the highest magnitude frequency
    for (uint16_t i = 1; i < SAMPLES / 2; i++) { 
        if (vReal[i] > maxMagnitude) {
            maxMagnitude = vReal[i];
            indexOfMax = i;
        }
    }

    // Calculate the frequency corresponding to the peak magnitude
    double dominantFreq = (indexOfMax * SAMPLING_FREQUENCY) / SAMPLES;
    return dominantFreq;
}

inline void displayData(float x[], float y[], float z[]) {
  for (int i = 0; i < sampleIndex; i++) {
    Serial.print("Sample ");
    Serial.print(i);
    Serial.print(": X = ");
    Serial.print(x[i]);
    Serial.print(", Y = ");
    Serial.print(y[i]);
    Serial.print(", Z = ");
    Serial.println(z[i]);
  }
}

void collectAccelerometerData() {
  unsigned long currentMillis = millis();

  // Collect data for 3 seconds (SAMPLING_INTERVAL)
  if (currentMillis - startSampleTime < SAMPLING_INTERVAL && sampleIndex < SAMPLES) {
    if (currentMillis - lastSampleTime >= 20) {  // Collect data every 20ms (50 Hz sample rate)
      lastSampleTime = currentMillis;

      // Collect X, Y, and Z axis data from the accelerometer
      float x = CircuitPlayground.motionX();
      float y = CircuitPlayground.motionY();
      float z = CircuitPlayground.motionZ();
    
      // Store the data in the arrays
      xData[sampleIndex] = x;
      yData[sampleIndex] = y;
      zData[sampleIndex] = z;

      // Calculate magnitude (sqrt(x^2 + y^2 + z^2))
      double magnitude = sqrt((x * x) + (y * y) + (z * z)) - 9.8;  // Remove gravity component
      vReal[sampleIndex] = magnitude;
      vImag[sampleIndex] = 0.0;  // Imaginary part must be zeroed

      // Move to the next sample index
      sampleIndex++;
    }
  }

  // Once 3 seconds are over or we've collected all samples, display data and reset
  if (currentMillis - startSampleTime >= SAMPLING_INTERVAL || sampleIndex >= SAMPLES) {
    Serial.println("Data collection complete for 3 seconds.");
    // Get the dominant frequency from FFT
    double freq = getDominantFrequency(vReal);
    Serial.print("Dominant Frequency: ");
    Serial.print(freq);
    Serial.println(" Hz");
    indicator(freq);
    // Reset for next collection cycle
    sampleIndex = 0;  // Reset sample index for the next cycle
    startSampleTime = currentMillis;  // Reset start time for the next collection
  }
}

void setup() {
  // Initialize Circuit Playground
  CircuitPlayground.begin();
  
  // Start serial communication
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Accelerometer Data Collection Started...");
  
  // Set the start time for the first collection cycle
  startSampleTime = millis();
}

void loop() {
  collectAccelerometerData();  // Collect data for 3 seconds and process FFT
}