
#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include "arduinoFFT.h"

#define SAMPLES 128                 // Number of samples for each axis
#define SAMPLING_INTERVAL 3000      // Sampling interval in milliseconds (3 seconds)
#define SAMPLING_FREQUENCY 50      // Sampling frequency (50 Hz)
#define AMPLITUDE 100              // Signal amplitude

#define  ParkinsoTone 1000 
#define dyskinesiaTone 1500 


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


/*
Takes freq and based on that sets indicators 
*/
void indicator(double freq) {
  CircuitPlayground.clearPixels();
  if (freq >= 3 && freq <= 5) {
      digitalWrite(LED_BUILTIN, HIGH);
      if(CircuitPlayground.slideSwitch())
        tone(5, ParkinsoTone);
      else 
        noTone(5);

      for (uint8_t i = 0; i < 10; i++) {

        CircuitPlayground.setPixelColor(i, 255, 0, 0); // R, G, B

      }
  } else if (freq > 5 && freq <= 7) {
      digitalWrite(LED_BUILTIN, HIGH);
      if(CircuitPlayground.slideSwitch())
        tone(5, dyskinesiaTone);
      else 
        noTone(5);

      for (uint8_t i = 0; i < 10; i++) {
        CircuitPlayground.setPixelColor(i, 0, 0, 255); // R, G, B
      }
 
    }
    else{
      digitalWrite(LED_BUILTIN, LOW);
      noTone(5);
    }
  }



/*
Computed the RMS which is used as the indicator for intesity  
*/
  double computeRMS(const double data[], uint16_t len) {
    double sumSq = 0;
    for (uint16_t i = 0; i < len; i++) {
      sumSq += data[i] * data[i];
    }
    return sqrt(sumSq / len);
  }

/*
Moving Avg Function used in filtering sampled data to remove the noise 
*/
void moving_avg(double* vReal) {
  uint8_t filter_size = 3;  
  double movAvg = 0.0;     

  for (uint16_t i = filter_size; i < SAMPLES; i++) {
      double sum = 0.0;
    
      for (uint8_t j = 0; j < filter_size; j++) {
          sum += vReal[i - j];  
      }
      movAvg = sum / filter_size;
      vReal[i] = movAvg;  
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
    //Seri.println("Data collection complete for 3 seconds.");
    // Get the dominant frequency from FFT
    moving_avg(vReal);
    double freq = getDominantFrequency(vReal);
    double rms = computeRMS(vReal, SAMPLES);
    
    double rms_g = rms / 9.8;                 //removing gravity 
    double norm  = rms_g / 5.0;                 // normalize
    norm = constrain(norm, 0.0, 1.0);

    uint8_t brightness = norm * 30;            // 0-30 
    CircuitPlayground.setBrightness(brightness);

    Serial.print("rms is "); 
    Serial.print(brightness);
    Serial.print(", Dominant Frequency: ");
    Serial.print(freq);
    Serial.println(" Hz");
    
    indicator(freq);
    // Reset for next collection cycle
    sampleIndex = 0;  // Reset sample index for the next cycle
    startSampleTime = currentMillis;  // Reset start time for the next collection
  }
}

/*
Function used for user to test the sounds 
*/
inline void testing_sound(){
  if(!CircuitPlayground.slideSwitch() && CircuitPlayground.leftButton())
  tone(5, ParkinsoTone); 

if(!CircuitPlayground.slideSwitch() && CircuitPlayground.rightButton())
  tone(5,  dyskinesiaTone);
}

void setup() {
  // Initialize Circuit Playground
  CircuitPlayground.begin();
  
  // Start //Seri communication
  Serial.begin(115200);
  Serial.println("Accelerometer Data Collection Started...");
  
  // Set the start time for the first collection cycle
  startSampleTime = millis();
}


void loop() {
  collectAccelerometerData();  // Collect data for 3 seconds and process FFT
  testing_sound();
}