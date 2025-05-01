#include <Arduino.h>
#include "arduinoFFT.h"
/*
Four tasks 
each implemented as a function 
1. Indicators: (LED lights, sound, etc): takes freq value return nothign
2. Collecting the data into an array :   takes nothing returns array of values (3 seconds)
3. Cleaning the data (moving average) :   takes array of data and returns an array of same size 
4. Using FFT library on the data :       takes array of data returns freq (possibly more than one but for one let's assume one)


each task will come with diff set up, so u can have an additional setup function that takes nothing and returns nothing which we can call in the void setup();


*/
const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
const double signalFrequency = 1000;
const double samplingFrequency = 5000;
const uint8_t amplitude = 100;


double vReal[samples];
double vImag[samples];


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



void setup() {

}

void loop() {
  /* Build raw data */
  double ratio = twoPi * signalFrequency / samplingFrequency; // Fraction of a complete cycle stored at each sample (in radians)
  for (uint16_t i = 0; i < samples; i++)
  {
    vReal[i] = int8_t(amplitude * sin(i * ratio) / 2.0);/* Build data with positive and negative values*/
    vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
  }
  moving_avg(vReal); 

}

