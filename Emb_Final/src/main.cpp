#include <Arduino.h>
#include "indicators.h"

void setup() {
    setupIndicators();
}
void loop() {
    float simulated_freq = 4.0; // Replace with FFT result later
    showIndicators(simulated_freq);
    delay(1000); // Adjust based on how often you call FFT
}

