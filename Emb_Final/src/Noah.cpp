#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>

#define N_SAMPLES 128
#define SAMPLE_DURATION 3000

Adafruit_ADXL343 accel = Adafruit_ADXL343(123);
double rawData[N_SAMPLES]; // Shared array to hold 3s of data

// Task 2 setup function (initializes IMU)
void task2_setup()
{
    if (!accel.begin())
    {
        while (1)
            ; // Stop if accelerometer not found
    }
    accel.setRange(ADXL343_RANGE_2_G); // Set ±2G range
}

// Task 2: Collect 3 seconds of X-axis acceleration data
double *collectData()
{
    int delayTime = SAMPLE_DURATION / N_SAMPLES;
    for (int i = 0; i < N_SAMPLES; i++)
    {
        sensors_event_t event;
        accel.getEvent(&event);
        rawData[i] = event.acceleration.x;
        delay(delayTime);
    }
    return rawData;
}

// Debug only: Print the collected data
void printData(double *data)
{
    for (int i = 0; i < N_SAMPLES; i++)
    {
        Serial.print("Sample ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(data[i]);
    }
}

void setup()
{
    Serial.begin(115200); // For testing only, remove in final version
    task2_setup();
}

void loop()
{
    double *data = collectData(); // Collect 3 seconds of data
    printData(data);              // Debug print
    delay(3000);                  // Wait before collecting again
}
