#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// Define sampling frequency and interval
#define FREQUENCY_HZ        50
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))

// Define accelerometer range (0: +/-2G, 1: +/-4G, etc.)
#define ACC_RANGE           1

// Conversion factor from g to m/s^2 based on accelerometer range
#define CONVERT_G_TO_MS2    (9.81 / (16384.0 / (1.0 + ACC_RANGE)))

// Variables for timing and smoothing
static unsigned long last_interval_ms = 0;

// MPU6050 instance
MPU6050 imu;

// Raw accelerometer data
int16_t ax, ay, az;

// Low-pass filter constants
const float alpha = 0.03; // Smoothing factor (0 < alpha <= 1)
float filtered_ax = 0, filtered_ay = 0, filtered_az = 0;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    imu.initialize(); // Initialize the MPU6050
    delay(200);

    /* Set full-scale accelerometer range.
     * 0 = +/- 2g
     * 1 = +/- 4g
     * 2 = +/- 8g
     * 3 = +/- 16g
     */
    imu.setFullScaleAccelRange(ACC_RANGE);
}

// Apply a low-pass filter to smooth accelerometer data
void lowPassFilter(float ax, float ay, float az) {
    filtered_ax = alpha * ax + (1 - alpha) * filtered_ax;
    filtered_ay = alpha * ay + (1 - alpha) * filtered_ay;
    filtered_az = alpha * az + (1 - alpha) * filtered_az;
}

void loop() {
    // Check if it's time for the next sample
    if (millis() > last_interval_ms + INTERVAL_MS) {
        last_interval_ms = millis(); // Update timing

        // Read raw accelerometer data
        imu.getAcceleration(&ax, &ay, &az);

        // Convert raw accelerometer data to m/s^2
        float ax_m_s2 = ax * CONVERT_G_TO_MS2;
        float ay_m_s2 = ay * CONVERT_G_TO_MS2;
        float az_m_s2 = az * CONVERT_G_TO_MS2;

        // Apply low-pass filter
        lowPassFilter(ax_m_s2, ay_m_s2, az_m_s2);

        // Print filtered data to Serial Monitor
        Serial.print(filtered_ax);
        Serial.print("\t");
        Serial.print(filtered_ay);
        Serial.print("\t");
        Serial.println(filtered_az);
    }
}
