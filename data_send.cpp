#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define FREQUENCY_HZ        50
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))
#define ACC_RANGE           1 // 0: -/+2G; 1: +/-4G

// convert factor g to m/s2 ==> [-32768, +32767] ==> [-2g, +2g]
#define CONVERT_G_TO_MS2    (9.81/(16384.0/(1.+ACC_RANGE))) 

static unsigned long last_interval_ms = 0;

MPU6050 imu;
int16_t ax, ay, az;

const float alpha = 0.03; // Smoothing factor (0 < alpha <= 1)
float filtered_ax = 0, filtered_ay = 0, filtered_az = 0;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    imu.initialize();
    delay(200);
    
    //Set MCU 6050 OffSet Calibration 
    imu.setXAccelOffset(-2957);
    imu.setYAccelOffset(-1994);
    imu.setZAccelOffset(2153);
    imu.setXGyroOffset(147);
    imu.setYGyroOffset(-28);
    imu.setZGyroOffset(24);
    
    /* Set full-scale accelerometer range.
     * 0 = +/- 2g
     * 1 = +/- 4g
     * 2 = +/- 8g
     * 3 = +/- 16g
     */

    imu.setFullScaleAccelRange(ACC_RANGE);
}

void lowPassFilter(float ax, float ay, float az) {
    filtered_ax = alpha * ax + (1 - alpha) * filtered_ax;
    filtered_ay = alpha * ay + (1 - alpha) * filtered_ay;
    filtered_az = alpha * az + (1 - alpha) * filtered_az;
}

void loop() {
    if (millis() > last_interval_ms + INTERVAL_MS) {
        last_interval_ms = millis();

        imu.getAcceleration(&ax, &ay, &az);

        float ax_m_s2 = ax * CONVERT_G_TO_MS2;
        float ay_m_s2 = ay * CONVERT_G_TO_MS2;
        float az_m_s2 = az * CONVERT_G_TO_MS2;

        lowPassFilter(ax_m_s2, ay_m_s2, az_m_s2);

        Serial.print(filtered_ax);
        Serial.print("\t");
        Serial.print(filtered_ay);
        Serial.print("\t");
        Serial.println(filtered_az);
    }
}
