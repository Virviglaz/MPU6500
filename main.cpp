#include "mpu6500.h"
#include "i2c.h"
#include <exception>
#include <cstdio>
#include <thread>
#include <chrono>

/* I2C interface and device provided by my custom platform-independent implementation */
static I2C_Interface i2c_interface;
static I2C_DeviceBase i2c_device(i2c_interface, 0x68);
static MPU6500_I2C mpu6500_i2c(i2c_device);
static MPU6500_DMP_Base mpu6500(mpu6500_i2c);

int main(int argc, char *argv[])
{
    i2c_interface.Init(argc > 1 ? argv[1] : "/dev/i2c-0");

    int res = 0;

    try {
        mpu6500.Reset(); // Reset the MPU6500 to ensure it's in a known state before initialization
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for 100 ms after reset
        res = mpu6500.Init();
        if (!res) {
            mpu6500.SetSampleRate(125); // Set sample rate to 125 Hz
            mpu6500.SetFilterOrder(3); // Set digital low-pass filter order to 3
            res = mpu6500.Calibrate();
        }
    } catch (const std::exception &e) {
        fprintf(stderr, "Failed to initialize MPU6500: %s\n", e.what());
        return 1;
    }

    /* Read and print raw sensor data */
    for (int i = 0; i < 10; ++i) {
        auto real_data = mpu6500.GetData();
        printf("Accelerometer: ax=%.2f g, ay=%.2f g, az=%.2f g\n", real_data.GetAccX(), real_data.GetAccY(), real_data.GetAccZ());
        printf("Gyroscope: gx=%.2f °/s, gy=%.2f °/s, gz=%.2f °/s\n", real_data.GetGyroX(), real_data.GetGyroY(), real_data.GetGyroZ());
        printf("Temperature: %.2f °C\n\n", real_data.GetTemperature());
    }

    /* Read and print real-world IMU data from DMP */
    for (int i = 0; i < 10; ++i) {
        auto real_data = mpu6500.WaitForRealIMUData();
        printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", real_data.roll, real_data.pitch, real_data.yaw);
    }

    return res;
}
