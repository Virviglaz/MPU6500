/*
 * This file is provided under a MIT license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * MIT License
 *
 * Copyright (c) 2026 Pavel Nadein
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * MPU-6050 C++ driver implementation.
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

#ifndef MPU6500_H
#define MPU6500_H

#ifndef __cplusplus
#error "This header requires C++11 or higher"
#endif

#include "devices.h"
#include <cstdint>

/**
 * Abstract base class for MPU-6500 interfaces (I2C, SPI, etc.)
 */
class MPU6500_IFS_Base
{
public:
    explicit MPU6500_IFS_Base() = default;
    virtual ~MPU6500_IFS_Base() = default;

    /* Bit manipulation */
    void SetBit(uint8_t reg, uint8_t bit_mask);
    void ClrBit(uint8_t reg, uint8_t bit_mask);

    /* Register access */
    void WriteReg(uint8_t reg, uint8_t value);
    void WriteReg_s16(uint8_t reg, int16_t value);
    int16_t ReadReg_s16(uint8_t reg);
    uint8_t ReadReg(uint8_t reg);
    void Read(uint8_t reg, uint8_t* data, size_t length);
    void Write(uint8_t reg, const uint8_t* data, size_t length);
    void ReadFIFO(uint8_t *buf, size_t size);

    /* Pure virtual methods to be implemented by derived classes for specific interfaces */
    virtual void Write(const uint8_t *reg_addr,
                       size_t reg_addr_size,
                       const uint8_t *data,
                       size_t length) = 0;

    virtual void Read(const uint8_t *reg_addr,
                      size_t reg_addr_size,
                      uint8_t *data,
                      size_t length) = 0;
};

/**
 * MPU-6500 I2C interface implementation.
 */
class MPU6500_I2C : public MPU6500_IFS_Base
{
public:
    explicit MPU6500_I2C(I2C_DeviceBase& device) : device_(device) {}
    ~MPU6500_I2C() override = default;

    void Write(const uint8_t *reg_addr,
               size_t reg_addr_size,
               const uint8_t *data,
               size_t length) override;

    void Read(const uint8_t *reg_addr,
              size_t reg_addr_size,
              uint8_t *data,
              size_t length) override;
private:
    I2C_DeviceBase& device_;
};

/**
 * MPU-6500 SPI interface implementation.
 */
class MPU6500_SPI : public MPU6500_IFS_Base
{
public:
    explicit MPU6500_SPI(SPI_DeviceBase& device) : device_(device) {}
    ~MPU6500_SPI() override = default;

    void Write(const uint8_t *reg_addr,
               size_t reg_addr_size,
               const uint8_t *data,
               size_t length) override;

    void Read(const uint8_t *reg_addr,
              size_t reg_addr_size,
              uint8_t *data,
              size_t length) override;
private:
    SPI_DeviceBase& device_;
    uint8_t buffer_[512]; // Temporary buffer for SPI transactions (adjust size as needed)
};

/**
 * MPU6500_Base class provides basic functionality to interface with the MPU-6500 sensor,
 * including reading raw accelerometer, gyroscope, and temperature data, as well
 * as configuring the sensor's gain settings, sample rate, and filter order. It also includes
 * methods to convert raw sensor data to real-world units and to handle data ready interrupts.
 */
class MPU6500_Base {
public:
    /**
     * Accelerometer gain settings (full scale range):
     * SCALE_2G  = ±2g   (16384 LSB/g)
     * SCALE_4G  = ±4g   (8192 LSB/g)
     * SCALE_8G  = ±8g   (4096 LSB/g)
     * SCALE_16G = ±16g  (2048 LSB/g)
     */
    enum class Acc_Gain
    {
        SCALE_2G,
        SCALE_4G,
        SCALE_8G,
        SCALE_16G,
    };

    /**
     * Gyroscope gain settings (full scale range):
     * GYRO_0250DS = ±250°/s   (131 LSB/°/s)
     * GYRO_0500DS = ±500°/s   (65.5 LSB/°/s)
     * GYRO_1000DS = ±1000°/s  (32.8 LSB/°/s)
     * GYRO_2000DS = ±2000°/s  (16.4 LSB/°/s)
     */
    enum class Gyro_Gain
    {
        GYRO_0250DS,
        GYRO_0500DS,
        GYRO_1000DS,
        GYRO_2000DS,
    };

    MPU6500_Base() = delete;
    ~MPU6500_Base() = default;

    /**
     * Constructor that takes an I2C device interface. The caller is responsible for ensuring
     * that the I2C device is properly initialized and configured before calling Init().
     */
    MPU6500_Base(MPU6500_IFS_Base& ifs)
        : ifs_(ifs) {}

    /**
     * Initializes the MPU6500 device. This must be called before any other operations.
     * Returns 0 on success, or a negative error code on failure.
     * Returns -ENODEV if the device is not found or does not respond correctly.
     */
    int Init();

    /**
     * Resets the MPU6500 device. This will reset all registers to their default values and clear the FIFO buffer.
     * After calling this method, it is recommended to wait at least 100 ms before calling Init() again to reconfigure the device.
     */
    void Reset();

    /**
     * Sets the accelerometer gain (full scale range). This affects the sensitivity of the accelerometer readings.
     * The default gain is SCALE_2G (±2g).
     *
     * @param gain The desired accelerometer gain setting.
     */
    void SetAccGain(Acc_Gain gain);


    /**
     * Sets the gyroscope gain (full scale range). This affects the sensitivity of the gyroscope readings.
     * The default gain is GYRO_0250DS (±250°/s).
     *
     * @param gain The desired gyroscope gain setting.
     */
    void SetGyroGain(Gyro_Gain gain);

    /**
     * Sets the sample rate of the MPU6500. The sample rate determines how often the sensor data is updated.
     * The default sample rate is 125 Hz (when using the DMP, the sample rate is determined by the DMP firmware).
     *
     * @param sample_rate_hz The desired sample rate in Hertz (Hz). Valid values are typically between 4 Hz and 1000 Hz.
     */
    void SetSampleRate(uint8_t sample_rate_hz);

    /**
     * Sets the digital low-pass filter (DLPF) order. This affects the bandwidth of the sensor data and can help reduce noise.
     * The default filter order is 0 (no filtering).
     *
     * @param filter_order The desired filter order (0-7). Higher values correspond to stronger filtering and lower bandwidth.
     */
    void SetFilterOrder(uint8_t filter_order);

    /**
     * Enables the data ready interrupt. When enabled, the MPU6500 will generate an interrupt signal when new sensor data is available.
     * The caller must ensure that the appropriate interrupt handler is set up to handle the data ready interrupt.
     * Note: Enabling the data ready interrupt may require additional configuration of the MPU6500's interrupt settings and the host system's interrupt handling.
     * Override IsDataReady() if you want to use a different method of checking for new data instead of polling.
     */
    void EnableDataReadyInterrupt();

    /**
     * RawData struct represents the raw sensor data read from the MPU6500. It includes raw accelerometer, gyroscope, and temperature readings.
     * The GetTemperature(), GetAccX(), GetAccY(), GetAccZ(), GetGyroX(), GetGyroY(), and GetGyroZ() methods convert the raw readings to
     * real-world units (°C for temperature, g for acceleration, and °/s for angular velocity) based on the current gain settings.
     */
    class RawData {
        friend class MPU6500_Base;
    public:
        RawData(Acc_Gain acc_gain, Gyro_Gain gyro_gain)
            : _acc_gain(acc_gain), _gyro_gain(gyro_gain) {}
        float GetTemperature() const;
        float GetAccX() const;
        float GetAccY() const;
        float GetAccZ() const;
        float GetGyroX() const;
        float GetGyroY() const;
        float GetGyroZ() const;
    private:
        int16_t x; /* MPU6500_RA_ACCEL_XOUT H/L */
        int16_t y; /* MPU6500_RA_ACCEL_YOUT H/L */
        int16_t z; /* MPU6500_RA_ACCEL_ZOUT H/L */

        int16_t temp; /* MPU6500_RA_TEMP_OUT H/L */

        int16_t ax; /* MPU6500_RA_GYRO_XOUT H/L */
        int16_t ay; /* MPU6500_RA_GYRO_YOUT H/L */
        int16_t az; /* MPU6500_RA_GYRO_ZOUT H/L */
        Acc_Gain _acc_gain;
        Gyro_Gain _gyro_gain;
    };

    /**
     * Waits for new sensor data to be available and returns the raw data. This method will block until new data is ready.
     * It is recommended to use EnableDataReadyInterrupt() and override IsDataReady() for a more efficient implementation
     * that does not rely on busy-waiting.
     */
    RawData WaitForData();

    /**
     * Returns the latest raw sensor data without waiting. The caller should ensure that new data is available before calling
     * this method, either by using EnableDataReadyInterrupt() and checking IsDataReady(), or by implementing their own timing mechanism.
     * This method reads the raw accelerometer, gyroscope, and temperature data from the MPU6500 and returns it as a RawData
     * struct. The raw values are in big-endian format and are converted to native endianness before being returned.
     */
    RawData GetData();

    /**
     * Performs a calibration of the MPU6500 sensor. The calibration process adjusts the hardware offset registers
     * iteratively to minimize the error between the sensor readings and the expected values (0 g for accelerometer axes,
     * 0 °/s for gyro axes). The calibration will run for a maximum of max_iterations or until the error is within
     * target_error LSB of the target values.
     * Returns 0 on success, or a negative error code on failure.
     *
     * @param max_iterations The maximum number of calibration iterations to perform.
     * A higher number may yield better results but will take longer.
     * @param target_error The target error threshold in LSB.
     * Calibration will stop when all sensor readings are within this error
     * threshold of the target values (0 for accel X/Y, 16384 for accel Z, 0 for gyro X/Y/Z).
     *
     * @return 0 if calibration succeeded, or a negative error code if calibration failed after max_iterations.
     */
    int Calibrate(int max_iterations = 100, int16_t target_error = 50);

    /**
     * Struct to hold the calibration offsets for the accelerometer and gyroscope.
     * This struct is used to read and write the hardware offset registers of the
     * MPU6500 during the calibration process.
     * The offsets are stored as 16-bit signed integers, and the struct is packed
     * to ensure it matches the layout of the MPU6500's offset registers.
     */
    #pragma pack(push, 1)
    struct cal_offsets {
        cal_offsets() :
            acc_x_offset(0), acc_y_offset(0), acc_z_offset(0),
            gyro_x_offset(0), gyro_y_offset(0), gyro_z_offset(0) {}
        int16_t acc_x_offset;
        int16_t acc_y_offset;
        int16_t acc_z_offset;
        int16_t gyro_x_offset;
        int16_t gyro_y_offset;
        int16_t gyro_z_offset;
    };
    #pragma pack(pop)
    static_assert(sizeof(cal_offsets) == 12,
        "cal_offsets struct must be exactly 12 bytes to match the MPU6500 offset register layout");

    /**
     * Reads the current calibration offsets from the MPU6500's hardware
     * offset registers and returns them as a cal_offsets struct.
     * The caller can use this method to retrieve the current offsets
     * before starting a calibration process, or to read the offsets after
     * calibration to save them for future use.
     */
    cal_offsets ReadCalibrationOffsets();

    /**
     * Writes the given calibration offsets to the MPU6500's hardware offset registers.
     * The caller can use this method to apply new offsets to the sensor, either as
     * part of a calibration process or to restore previously saved offsets. The offsets
     * should be provided as a cal_offsets struct, which contains the accelerometer and gyroscope
     * offsets as 16-bit signed integers. The method will write the offsets to the appropriate
     * registers on the MPU6500.
     */
    void WriteCalibrationOffsets(const cal_offsets& offsets);
protected:
    /* Replace this with a GPIO read implementation if needed */
    virtual bool IsDataReady();

    MPU6500_IFS_Base& ifs_;
    void SetBit(uint8_t reg, uint8_t bit_mask);
    void ClearBit(uint8_t reg, uint8_t bit_mask);
    int16_t ReadReg_s16(uint8_t reg);
    void WriteReg_s16(uint8_t reg, int16_t value);
    uint16_t GetFIFOCount();
    void ResetFIFO();
    Acc_Gain _acc_gain = Acc_Gain::SCALE_2G;
    Gyro_Gain _gyro_gain = Gyro_Gain::GYRO_0250DS;
};

/**
 * MPU6500_DMP_Base is an abstract base class for MPU6500 drivers that utilize
 * the Digital Motion Processor (DMP) firmware. It provides common functionality
 * for initializing the DMP, reading DMP packets from the FIFO buffer, and
 * converting raw DMP data into real-world IMU data (roll, pitch, yaw).
 */
class MPU6500_DMP_Base : public MPU6500_Base {
public:
    using MPU6500_Base::MPU6500_Base;

    /**
     * @brief Initializes the MPU6500 with DMP support. This method uploads the DMP firmware to the sensor,
     * configures the necessary registers, and prepares the sensor for DMP operation.
     */
    int Init();

    /**
     * @brief Structure to hold real-world IMU data.
     */
    struct RealIMUData { float roll, pitch, yaw; };

    /**
     * @brief Retrieves real-world IMU data from the DMP. This method reads a DMP packet from the FIFO buffer,
     * parses the quaternion data, converts it to Euler angles (roll, pitch, yaw),
     * and calculates the angular velocity and linear acceleration in real-world units.
     * @return A RealIMUData struct containing roll, pitch, yaw angles in degrees, angular
     * velocity in degrees per second, and linear acceleration in m/s².
     * @note This method assumes that the DMP is properly initialized and that new data is available in the FIFO buffer.
     * It is recommended to call IsDataReady() before calling this method to ensure that new data is available.
     * @see IsDataReady() to check for new data availability.
     * @see WaitForRealIMUData() to block until new data is available and retrieve it in one call.
     */
    RealIMUData GetRealIMUData();

    /**
     * @brief Waits for new real-world IMU data to be available and retrieves it. This method blocks until
     * new data is ready in the FIFO buffer, then calls GetRealIMUData() to parse and return the data.
     * @return A RealIMUData struct containing roll, pitch, yaw angles in degrees, angular velocity
     * in degrees per second, and linear acceleration in m/s².
     * @note This method is useful for applications that require blocking behavior until new data is available.
     * It is recommended to use this method in conjunction with IsDataReady() to avoid unnecessary blocking
     * if new data is already available.
     * @see GetRealIMUData() to retrieve the latest real-world IMU data without blocking.
     * @see IsDataReady() to check for new data availability before calling this method.
     */
    RealIMUData WaitForRealIMUData();

protected:
    virtual size_t GetDMPPacketSize() const { return 14; } // Default DMP packet size is 16 bytes (quaternion data)

    /**
     * @brief Checks if new DMP data is available in the FIFO buffer. This method reads the FIFO count
     * and determines if a complete DMP packet is available for reading.
     * @return true if new DMP data is available, false otherwise.
     * @note This method should be called before attempting to read DMP data to ensure that a complete
     * packet is available. It is recommended to use this method in conjunction with WaitForRealIMUData()
     * to block until new data is available, or to implement a non-blocking polling mechanism for applications
     * that require continuous data processing.
     * Override this method in derived classes if a different mechanism for checking data availability
     * is required (e.g., using interrupts or other signaling mechanisms).
     */
    virtual bool IsDataReady() override;

    /**
     * @brief Configures the DMP firmware and initializes the necessary registers for DMP operation.
     * This method is called during the Init() process and should be overridden in derived classes
     * to provide the specific DMP firmware upload and configuration logic for the MPU6500.
     * @note This method is intended to be implemented by derived classes that provide specific DMP firmware support.
     * The base class provides a default implementation that does nothing, and it is expected that derived
     * classes will override this method to perform the necessary DMP configuration.
     */
    virtual void ConfigureDMP();
};

#endif /* MPU6500_H */
