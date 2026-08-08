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
#include "imu.h"
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
class MPU6500_Base : public virtual IMU::IDevice
{
public:
    explicit MPU6500_Base() = delete;
    virtual ~MPU6500_Base() = default;

    /**
     * Constructor that takes an I2C device interface. The caller is responsible for ensuring
     * that the I2C device is properly initialized and configured before calling Init().
     */
    MPU6500_Base(MPU6500_IFS_Base& ifs) : ifs_(ifs) {}

    int CheckWhoIam() override;

    int Init() override;

    int Reset() override;

    void SetAccGain(AccelGain gain) override;

    void SetGyroGain(GyroGain gain) override;

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
	 * @brief Holds raw accelerometer and gyroscope data from the sensor.
	 */
	class RawData: public IMU::RawData_6DOF_Base {
	public:
		float GetTemperature() const override {
			return (temperature / 333.87f) + 21.0f;
		}
		float GetTempRaw() const {
			return temperature;
		}
		using RawData_6DOF_Base::RawData_6DOF_Base;
	};

    virtual RawData& ReadData() override;

    int Calibrate(int max_iterations = 100, int16_t target_error = 50) override;

    CalibrationData ReadCalibrationOffsets() override;

    void WriteCalibrationOffsets(CalibrationData offsets) override;

    const char *GetName() const override { return "MPU6500"; }
    virtual const char *GetDescription() const override { return "MPU6500 6DOF 16-bit accelerometer and gyroscope"; }
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
    AccelGain _acc_gain = AccelGain::ACC_GAIN_2G;
    GyroGain _gyro_gain = GyroGain::GYRO_GAIN_250DS;
    RawData cached_data;
};

using IMU_DMP_DeviceBase = IMU::IDeviceDMP<double>;
using IMU_DMP_Quaternion = IMU::Quaternion<double>;

/**
 * MPU6500_DMP is a derived class that extends MPU6500_Base and implements
 * the IMU_DMP_DeviceBase interface.
 * It provides functionality to read quaternion data from the MPU6500's
 * Digital Motion Processor (DMP) and convert it into real-world IMU data.
 * The class also includes methods to initialize the DMP firmware,
 * read DMP packets from the FIFO buffer, and handle data ready interrupts.
 * Derived classes can implement specific DMP firmware upload and packet
 * parsing logic as needed.
 */
class MPU6500_DMP : public virtual IMU_DMP_DeviceBase, public MPU6500_Base {
public:
    using MPU6500_Base::MPU6500_Base;

    int Init() override;

    IMU_DMP_Quaternion& ReadQuaternion() override;

    virtual const char *GetName() const override { return "MPU6500 with DMP"; }
    virtual const char *GetDescription() const override { return MPU6500_Base::GetDescription(); }
protected:
    virtual bool IsDataReady() override;
private:
    IMU_DMP_Quaternion cached_quaternion;
};

#endif /* MPU6500_H */
