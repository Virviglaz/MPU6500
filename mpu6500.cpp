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
 * MPU-6500 C++ driver implementation.
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

#include "mpu6500.h"
#include "bitops.h"
#include <errno.h>
#include <cmath>

#define MPU6500_XA_OFFS_H       0x77
#define MPU6500_XG_OFFS_H       0x13
#define MPU6500_WHO_AM_I        0x75
#define MPU6500_PWR_MGMT_1      0x6B
#define MPU6500_PWR_MGMT_2      0x6C
#define MPU6500_CONFIG          0x1A
#define MPU6500_SMPLRT_DIV      0x19
#define MPU6500_INT_PIN_CFG     0x37
#define MPU6500_INT_ENABLE      0x38
#define MPU6500_INT_STATUS      0x3A
#define MPU6500_ACCEL_CONFIG    0x1C
#define MPU6500_GYRO_CONFIG     0x1B
#define MPU6500_ACCEL_XOUT_H    0x3B
#define MPU6500_FIFO_R_W        0x74
#define MPU6500_FIFO_COUNTH     0x72
#define MPU6500_FIFO_ENABLE     0x23
#define MPU6500_USER_CTRL       0x6A
#define MPU6500_BANK_SEL        0x6D
#define MPU6500_MEM_START_ADDR  0x6E
#define MPU6500_MEM_R_W         0x6F
#define PRGM_START_H            0x70

/****************************** MPU6500_IFS_Base *****************************/
void MPU6500_IFS_Base::SetBit(uint8_t reg, uint8_t bit_mask)
{
    uint8_t value;
    Read(&reg, 1, &value, 1);
    value |= bit_mask;
    Write(&reg, 1, &value, 1);
}

void MPU6500_IFS_Base::ClrBit(uint8_t reg, uint8_t bit_mask)
{
    uint8_t value;
    Read(&reg, 1, &value, 1);
    value &= ~bit_mask;
    Write(&reg, 1, &value, 1);
}

void MPU6500_IFS_Base::WriteReg(uint8_t reg, uint8_t value)
{
    Write(&reg, 1, &value, 1);
}

void MPU6500_IFS_Base::WriteReg_s16(uint8_t reg, int16_t value)
{
    int16_t be_value = ToNative<Endian::Big>(value);
    Write(&reg, 1, reinterpret_cast<uint8_t*>(&be_value), 2);
}

int16_t MPU6500_IFS_Base::ReadReg_s16(uint8_t reg)
{
    int16_t value;
    Read(&reg, 1, reinterpret_cast<uint8_t*>(&value), 2);
    return ToNative<Endian::Big>(value);
}

uint8_t MPU6500_IFS_Base::ReadReg(uint8_t reg)
{
    uint8_t value;
    Read(&reg, 1, &value, 1);
    return value;
}

void MPU6500_IFS_Base::Read(uint8_t reg, uint8_t* data, size_t length)
{
    Read(&reg, 1, data, length);
}

void MPU6500_IFS_Base::Write(uint8_t reg, const uint8_t* data, size_t length)
{
    Write(&reg, 1, data, length);
}

void MPU6500_IFS_Base::ReadFIFO(uint8_t *buf, size_t size)
{
    Read(MPU6500_FIFO_R_W, buf, size);
}

/****************************** MPU6500 I2C *****************************/
void MPU6500_I2C::Write(const uint8_t *reg_addr,
                         size_t reg_addr_size,
                         const uint8_t *data,
                         size_t length)
{
    device_.Write(reg_addr, reg_addr_size, data, length);
}

void MPU6500_I2C::Read(const uint8_t *reg_addr,
                        size_t reg_addr_size,
                        uint8_t *data,
                        size_t length)
{
    device_.Read(reg_addr, reg_addr_size, data, length);
}

/****************************** MPU6500 SPI *****************************/
void MPU6500_SPI::Write(const uint8_t *reg_addr,
                         size_t reg_addr_size,
                         const uint8_t *data,
                         size_t length)
{
    // SPI write operation: MSB of register address should be 0
    memcpy(buffer_, reg_addr, reg_addr_size);
    memcpy(buffer_ + reg_addr_size, data, length);
    buffer_[0] &= 0x7F; // Update the register address with MSB cleared
    device_.Transfer(buffer_, nullptr, reg_addr_size + length); // Send register address and data
}

void MPU6500_SPI::Read(const uint8_t *reg_addr,
                        size_t reg_addr_size,
                        uint8_t *data,
                        size_t length)
{
    // SPI read operation: MSB of register address should be 1
    memcpy(buffer_, reg_addr, reg_addr_size);
    buffer_[0] |= 0x80; // Update the register address with MSB set
    device_.Transfer(buffer_, buffer_, reg_addr_size + length); // Send register address and read data into buffer
    memcpy(data, buffer_ + reg_addr_size, length); // Copy the read data to the output buffer
}

int MPU6500_Base::Init()
{
    if (ifs_.ReadReg(MPU6500_WHO_AM_I) != 0x70)
        return -ENODEV;

    /* Minimal configuration */
    ifs_.WriteReg(MPU6500_PWR_MGMT_1, 0);
    return 0;
}

void MPU6500_Base::Reset()
{
    ifs_.SetBit(MPU6500_PWR_MGMT_1, BIT(7)); // Set the reset bit
}

bool MPU6500_Base::IsDataReady()
{
    return ifs_.ReadReg(MPU6500_INT_STATUS) & BIT(0);
}

MPU6500_Base::RawData MPU6500_Base::WaitForData()
{
    while (!MPU6500_Base::IsDataReady()) {};
    return GetData();
}

void MPU6500_Base::SetAccGain(MPU6500_Base::Acc_Gain gain)
{
     uint8_t reg_val = static_cast<uint8_t>(gain) << 3;
    ifs_.WriteReg(MPU6500_ACCEL_CONFIG, reg_val);
    _acc_gain = gain;
}

void MPU6500_Base::SetGyroGain(MPU6500_Base::Gyro_Gain gain)
{
    uint8_t reg_val = static_cast<uint8_t>(gain) << 3;
    ifs_.WriteReg(MPU6500_GYRO_CONFIG, reg_val);
    _gyro_gain = gain;
}

void MPU6500_Base::SetSampleRate(uint8_t sample_rate_hz)
{
    if (sample_rate_hz == 0) sample_rate_hz = 1;
     int32_t div = 1000 / sample_rate_hz - 1;
    
    if (div < 0)   div = 0;
    if (div > 255) div = 255;
    
    ifs_.WriteReg(MPU6500_SMPLRT_DIV, static_cast<uint8_t>(div));
}

void MPU6500_Base::SetFilterOrder(uint8_t filter_order)
{
    ifs_.WriteReg(MPU6500_CONFIG, filter_order & 0x07);
}

void MPU6500_Base::EnableDataReadyInterrupt()
{
    /* LATCH_INT_EN + INT_RD_CLEAR */
    ifs_.WriteReg(MPU6500_INT_PIN_CFG, 0x30);
}

MPU6500_Base::RawData MPU6500_Base::GetData()
{
    RawData data(_acc_gain, _gyro_gain);

    ifs_.Read(MPU6500_ACCEL_XOUT_H, reinterpret_cast<uint8_t*>(&data), 14);

    data.x = ToNative<Endian::Big>(data.x);
    data.y = ToNative<Endian::Big>(data.y);
    data.z = ToNative<Endian::Big>(data.z);
    data.temp = ToNative<Endian::Big>(data.temp);
    data.ax = ToNative<Endian::Big>(data.ax);
    data.ay = ToNative<Endian::Big>(data.ay);
    data.az = ToNative<Endian::Big>(data.az);

    return data;
}

float MPU6500_Base::RawData::GetTemperature() const
{
    return (static_cast<float>(temp) / 333.87f) + 21.0f;
}

/* Convert to real data */
float MPU6500_Base::RawData::GetAccX() const { return (static_cast<float>(x) * static_cast<float>(1 << static_cast<int>(_acc_gain))) / 16384.0f; }
float MPU6500_Base::RawData::GetAccY() const { return (static_cast<float>(y) * static_cast<float>(1 << static_cast<int>(_acc_gain))) / 16384.0f; }
float MPU6500_Base::RawData::GetAccZ() const { return (static_cast<float>(z) * static_cast<float>(1 << static_cast<int>(_acc_gain))) / 16384.0f; }
float MPU6500_Base::RawData::GetGyroX() const { return (static_cast<float>(ax) * static_cast<float>(1 << static_cast<int>(_gyro_gain))) / 131.0f; }
float MPU6500_Base::RawData::GetGyroY() const { return (static_cast<float>(ay) * static_cast<float>(1 << static_cast<int>(_gyro_gain))) / 131.0f; }
float MPU6500_Base::RawData::GetGyroZ() const { return (static_cast<float>(az) * static_cast<float>(1 << static_cast<int>(_gyro_gain))) / 131.0f; }

uint16_t MPU6500_Base::GetFIFOCount()
{
    return ifs_.ReadReg_s16(MPU6500_FIFO_COUNTH);
}

MPU6500_Base::cal_offsets MPU6500_Base::ReadCalibrationOffsets()
{
    cal_offsets offsets;
    uint8_t buf[6];

    // 1. Safe packet read from I2C into a local byte array (No Strict Aliasing violation)
    ifs_.Read(MPU6500_XA_OFFS_H, buf, 6);
    
    // 2. Reassemble sign-correct values from Big-Endian array
    // We do sign extension by casting the high byte to int8_t before shifting
    offsets.acc_x_offset = ((static_cast<int16_t>(static_cast<int8_t>(buf[0])) << 8) | buf[1]) >> 1;
    offsets.acc_y_offset = ((static_cast<int16_t>(static_cast<int8_t>(buf[2])) << 8) | buf[3]) >> 1;
    offsets.acc_z_offset = ((static_cast<int16_t>(static_cast<int8_t>(buf[4])) << 8) | buf[5]) >> 1;

    // 3. Safe packet read for Gyroscope
    ifs_.Read(MPU6500_XG_OFFS_H, buf, 6);
    offsets.gyro_x_offset = (static_cast<int16_t>(static_cast<int8_t>(buf[0])) << 8) | buf[1];
    offsets.gyro_y_offset = (static_cast<int16_t>(static_cast<int8_t>(buf[2])) << 8) | buf[3];
    offsets.gyro_z_offset = (static_cast<int16_t>(static_cast<int8_t>(buf[4])) << 8) | buf[5];

    return offsets;
}

void MPU6500_Base::WriteCalibrationOffsets(const cal_offsets& offsets)
{
    uint8_t buf[6];

    // 1. Pack Accelerometer offsets safely (Using const input, no user data corruption)
    // Shift left by 1 and force the temperature compensation bit to 1
    int16_t x = (offsets.acc_x_offset << 1) | 1;
    int16_t y = (offsets.acc_y_offset << 1) | 1;
    int16_t z = (offsets.acc_z_offset << 1) | 1;

    buf[0] = static_cast<uint8_t>((x >> 8) & 0xFF);  buf[1] = static_cast<uint8_t>(x & 0xFF);
    buf[2] = static_cast<uint8_t>((y >> 8) & 0xFF);  buf[3] = static_cast<uint8_t>(y & 0xFF);
    buf[4] = static_cast<uint8_t>((z >> 8) & 0xFF);  buf[5] = static_cast<uint8_t>(z & 0xFF);
    ifs_.Write(MPU6500_XA_OFFS_H, buf, 6);

    // 2. Pack Gyroscope offsets safely
    buf[0] = static_cast<uint8_t>((offsets.gyro_x_offset >> 8) & 0xFF); buf[1] = static_cast<uint8_t>(offsets.gyro_x_offset & 0xFF);
    buf[2] = static_cast<uint8_t>((offsets.gyro_y_offset >> 8) & 0xFF); buf[3] = static_cast<uint8_t>(offsets.gyro_y_offset & 0xFF);
    buf[4] = static_cast<uint8_t>((offsets.gyro_z_offset >> 8) & 0xFF); buf[5] = static_cast<uint8_t>(offsets.gyro_z_offset & 0xFF);
    ifs_.Write(MPU6500_XG_OFFS_H, buf, 6);
}

#ifdef DEBUG
#include <stdio.h>
#endif
int MPU6500_Base::Calibrate(int max_iterations, int16_t target_error)
{
    WriteCalibrationOffsets(MPU6500_Base::cal_offsets()); // Clear existing offsets to start calibration from a known state
    int res = -EFAULT;

    cal_offsets offsets;

    for (int i = 0; i != max_iterations; i++) {
        RawData data = WaitForData();

        /* Lambda function to divide by 64 with rounding */
        auto divide_by_64_fast = [](int16_t x) -> int16_t {
            return (x + ((x >> 15) & 63)) >> 6;
        };

        offsets.acc_x_offset  -= divide_by_64_fast(data.x);
        offsets.acc_y_offset  -= divide_by_64_fast(data.y);
        offsets.acc_z_offset  -= divide_by_64_fast(data.z);
        offsets.gyro_x_offset -= divide_by_64_fast(data.ax);
        offsets.gyro_y_offset -= divide_by_64_fast(data.ay);
        offsets.gyro_z_offset -= divide_by_64_fast(data.az);

        WriteCalibrationOffsets(offsets); // Apply new offsets to the sensor

        /* Calculate the maximum error between the current readings and the target values */
        auto get_error = [&data]() {
            int16_t *ptr = reinterpret_cast<int16_t*>(&data);
            int16_t max_error = 0;
            for (size_t k = 0; k < 7; k++) {
                int16_t error = std::abs(*ptr++);
                if (k != 3 && error > max_error)
                    max_error = error;
            }
            return max_error;
        };

        /* Measure the maximum error and break if within target */
        int16_t error = get_error();
#ifdef DEBUG
        printf("Iter %3d: x=%6d y=%6d z=%6d gx=%6d gy=%6d gz=%6d Err=%d\n",
               i + 1, data.x, data.y, data.z, data.ax, data.ay, data.az, error);
#endif
        if (error < target_error) {
            res = 0;
            break; // Calibration successful if all readings are within target_error LSB of target
        }
    }

    ResetFIFO(); // Clear FIFO to remove any residual data after calibration
    return res;
}

void MPU6500_Base::ResetFIFO()
{
    ifs_.SetBit(MPU6500_USER_CTRL, BIT(2)); // Set the FIFO reset bit
    while (1) {
        if ((ifs_.ReadReg(MPU6500_USER_CTRL) & BIT(2)) == 0)
            break;
    };
}

int MPU6500_DMP_Base::Init()
{
    int res = MPU6500_Base::Init();
    if (res != 0) return res;

    /* Load DMP firmware */
    const uint8_t dmp_img[] = {
        #include "dmp_image.h"
    };
    const size_t DMP_BANK_SIZE = 256;
    const size_t DMP_CHUNK_SIZE = 16;

    uint8_t bank = 0;
    uint8_t address = 0;
    size_t bytes_written = 0;

    while (bytes_written < sizeof(dmp_img)) {
        /* 1. Select the appropriate bank and address based on how many bytes we've already written to the DMP memory. */
        bank = static_cast<uint8_t>(bytes_written / DMP_BANK_SIZE);
        address = static_cast<uint8_t>(bytes_written % DMP_BANK_SIZE);

        /* 2. Choose the bank by writing to the BANK_SEL register (0x6D) */
        ifs_.WriteReg(MPU6500_BANK_SEL, bank);

        /* 3. Set the start address within the bank (Register 0x6E) */
        ifs_.WriteReg(MPU6500_MEM_START_ADDR, address);

        /*
         * 4. Calculate how many bytes we will send in the current transaction
         * (No more than the chunk limit and not exceeding the 256-byte bank boundary)
         */
        size_t current_chunk = DMP_CHUNK_SIZE;
        if (address + current_chunk > DMP_BANK_SIZE) {
            current_chunk = DMP_BANK_SIZE - address;
        }
        if (bytes_written + current_chunk > sizeof(dmp_img)) {
            current_chunk = sizeof(dmp_img) - bytes_written;
        }

        /*
         * 5. Write the block of data to the memory port 0x6F.
         * Use your corrected Write method that combines
         * register address and data into a single transaction.
         * The register address for the memory port is 0x6F.
         */
        ifs_.Write(MPU6500_MEM_R_W, &dmp_img[bytes_written], current_chunk);

        bytes_written += current_chunk;
    }

    /* Enable FIFO */
    ifs_.WriteReg(MPU6500_FIFO_ENABLE, BIT(7) | BIT(6) | BIT(5) | BIT(4) | BIT(3)); // Enable FIFO for accelerometer and gyroscope
    ifs_.WriteReg(MPU6500_USER_CTRL, BIT(7) | BIT(6) | BIT(3)); // Enable DMP and FIFO

    ifs_.WriteReg_s16(PRGM_START_H, 0x0400); // Set the DMP program start address to 0x0400

    ConfigureDMP();

    ResetFIFO(); // Clear any existing data in the FIFO buffer

    return 0;
}

bool MPU6500_DMP_Base::IsDataReady()
{
    return GetFIFOCount() >= GetDMPPacketSize(); // DMP packet size is 16 bytes
}

void MPU6500_DMP_Base::ConfigureDMP()
{
    /* Configure DMP for 16 bytes package */
    ifs_.WriteReg(MPU6500_BANK_SEL, 0x03);
    ifs_.WriteReg(MPU6500_MEM_START_ADDR, 0xAC);
    uint8_t quat_mask[2] = { 0x08, 0x00 }; // INV_DMP_32_BIT_QUAT
    ifs_.Write(MPU6500_MEM_R_W, quat_mask, 2);
    ifs_.WriteReg(MPU6500_BANK_SEL, 0x03);
    ifs_.WriteReg(MPU6500_MEM_START_ADDR, 0xB2);
    uint8_t clear_mask[2] = { 0x00, 0x00 };
    ifs_.Write(MPU6500_MEM_R_W, clear_mask, 2);
}

MPU6500_DMP_Base::RealIMUData MPU6500_DMP_Base::GetRealIMUData()
{
    #pragma pack(push, 1)
    struct {
        int32_t x, y, z;
        uint16_t footer;
    } raw_packet;
    #pragma pack(pop)

    static_assert(sizeof(raw_packet) == 14, "DMP packet size mismatch");

    ifs_.ReadFIFO(reinterpret_cast<uint8_t *>(&raw_packet), sizeof(raw_packet));

    raw_packet.x = ToNative<Endian::Big>(raw_packet.x);
    raw_packet.y = ToNative<Endian::Big>(raw_packet.y);
    raw_packet.z = ToNative<Endian::Big>(raw_packet.z);

    const constexpr float scale = 1.0f / 1073741824.0f; // 2^30
    float q_x = static_cast<float>(raw_packet.x) * scale;
    float q_y = static_cast<float>(raw_packet.y) * scale;
    float q_z = static_cast<float>(raw_packet.z) * scale;

    float w_sq = 1.0f - (q_x * q_x + q_y * q_y + q_z * q_z);
    float q_w = (w_sq > 0.0f) ? sqrtf(w_sq) : 0.0f;

    float asin_arg = 2.0f * (q_w * q_y - q_x * q_z);
    if (asin_arg > 1.0f)  asin_arg = 1.0f;
    if (asin_arg < -1.0f) asin_arg = -1.0f;

    const float rad_to_deg = 57.2957795f;
    RealIMUData output;
    output.pitch = asinf(asin_arg) * rad_to_deg;
    output.roll  = atan2f(2.0f * (q_w * q_x + q_y * q_z), 1.0f - 2.0f * (q_x * q_x + q_y * q_y)) * rad_to_deg;
    output.yaw   = atan2f(2.0f * (q_w * q_z + q_x * q_y), 1.0f - 2.0f * (q_y * q_y + q_z * q_z)) * rad_to_deg;

    return output;
}

MPU6500_DMP_Base::RealIMUData MPU6500_DMP_Base::WaitForRealIMUData()
{
    while (!IsDataReady()) {};

    return GetRealIMUData();
}
