#pragma once

#include <Arduino.h>
#include <Wire.h>

struct Mpu6500RawData
{
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;

    int16_t temperature;

    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
};

struct Mpu6500Data
{
    float accelXG;
    float accelYG;
    float accelZG;

    float accelXMps2;
    float accelYMps2;
    float accelZMps2;

    float gyroXDps;
    float gyroYDps;
    float gyroZDps;

    float temperatureC;
};

class Mpu6500
{
public:
    bool begin(
        TwoWire &wirePort,
        uint8_t sdaPin,
        uint8_t sclPin,
        uint32_t clockFrequency = 100000);

    bool isConnected() const;
    uint8_t getAddress() const;
    uint8_t getWhoAmI() const;

    bool readRaw(Mpu6500RawData &data);
    bool read(Mpu6500Data &data);

private:
    static constexpr uint8_t ADDRESS_LOW = 0x68;
    static constexpr uint8_t ADDRESS_HIGH = 0x69;

    static constexpr uint8_t REGISTER_WHO_AM_I = 0x75;
    static constexpr uint8_t REGISTER_PWR_MGMT_1 = 0x6B;
    static constexpr uint8_t REGISTER_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REGISTER_GYRO_CONFIG = 0x1B;
    static constexpr uint8_t REGISTER_ACCEL_XOUT_H = 0x3B;

    static constexpr float ACCEL_SCALE_LSB_PER_G = 16384.0f;
    static constexpr float GYRO_SCALE_LSB_PER_DPS = 131.0f;
    static constexpr float GRAVITY_MPS2 = 9.80665f;

    TwoWire *wire = nullptr;

    uint8_t deviceAddress = 0;
    uint8_t whoAmI = 0;

    bool connected = false;

    bool detectDevice();
    bool readRegister(uint8_t registerAddress, uint8_t &value);
    bool writeRegister(uint8_t registerAddress, uint8_t value);

    bool readRegisters(
        uint8_t startRegister,
        uint8_t *buffer,
        uint8_t length);

    static int16_t combineBytes(
        uint8_t highByte,
        uint8_t lowByte);
};