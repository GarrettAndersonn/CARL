#include "mpu6500.h"

bool Mpu6500::begin(
    TwoWire &wirePort,
    uint8_t sdaPin,
    uint8_t sclPin,
    uint32_t clockFrequency)
{
    wire = &wirePort;

    wire->setSDA(sdaPin);
    wire->setSCL(sclPin);
    wire->begin();
    wire->setClock(clockFrequency);

    delay(100);

    if (!detectDevice())
    {
        connected = false;
        return false;
    }

    if (!writeRegister(REGISTER_PWR_MGMT_1, 0x00))
    {
        connected = false;
        return false;
    }

    delay(100);

    // Accelerometer: ±2 g
    if (!writeRegister(REGISTER_ACCEL_CONFIG, 0x00))
    {
        connected = false;
        return false;
    }

    // Gyroscope: ±250 degrees per second
    if (!writeRegister(REGISTER_GYRO_CONFIG, 0x00))
    {
        connected = false;
        return false;
    }

    connected = true;
    return true;
}

bool Mpu6500::isConnected() const
{
    return connected;
}

uint8_t Mpu6500::getAddress() const
{
    return deviceAddress;
}

uint8_t Mpu6500::getWhoAmI() const
{
    return whoAmI;
}

bool Mpu6500::detectDevice()
{
    const uint8_t addresses[] = {
        ADDRESS_LOW,
        ADDRESS_HIGH
    };

    for (const uint8_t address : addresses)
    {
        wire->beginTransmission(address);

        if (wire->endTransmission() != 0)
        {
            continue;
        }

        deviceAddress = address;

        uint8_t detectedId = 0;

        if (!readRegister(REGISTER_WHO_AM_I, detectedId))
        {
            continue;
        }

        whoAmI = detectedId;

        if (whoAmI == 0x70)
        {
            connected = true;
            return true;
        }
    }

    deviceAddress = 0;
    whoAmI = 0;
    connected = false;

    return false;
}

bool Mpu6500::readRegister(
    uint8_t registerAddress,
    uint8_t &value)
{
    if (wire == nullptr || deviceAddress == 0)
    {
        return false;
    }

    wire->beginTransmission(deviceAddress);
    wire->write(registerAddress);

    if (wire->endTransmission(false) != 0)
    {
        return false;
    }

    const uint8_t received =
        wire->requestFrom(
            deviceAddress,
            static_cast<uint8_t>(1));

    if (received != 1 || !wire->available())
    {
        return false;
    }

    value = wire->read();
    return true;
}

bool Mpu6500::writeRegister(
    uint8_t registerAddress,
    uint8_t value)
{
    if (wire == nullptr || deviceAddress == 0)
    {
        return false;
    }

    wire->beginTransmission(deviceAddress);
    wire->write(registerAddress);
    wire->write(value);

    return wire->endTransmission() == 0;
}

bool Mpu6500::readRegisters(
    uint8_t startRegister,
    uint8_t *buffer,
    uint8_t length)
{
    if (wire == nullptr ||
        deviceAddress == 0 ||
        buffer == nullptr ||
        length == 0)
    {
        return false;
    }

    wire->beginTransmission(deviceAddress);
    wire->write(startRegister);

    if (wire->endTransmission(false) != 0)
    {
        return false;
    }

    const uint8_t received =
        wire->requestFrom(deviceAddress, length);

    if (received != length)
    {
        return false;
    }

    for (uint8_t index = 0; index < length; ++index)
    {
        if (!wire->available())
        {
            return false;
        }

        buffer[index] = wire->read();
    }

    return true;
}

int16_t Mpu6500::combineBytes(
    uint8_t highByte,
    uint8_t lowByte)
{
    return static_cast<int16_t>(
        (static_cast<uint16_t>(highByte) << 8) |
        static_cast<uint16_t>(lowByte));
}

bool Mpu6500::readRaw(Mpu6500RawData &data)
{
    uint8_t buffer[14];

    if (!readRegisters(
            REGISTER_ACCEL_XOUT_H,
            buffer,
            sizeof(buffer)))
    {
        connected = false;
        return false;
    }

    data.accelX = combineBytes(buffer[0], buffer[1]);
    data.accelY = combineBytes(buffer[2], buffer[3]);
    data.accelZ = combineBytes(buffer[4], buffer[5]);

    data.temperature =
        combineBytes(buffer[6], buffer[7]);

    data.gyroX = combineBytes(buffer[8], buffer[9]);
    data.gyroY = combineBytes(buffer[10], buffer[11]);
    data.gyroZ = combineBytes(buffer[12], buffer[13]);

    connected = true;
    return true;
}

bool Mpu6500::read(Mpu6500Data &data)
{
    Mpu6500RawData raw;

    if (!readRaw(raw))
    {
        return false;
    }

    data.accelXG =
        static_cast<float>(raw.accelX) /
        ACCEL_SCALE_LSB_PER_G;

    data.accelYG =
        static_cast<float>(raw.accelY) /
        ACCEL_SCALE_LSB_PER_G;

    data.accelZG =
        static_cast<float>(raw.accelZ) /
        ACCEL_SCALE_LSB_PER_G;

    data.accelXMps2 =
        data.accelXG * GRAVITY_MPS2;

    data.accelYMps2 =
        data.accelYG * GRAVITY_MPS2;

    data.accelZMps2 =
        data.accelZG * GRAVITY_MPS2;

    data.gyroXDps =
        static_cast<float>(raw.gyroX) /
        GYRO_SCALE_LSB_PER_DPS;

    data.gyroYDps =
        static_cast<float>(raw.gyroY) /
        GYRO_SCALE_LSB_PER_DPS;

    data.gyroZDps =
        static_cast<float>(raw.gyroZ) /
        GYRO_SCALE_LSB_PER_DPS;

    data.temperatureC =
        static_cast<float>(raw.temperature) /
        333.87f +
        21.0f;

    return true;
}