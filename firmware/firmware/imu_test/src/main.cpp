#include <Arduino.h>
#include <Wire.h>

#include "mpu6500.h"

Mpu6500 imu;

uint32_t lastReportMs = 0;

void setup()
{
    Serial.begin(115200);
    delay(1500);

    Serial.println();
    Serial.println(F("CARL MPU-6500 LIBRARY TEST"));

    const bool started =
        imu.begin(
            Wire,
            PB9,
            PB8,
            100000);

    if (!started)
    {
        Serial.println(F("ERROR: MPU-6500 initialization failed"));
        return;
    }

    Serial.print(F("MPU-6500 detected at 0x"));
    Serial.println(imu.getAddress(), HEX);

    Serial.print(F("WHO_AM_I = 0x"));
    Serial.println(imu.getWhoAmI(), HEX);
}

void loop()
{
    const uint32_t now = millis();

    if (now - lastReportMs < 100)
    {
        return;
    }

    lastReportMs = now;

    if (!imu.isConnected())
    {
        Serial.println(F("ERROR: MPU-6500 disconnected"));
        return;
    }

    Mpu6500Data data;

    if (!imu.read(data))
    {
        Serial.println(F("ERROR: MPU-6500 read failed"));
        return;
    }

    Serial.print(F("IMU"));

    Serial.print(F(",ax_mps2="));
    Serial.print(data.accelXMps2, 4);

    Serial.print(F(",ay_mps2="));
    Serial.print(data.accelYMps2, 4);

    Serial.print(F(",az_mps2="));
    Serial.print(data.accelZMps2, 4);

    Serial.print(F(",gx_dps="));
    Serial.print(data.gyroXDps, 4);

    Serial.print(F(",gy_dps="));
    Serial.print(data.gyroYDps, 4);

    Serial.print(F(",gz_dps="));
    Serial.print(data.gyroZDps, 4);

    Serial.print(F(",temp_c="));
    Serial.println(data.temperatureC, 2);
}