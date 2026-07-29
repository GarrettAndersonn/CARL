#include <Arduino.h>
#include <STM32_CAN.h>

#include "mpu6500.h"
#include "carl_protocol.h"
#include "messages.h"

using namespace carl;

// -----------------------------------------------------------------------------
// Hardware and companion-link configuration
// -----------------------------------------------------------------------------

static constexpr uint32_t COMPANION_BAUD = 115200;

static constexpr uint8_t MPU_ADDRESS_LOW = 0x68;
static constexpr uint8_t MPU_ADDRESS_HIGH = 0x69;
static constexpr uint8_t MPU_WHO_AM_I_REGISTER = 0x75;

// Companion-computer binary frame:
// [0xAA][MSG_ID low][MSG_ID high][LEN][PAYLOAD...][CRC8]
static constexpr uint8_t COMPANION_SOF = 0xAA;
static constexpr uint8_t COMPANION_MAX_PAYLOAD = 8;
static constexpr uint8_t COMPANION_MAX_FRAME_SIZE =
    1 + 2 + 1 + COMPANION_MAX_PAYLOAD + 1;

// -----------------------------------------------------------------------------
// Hardware objects
// -----------------------------------------------------------------------------

STM32_CAN Can1(CAN1, ALT_2);  // PD0 RX, PD1 TX
Mpu6500 imu;

// -----------------------------------------------------------------------------
// IMU state
// -----------------------------------------------------------------------------

static bool imuDetected = false;
static uint8_t imuAddress = 0;
static uint8_t imuWhoAmI = 0;

bool imuReady = false;
uint32_t lastImuMs = 0;
uint32_t imuReadErrorCount = 0;

Mpu6500Data latestImuData = {};

// -----------------------------------------------------------------------------
// Vehicle-command state
// -----------------------------------------------------------------------------

int16_t teleopVx = 0;
int16_t teleopWz = 0;

bool estopLatched = false;

int testIdx = -1;
int16_t testVal = 0;

// Direct four-wheel command received from ROS 2.
ThrottleCmd companionThrottleCmd = {0, 0, 0, 0};
bool companionThrottleActive = false;

// -----------------------------------------------------------------------------
// Feedback state
// -----------------------------------------------------------------------------

ThrottleStatus lastStatus = {TST_INIT, TF_NONE, 0, 0};
EncoderFb lastEnc = {0, 0};

bool haveStatus = false;
bool haveEncoderFeedback = false;

// -----------------------------------------------------------------------------
// Timing state
// -----------------------------------------------------------------------------

uint32_t lastCmdMs = 0;
uint32_t lastHbMs = 0;
uint32_t lastTlmMs = 0;
uint32_t lastImuReportMs = 0;
uint32_t lastTeleopMs = 0;

uint8_t hbCounter = 0;

// -----------------------------------------------------------------------------
// Text-command parser state
// -----------------------------------------------------------------------------

char lineBuf[64];
uint8_t lineLen = 0;

// -----------------------------------------------------------------------------
// Binary companion-frame parser state
// -----------------------------------------------------------------------------

uint8_t companionRxFrame[COMPANION_MAX_FRAME_SIZE];
uint8_t companionRxPosition = 0;
uint8_t companionExpectedFrameLength = 0;
bool companionFrameActive = false;

// -----------------------------------------------------------------------------
// Utility functions
// -----------------------------------------------------------------------------

static int16_t clampMille(int32_t value)
{
    if (value > THROTTLE_MAX)
    {
        return THROTTLE_MAX;
    }

    if (value < THROTTLE_MIN)
    {
        return THROTTLE_MIN;
    }

    return static_cast<int16_t>(value);
}

/**
 * CRC-8 used by the ROS 2 companion protocol.
 *
 * Polynomial: 0x07
 * Initial value: 0x00
 */
static uint8_t companionCrc8(
    const uint8_t *data,
    size_t length)
{
    uint8_t crc = 0;

    for (size_t index = 0; index < length; ++index)
    {
        crc ^= data[index];

        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = static_cast<uint8_t>((crc << 1U) ^ 0x07U);
            }
            else
            {
                crc = static_cast<uint8_t>(crc << 1U);
            }
        }
    }

    return crc;
}

// -----------------------------------------------------------------------------
// Companion-computer binary transport
// -----------------------------------------------------------------------------

static void sendCompanionFrame(
    uint16_t messageId,
    const uint8_t *payload,
    uint8_t payloadLength)
{
    if (payloadLength > COMPANION_MAX_PAYLOAD)
    {
        return;
    }

    uint8_t frame[COMPANION_MAX_FRAME_SIZE];

    frame[0] = COMPANION_SOF;
    frame[1] = static_cast<uint8_t>(messageId & 0xFFU);
    frame[2] = static_cast<uint8_t>((messageId >> 8U) & 0xFFU);
    frame[3] = payloadLength;

    for (uint8_t index = 0; index < payloadLength; ++index)
    {
        frame[4 + index] = payload[index];
    }

    const uint8_t frameWithoutCrcLength =
        static_cast<uint8_t>(4U + payloadLength);

    frame[frameWithoutCrcLength] =
        companionCrc8(frame, frameWithoutCrcLength);

    Serial.write(
        frame,
        static_cast<size_t>(frameWithoutCrcLength + 1U));
}

// -----------------------------------------------------------------------------
// CAN transmit functions
// -----------------------------------------------------------------------------

static void canSend(
    uint16_t id,
    const uint8_t *buffer,
    uint8_t length)
{
    CAN_message_t message;

    message.id = id;
    message.len = length;

    for (uint8_t index = 0;
         index < length && index < 8;
         ++index)
    {
        message.buf[index] = buffer[index];
    }

    const bool sent = Can1.write(message);

    if (!sent)
    {
        Serial.println(F("CAN WRITE FAILED"));
    }
}

static void sendEstopFrame()
{
    uint8_t buffer[8];

    const uint8_t length =
        pack_estop(buffer, ESTOP_OPERATOR);

    canSend(CANID_ESTOP, buffer, length);
}

static void sendHeartbeat(bool teleopOk)
{
    Heartbeat heartbeat;

    heartbeat.counter = hbCounter++;
    heartbeat.flags = HB_FLAG_OK;

    if (teleopOk)
    {
        heartbeat.flags |= HB_FLAG_TELEOP_OK;
    }

    if (estopLatched)
    {
        heartbeat.flags |= HB_FLAG_ESTOP;
    }

    if (estopLatched)
    {
        heartbeat.mode = MODE_ESTOP;
    }
    else if (
        companionThrottleActive ||
        teleopVx != 0 ||
        teleopWz != 0 ||
        testIdx >= 0)
    {
        heartbeat.mode = MODE_TELEOP;
    }
    else
    {
        heartbeat.mode = MODE_IDLE;
    }

    uint8_t buffer[8];

    const uint8_t length =
        pack_heartbeat(buffer, heartbeat);

    // Heartbeat to embedded CAN nodes.
    canSend(CANID_HEARTBEAT, buffer, length);

    // Same heartbeat to the Jetson/ROS 2 companion computer.
    sendCompanionFrame(
        CANID_HEARTBEAT,
        buffer,
        length);
}

static void sendThrottleCmd()
{
    ThrottleCmd command = {0, 0, 0, 0};

    if (!estopLatched)
    {
        if (companionThrottleActive)
        {
            command = companionThrottleCmd;
        }
        else if (testIdx >= 0 && testIdx < 4)
        {
            int16_t values[4] = {0, 0, 0, 0};

            values[testIdx] = clampMille(testVal);

            command.fl = values[0];
            command.fr = values[1];
            command.rl = values[2];
            command.rr = values[3];
        }
        else
        {
            const int16_t left =
                clampMille(
                    static_cast<int32_t>(teleopVx) +
                    teleopWz);

            const int16_t right =
                clampMille(
                    static_cast<int32_t>(teleopVx) -
                    teleopWz);

            command.fl = left;
            command.rl = left;
            command.fr = right;
            command.rr = right;
        }
    }

    uint8_t buffer[8];

    const uint8_t length =
        pack_throttle_cmd(buffer, command);

    canSend(
        CANID_THROTTLE_CMD,
        buffer,
        length);

    // Retained for bench debugging.
    Serial.print(F("TX CMD: "));
    Serial.print(command.fl);
    Serial.print(' ');
    Serial.print(command.fr);
    Serial.print(' ');
    Serial.print(command.rl);
    Serial.print(' ');
    Serial.println(command.rr);
}

// -----------------------------------------------------------------------------
// ROS 2 companion-command handling
// -----------------------------------------------------------------------------

static void handleCompanionFrame(
    uint16_t messageId,
    const uint8_t *payload,
    uint8_t payloadLength)
{
    switch (messageId)
    {
        case CANID_THROTTLE_CMD:
        {
            ThrottleCmd receivedCommand;

            if (unpack_throttle_cmd(
                    payload,
                    payloadLength,
                    &receivedCommand))
            {
                companionThrottleCmd.fl =
                    clampMille(receivedCommand.fl);

                companionThrottleCmd.fr =
                    clampMille(receivedCommand.fr);

                companionThrottleCmd.rl =
                    clampMille(receivedCommand.rl);

                companionThrottleCmd.rr =
                    clampMille(receivedCommand.rr);

                companionThrottleActive = true;

                // ROS direct command overrides text teleop and
                // individual-wheel test mode.
                teleopVx = 0;
                teleopWz = 0;
                testIdx = -1;
                testVal = 0;

                lastTeleopMs = millis();
            }

            break;
        }

        case CANID_ESTOP:
        {
            if (payloadLength < 1)
            {
                break;
            }

            const uint8_t reason = payload[0];

            if (reason == 0)
            {
                // ROS reset request.
                estopLatched = false;
            }
            else
            {
                // Any nonzero reason is treated as an E-stop event.
                estopLatched = true;

                companionThrottleActive = false;
                companionThrottleCmd = {0, 0, 0, 0};

                teleopVx = 0;
                teleopWz = 0;
                testIdx = -1;
                testVal = 0;

                sendEstopFrame();
            }

            lastTeleopMs = millis();
            break;
        }

        default:
            break;
    }
}

// -----------------------------------------------------------------------------
// Existing text-command handling
// -----------------------------------------------------------------------------

static void parseLine(char *line)
{
    if (line[0] == 'D' || line[0] == 'd')
    {
        int vx;
        int wz;

        if (sscanf(line + 1, "%d %d", &vx, &wz) == 2)
        {
            teleopVx = clampMille(vx);
            teleopWz = clampMille(wz);

            companionThrottleActive = false;
            companionThrottleCmd = {0, 0, 0, 0};

            testIdx = -1;
            lastTeleopMs = millis();
        }
    }
    else if (line[0] == 'M' || line[0] == 'm')
    {
        int index;
        int value;

        if (sscanf(
                line + 1,
                "%d %d",
                &index,
                &value) == 2)
        {
            testIdx = index;
            testVal = clampMille(value);

            companionThrottleActive = false;
            companionThrottleCmd = {0, 0, 0, 0};

            teleopVx = 0;
            teleopWz = 0;

            lastTeleopMs = millis();
        }
    }
    else if (
        line[0] == 'S' ||
        line[0] == 's' ||
        line[0] == 'X' ||
        line[0] == 'x')
    {
        teleopVx = 0;
        teleopWz = 0;

        companionThrottleActive = false;
        companionThrottleCmd = {0, 0, 0, 0};

        testIdx = -1;
        testVal = 0;

        lastTeleopMs = millis();
    }
    else if (line[0] == 'E' || line[0] == 'e')
    {
        estopLatched = true;

        teleopVx = 0;
        teleopWz = 0;

        companionThrottleActive = false;
        companionThrottleCmd = {0, 0, 0, 0};

        testIdx = -1;
        testVal = 0;

        sendEstopFrame();
        lastTeleopMs = millis();
    }
    else if (line[0] == 'R' || line[0] == 'r')
    {
        estopLatched = false;
        lastTeleopMs = millis();
    }
}

static void processTextSerialByte(uint8_t byteValue)
{
    const char character =
        static_cast<char>(byteValue);

    if (character == '\n' || character == '\r')
    {
        if (lineLen > 0)
        {
            lineBuf[lineLen] = '\0';
            parseLine(lineBuf);
            lineLen = 0;
        }

        return;
    }

    if (lineLen < sizeof(lineBuf) - 1)
    {
        lineBuf[lineLen++] = character;
    }
    else
    {
        lineLen = 0;
    }
}

// -----------------------------------------------------------------------------
// Combined text and binary serial receiver
// -----------------------------------------------------------------------------

static void resetCompanionReceiver()
{
    companionRxPosition = 0;
    companionExpectedFrameLength = 0;
    companionFrameActive = false;
}

static void processCompanionSerialByte(uint8_t byteValue)
{
    if (!companionFrameActive)
    {
        if (byteValue == COMPANION_SOF)
        {
            companionFrameActive = true;
            companionRxPosition = 0;
            companionExpectedFrameLength = 0;

            companionRxFrame[companionRxPosition++] =
                byteValue;
        }
        else
        {
            // Preserve the original ASCII command interface.
            processTextSerialByte(byteValue);
        }

        return;
    }

    if (companionRxPosition >= COMPANION_MAX_FRAME_SIZE)
    {
        resetCompanionReceiver();
        return;
    }

    companionRxFrame[companionRxPosition++] =
        byteValue;

    // Once SOF, ID-low, ID-high, and LEN are present,
    // calculate the complete expected frame length.
    if (companionRxPosition == 4)
    {
        const uint8_t payloadLength =
            companionRxFrame[3];

        if (payloadLength > COMPANION_MAX_PAYLOAD)
        {
            resetCompanionReceiver();
            return;
        }

        companionExpectedFrameLength =
            static_cast<uint8_t>(
                4U +
                payloadLength +
                1U);
    }

    if (
        companionExpectedFrameLength == 0 ||
        companionRxPosition <
            companionExpectedFrameLength)
    {
        return;
    }

    const uint8_t receivedCrc =
        companionRxFrame[
            companionExpectedFrameLength - 1U];

    const uint8_t calculatedCrc =
        companionCrc8(
            companionRxFrame,
            companionExpectedFrameLength - 1U);

    if (receivedCrc == calculatedCrc)
    {
        const uint16_t messageId =
            static_cast<uint16_t>(
                companionRxFrame[1]) |
            static_cast<uint16_t>(
                companionRxFrame[2] << 8U);

        const uint8_t payloadLength =
            companionRxFrame[3];

        handleCompanionFrame(
            messageId,
            &companionRxFrame[4],
            payloadLength);
    }

    resetCompanionReceiver();
}

static void pollSerial()
{
    while (Serial.available() > 0)
    {
        const int received = Serial.read();

        if (received >= 0)
        {
            processCompanionSerialByte(
                static_cast<uint8_t>(received));
        }
    }
}

// -----------------------------------------------------------------------------
// CAN receive and forwarding
// -----------------------------------------------------------------------------

static void pollCan()
{
    CAN_message_t receivedMessage;

    while (Can1.read(receivedMessage))
    {
        switch (receivedMessage.id)
        {
            case CANID_THROTTLE_STATUS:
            {
                if (unpack_throttle_status(
                        receivedMessage.buf,
                        receivedMessage.len,
                        &lastStatus))
                {
                    haveStatus = true;

                    // Forward the original six-byte CAN payload
                    // directly to the ROS 2 bridge.
                    sendCompanionFrame(
                        CANID_THROTTLE_STATUS,
                        receivedMessage.buf,
                        receivedMessage.len);
                }

                break;
            }

case CANID_ENCODER_FB:
{
    if (unpack_encoder_fb(
            receivedMessage.buf,
            receivedMessage.len,
            &lastEnc))
    {
        haveEncoderFeedback = true;

        // Forward the validated eight-byte encoder payload
        // to the Jetson using the existing companion protocol.
        sendCompanionFrame(
            CANID_ENCODER_FB,
            receivedMessage.buf,
            receivedMessage.len);
    }

    break;
}

            case CANID_ESTOP:
            {
                estopLatched = true;

                companionThrottleActive = false;
                companionThrottleCmd = {0, 0, 0, 0};

                break;
            }

            default:
                break;
        }
    }
}

// -----------------------------------------------------------------------------
// Human-readable diagnostic telemetry
// -----------------------------------------------------------------------------

static void streamTelemetry(bool teleopOk)
{
    Serial.print(F("TLM hb="));
    Serial.print(hbCounter);

    Serial.print(F(" mode="));

    if (estopLatched)
    {
        Serial.print(F("ESTOP"));
    }
    else if (companionThrottleActive)
    {
        Serial.print(F("ROS"));
    }
    else if (testIdx >= 0)
    {
        Serial.print(F("TEST"));
    }
    else if (teleopVx != 0 || teleopWz != 0)
    {
        Serial.print(F("TELEOP"));
    }
    else
    {
        Serial.print(F("IDLE"));
    }

    Serial.print(F(" test="));
    Serial.print(testIdx);

    Serial.print(F(" link="));
    Serial.print(teleopOk ? 1 : 0);

    Serial.print(F(" vx="));
    Serial.print(teleopVx);

    Serial.print(F(" wz="));
    Serial.print(teleopWz);

    Serial.print(F(" tstate="));
    Serial.print(
        haveStatus ?
        lastStatus.state :
        255);

    Serial.print(F(" tflt="));
    Serial.print(
        haveStatus ?
        lastStatus.faults :
        0);

    Serial.print(F(" L="));
    Serial.print(
        haveStatus ?
        lastStatus.applied_left :
        0);

    Serial.print(F(" R="));
    Serial.print(
        haveStatus ?
        lastStatus.applied_right :
        0);

    Serial.print(F(" encValid="));
    Serial.print(haveEncoderFeedback ? 1 : 0);

    Serial.print(F(" encL="));
    Serial.print(lastEnc.left_count);

    Serial.print(F(" encR="));
    Serial.println(lastEnc.right_count);
}

// -----------------------------------------------------------------------------
// IMU support
// -----------------------------------------------------------------------------

static bool i2cDevicePresent(uint8_t address)
{
    Wire.beginTransmission(address);

    return Wire.endTransmission() == 0;
}

static bool readI2cRegister(
    uint8_t address,
    uint8_t registerAddress,
    uint8_t &value)
{
    Wire.beginTransmission(address);
    Wire.write(registerAddress);

    if (Wire.endTransmission(false) != 0)
    {
        return false;
    }

    const uint8_t received =
        Wire.requestFrom(
            address,
            static_cast<uint8_t>(1));

    if (received != 1 || !Wire.available())
    {
        return false;
    }

    value = Wire.read();
    return true;
}

static void detectImu()
{
    imuDetected = false;
    imuAddress = 0;
    imuWhoAmI = 0;

    const uint8_t addresses[] =
    {
        MPU_ADDRESS_LOW,
        MPU_ADDRESS_HIGH
    };

    for (uint8_t address : addresses)
    {
        if (!i2cDevicePresent(address))
        {
            continue;
        }

        uint8_t whoAmI = 0;

        if (readI2cRegister(
                address,
                MPU_WHO_AM_I_REGISTER,
                whoAmI))
        {
            imuDetected = true;
            imuAddress = address;
            imuWhoAmI = whoAmI;
            break;
        }
    }

    if (imuDetected)
    {
        Serial.print(F("IMU detected address=0x"));

        if (imuAddress < 0x10)
        {
            Serial.print('0');
        }

        Serial.print(imuAddress, HEX);
        Serial.print(F(" WHO_AM_I=0x"));

        if (imuWhoAmI < 0x10)
        {
            Serial.print('0');
        }

        Serial.println(imuWhoAmI, HEX);
    }
    else
    {
        Serial.println(
            F("IMU not detected at 0x68 or 0x69"));
    }
}

static void streamImuTelemetry()
{
    Serial.print(F("IMU"));

    Serial.print(F(",ok="));
    Serial.print(imuReady ? 1 : 0);

    Serial.print(F(",ax_mps2="));
    Serial.print(latestImuData.accelXMps2, 4);

    Serial.print(F(",ay_mps2="));
    Serial.print(latestImuData.accelYMps2, 4);

    Serial.print(F(",az_mps2="));
    Serial.print(latestImuData.accelZMps2, 4);

    Serial.print(F(",gx_dps="));
    Serial.print(latestImuData.gyroXDps, 4);

    Serial.print(F(",gy_dps="));
    Serial.print(latestImuData.gyroYDps, 4);

    Serial.print(F(",gz_dps="));
    Serial.print(latestImuData.gyroZDps, 4);

    Serial.print(F(",temp_c="));
    Serial.print(latestImuData.temperatureC, 2);

    Serial.print(F(",errors="));
    Serial.println(imuReadErrorCount);
}

// -----------------------------------------------------------------------------
// Arduino setup
// -----------------------------------------------------------------------------

void setup()
{
    // Must match carl_bridge/config/bridge.yaml.
    Serial.begin(COMPANION_BAUD);
    delay(1000);

    imuReady = imu.begin(
        Wire,
        PB9,
        PB8,
        100000);

    if (imuReady)
    {
        Serial.print(
            F("MPU-6500 online address=0x"));

        Serial.print(
            imu.getAddress(),
            HEX);

        Serial.print(F(" WHO_AM_I=0x"));

        Serial.println(
            imu.getWhoAmI(),
            HEX);
    }
    else
    {
        Serial.println(
            F("ERROR: MPU-6500 initialization failed"));
    }

    Can1.begin();
    Can1.setBaudRate(CAN_BITRATE);

    lastTeleopMs = millis();

    Serial.println(
        F("CARL master/VCU merged restore online"));
}

// -----------------------------------------------------------------------------
// Arduino main loop
// -----------------------------------------------------------------------------

void loop()
{
    const uint32_t now = millis();

    pollSerial();
    pollCan();

    const bool teleopOk =
        (now - lastTeleopMs) <=
        TELEOP_TIMEOUT_MS;

    if (!teleopOk)
    {
        teleopVx = 0;
        teleopWz = 0;

        companionThrottleActive = false;
        companionThrottleCmd = {0, 0, 0, 0};

        testIdx = -1;
        testVal = 0;
    }

    if (now - lastCmdMs >=
        THROTTLE_CMD_PERIOD_MS)
    {
        lastCmdMs = now;
        sendThrottleCmd();
    }

    if (now - lastHbMs >=
        HEARTBEAT_PERIOD_MS)
    {
        lastHbMs = now;
        sendHeartbeat(teleopOk);
    }

    if (now - lastTlmMs >= 100)
    {
        lastTlmMs = now;
        streamTelemetry(teleopOk);
    }

    if (now - lastImuMs >= 100)
    {
        lastImuMs = now;

        if (imuReady)
        {
            if (imu.read(latestImuData))
            {
                streamImuTelemetry();
            }
            else
            {
                imuReady = false;
                ++imuReadErrorCount;

                Serial.println(
                    F("ERROR: MPU-6500 read failed"));
            }
        }
    }
}
