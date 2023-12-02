#include <Arduino.h>

typedef enum eStabilityStatus_t : uint8_t
{
    STABILITY_UNKNOWN = 0,
    STABILITY_HANGING = 1,
    STABILITY_STABLE = 2,
    STABILITY_IN_MOTION = 3,
} stabilityStatus_t;

const char *STABILITY_STATUS_NAMES[] = {
    "UNKNOWN",
    "HANGING",
    "STABLE",
    "IN_MOTION",
};

typedef enum eCalibrationStatus_t : uint8_t
{
    UNCALIBRATED = 0,
    CALIBRATED = 1,
} calibrationStatus_t;

typedef enum eMessageType_t : uint16_t
{
    // Protocol level messages
    NONE = 0x0000,
    READY = 0x0001,
    ACK = 0x0002,
    // Data messages
    DEVICE_DATA = 0xAA01,
    // Command messages
    CMD_RESET = 0xFF01,
    CMD_CALIBRATE = 0xFF02,
    CMD_SET_LED = 0xFF03,
} messageType_t;

typedef struct sDeviceData_t
{
    float yaw;
    float pitch;
    eStabilityStatus_t stability_status;
    eCalibrationStatus_t calibration_status;
    uint8_t btn_a_state;
    uint8_t btn_b_state;
} deviceData_t;

#define DATA_SIZE sizeof(float) + sizeof(float) + sizeof(eStabilityStatus_t) + sizeof(eCalibrationStatus_t) + sizeof(uint8_t) + sizeof(uint8_t)

typedef struct sMessage_t
{
    uint16_t start = 0x5566;
    uint16_t type;
    uint8_t data[12];
    uint16_t crc;
} message_t;

#define MESSAGE_SIZE 18