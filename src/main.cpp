/*
 * ESP32 IMU-Based motion controller
 *
 * Developed by MISFIT STUDIO (https://misfit.ch)
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <Bounce2.h>

#include "types.h"
#include "config.h"

#include "utils.h"
#include "quat_math.h"

#define BTN_DEBOUNCE_TIME 50
#define DATA_REPEAT_INTERVAL 500
#define MOVEMENT_TIMEOUT 5 * 60 * 1000 // 5 minutes

/* -------------------------- IO GLOBALS -------------------------- */

Bounce btn_a = Bounce();
Bounce btn_b = Bounce();

HardwareSerial IMUSerialPort(2);   // use UART2
HardwareSerial RS485SerialPort(1); // use UART1

Adafruit_BNO08x bno08x(BNO08X_RESET);
// Struct to store bn008x sensor values
sh2_SensorValue_t sensorValue;

// This is the stored "zero" quaternion for calibration
sQuatRotationVector_t zero_quaternion = {0.0, 0.0, 0.0, 1.0};
sQuatRotationVector_t last_orientation = {0.0, 0.0, 0.0, 1.0};

float last_yaw_mapped = 0.5;
float last_pitch_mapped = 0.5;

eStabilityStatus_t last_stability_status = STABILITY_IN_MOTION;
eCalibrationStatus_t last_calibration_status = UNCALIBRATED;

/* -------------------------- STATE GLOBALS -------------------------- */

bool btn_a_state = false;
bool btn_b_state = false;

uint16_t last_crc = 0;
unsigned long last_data_repeat_time = 0;

// Timer for debug readouts
unsigned long last_ypr_log_time = 0;

// Timer for serial receive window
unsigned long serial_rx_window_time = 0;
int rx_counter = 0;
byte rx_buffer[MESSAGE_SIZE];

// Fuckery reset timeout
unsigned long last_movement_time = 0;

/* -------------------------- SETUP FUNCTIONS -------------------------- */

void bno08x_set_reports()
{
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_GRV, 5000U))
  {
    Serial.println("Could not enable stabilized remote vector");
  }
  if (!bno08x.enableReport(SH2_SHAKE_DETECTOR))
  {
    Serial.println("Could not enable shake detector");
  }
  if (!bno08x.enableReport(SH2_STABILITY_CLASSIFIER))
  {
    Serial.println("Could not enable stability classifier");
  }
}

void gpio_setup()
{
  pinMode(RS485_DE_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  btn_a.attach(BTNA_PIN, INPUT_PULLUP);
  btn_b.attach(BTNB_PIN, INPUT_PULLUP);

  btn_a.interval(BTN_DEBOUNCE_TIME);
  btn_b.interval(BTN_DEBOUNCE_TIME);
}

void bno08x_setup()
{
  if (!bno08x.begin_UART(&IMUSerialPort))
  {
    Serial.println("Failed to find BNO08x chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("BNO08x Found!");

  bno08x_set_reports();
}

void setup()
{
  Serial.begin(115200);
  IMUSerialPort.begin(115200, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
  RS485SerialPort.begin(57600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  Serial.println("BNO08x IMU Remote");
  Serial.println("Initializing...");

  gpio_setup();
  bno08x_setup();

  Serial.println("Ready!");
  delay(100);
}

/* -------------------------- SERIAL -------------------------- */

void send_message(eMessageType_t type, uint8_t *data, uint8_t data_size = 12)
{
  message_t message;
  message.type = type;
  memset(message.data, 0, 12);
  if (data != nullptr)
    memcpy(message.data, data, data_size);

  message.crc = calc_crc((uint8_t *)&message, MESSAGE_SIZE - 2);

  // Throttle the sending of repeated data messages
  if (message.type == DEVICE_DATA)
  {
    if (message.crc == last_crc)
    {
      if (millis() - last_data_repeat_time < DATA_REPEAT_INTERVAL)
      {
        return;
      }
      else
      {
        last_data_repeat_time = millis();
      }
    }
    last_crc = message.crc;
  }

  // Write mexssage to the RS485 bus
  digitalWrite(RS485_DE_PIN, HIGH);
  RS485SerialPort.write((uint8_t *)&message, MESSAGE_SIZE);
  RS485SerialPort.flush();
  digitalWrite(RS485_DE_PIN, LOW);

  // Write message to the serial console
  // serial_print_hex((uint8_t *)&message, MESSAGE_SIZE);
}

void send_data()
{
  sDeviceData_t device_data;
  device_data.yaw = last_yaw_mapped;
  device_data.pitch = last_pitch_mapped;
  device_data.stability_status = last_stability_status;
  device_data.calibration_status = last_calibration_status;
  device_data.btn_a_state = btn_a_state;
  device_data.btn_b_state = btn_b_state;

  send_message(DEVICE_DATA, (uint8_t *)&device_data, DATA_SIZE);
}

void send_ack()
{
  send_message(ACK, nullptr, 0);
}

/* -------------------------- MAIN LOOP -------------------------- */

void handle_ypr_event()
{
  last_orientation = sensorValue.un.arvrStabilizedGRV;
  sQuatRotationVector_t delta = quat_delta(last_orientation, zero_quaternion);
  sEulerRotationVector_t delta_euler = quat_to_euler(&delta, true);

  last_yaw_mapped = fmod(-delta_euler.yaw + 180.0, 360.0) / 360;
  last_pitch_mapped = fmod(delta_euler.pitch + 180.0, 360.0) / 360;

  if (millis() - last_ypr_log_time > 500)
  {
    last_ypr_log_time = millis();
    sEulerRotationVector_t orientation_euler = quat_to_euler(&last_orientation, true);
    Serial.printf("Yaw: %f, Pitch: %f | RYaw: %f, RPitch: %f, RRoll: %f\n", last_yaw_mapped, last_pitch_mapped, orientation_euler.yaw, orientation_euler.pitch, orientation_euler.roll);
  }
}

void handle_stability_event()
{
  sh2_StabilityClassifier_t stability_class = sensorValue.un.stabilityClassifier;

  eStabilityStatus_t stability_status = STABILITY_UNKNOWN;
  switch (stability_class.classification)
  {
  case STABILITY_CLASSIFIER_STABLE:
  {
    sEulerRotationVector_t orientation_euler = quat_to_euler(&last_orientation, true);
    bool is_facing_down = orientation_euler.pitch > 45 || abs(orientation_euler.roll) > 120;

    stability_status = is_facing_down ? STABILITY_HANGING : STABILITY_STABLE;
    break;
  }
  case STABILITY_CLASSIFIER_STATIONARY:
  case STABILITY_CLASSIFIER_ON_TABLE:
    stability_status = STABILITY_STABLE;
    break;
  case STABILITY_CLASSIFIER_MOTION:
    stability_status = STABILITY_IN_MOTION;
    break;
  default:
    stability_status = STABILITY_UNKNOWN;
    break;
  }

  if (stability_status != last_stability_status)
  {
    Serial.printf("Stability changed: %s -> %s\n", STABILITY_STATUS_NAMES[last_stability_status], STABILITY_STATUS_NAMES[stability_status]);
    last_stability_status = stability_status;
    last_movement_time = millis();
  }
}

void poll_buttons()
{
  btn_a.update();
  if (btn_a.changed())
  {
    btn_a_state = !btn_a.read();
    Serial.printf("Button A changed: %s\n", btn_a_state ? "PRESSED" : "RELEASED");
  }

  btn_b.update();
  if (btn_b.changed())
  {
    btn_b_state = !btn_b.read();
    Serial.printf("Button B changed: %s\n", btn_b_state ? "PRESSED" : "RELEASED");
  }
}

void calibrate()
{
  Serial.println("Calibrating...");
  zero_quaternion = last_orientation;
  last_calibration_status = CALIBRATED;
}

void handle_message_received(sMessage_t *message)
{
  Serial.printf("Received message: %04x\n", message->type);
  switch (message->type)
  {
  case CMD_CALIBRATE:
    calibrate();
    break;
  }
  send_ack();
}

void loop()
{
  if (bno08x.wasReset())
  {
    Serial.print("Sensor was reset");
    bno08x_set_reports();
  }

  if (bno08x.getSensorEvent(&sensorValue))
  {
    switch (sensorValue.sensorId)
    {
    case SH2_ARVR_STABILIZED_GRV:
      handle_ypr_event();
      break;
    case SH2_STABILITY_CLASSIFIER:
      handle_stability_event();
      break;
    }
  }

  poll_buttons();

  if (RS485SerialPort.available())
  {
    serial_rx_window_time = millis();
    while (RS485SerialPort.available())
    {
      byte b = RS485SerialPort.read();
      if (rx_counter == 0 && b != 0x66)
      {
        continue;
      }
      else
      {
        rx_buffer[rx_counter] = b;
        rx_counter++;
        if (rx_counter == MESSAGE_SIZE)
        {
          rx_counter = 0;
          Serial.println("Received data:");
          serial_print_hex(rx_buffer, MESSAGE_SIZE);
          message_t message;
          message.start = (rx_buffer[1] << 8) | rx_buffer[0];
          message.type = (rx_buffer[3] << 8) | rx_buffer[2];
          memcpy(message.data, rx_buffer + 4, 12);
          message.crc = (rx_buffer[17] << 8) | rx_buffer[16];

          if (message.start == 0x5566)
          {
            uint16_t crc = calc_crc(rx_buffer, MESSAGE_SIZE - 2);
            if (crc == message.crc)
            {
              handle_message_received(&message);
            }
          }
        }
      }
    }
  }

  if (millis() - serial_rx_window_time > 250)
  {
    send_data();
  }

  if (millis() - last_movement_time > MOVEMENT_TIMEOUT && last_stability_status != STABILITY_IN_MOTION)
  {
    // Reboot the device if it hasn't moved in a while
    Serial.println("Device has not moved in a while, rebooting...");
    ESP.restart();
  }
}
