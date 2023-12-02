// #include <Arduino.h>
// #include <HardwareSerial.h>
// #include <Adafruit_BNO08x.h>

// #define BNO08X_RESET -1

// #define IMU_RX_PIN 18
// #define IMU_TX_PIN 19

// #define RS485_RX_PIN 21
// #define RS485_TX_PIN 22
// #define RS485_DE_PIN 23

// #define BTNA_PIN 16
// #define BTNB_PIN 17
// #define LED_PIN 25

// typedef enum e_command_type_t
// {
//     CMD_NONE = 0,
//     CMD_YPR_DATA,
//     CMD_BUTTONS_CHANGED,
//     CMD_STABILITY_CHANGED,
//     CMD_WAS_CALIBRATED,
// } command_type_t;

// typedef enum e_stability_class_t
// {
//     STABILITY_UNKNOWN = STABILITY_CLASSIFIER_UNKNOWN,
//     STABILITY_ON_TABLE = STABILITY_CLASSIFIER_ON_TABLE,
//     STABILITY_STATIONARY = STABILITY_CLASSIFIER_STATIONARY,
//     STABILITY_STABLE = STABILITY_CLASSIFIER_STABLE,
//     STABILITY_MOTION = STABILITY_CLASSIFIER_MOTION,
// } stability_class_t;

// typedef struct s_euler_t
// {
//     float yaw;
//     float pitch;
//     float roll;
// } euler_t;

// typedef struct s_button_state_t
// {
//     uint8_t btnA;
//     uint8_t btnB;
// } button_state_t;

// typedef struct s_message_t
// {
//     uint16_t start = 0x5566;
//     uint16_t command;
//     uint8_t data[12];
//     uint16_t crc;
// } message_t;

// #define MESSAGE_SIZE 18

// static unsigned long btna_press_time = 0;

// s_euler_t ypr = {0, 0, 0};
// s_euler_t ypr_zero = {0, 0, 0};
// bool ypr_zero_set = false;

// uint8_t last_stability_class = STABILITY_UNKNOWN;
// s_button_state_t last_button_state = {0, 0};

// float lastVal = 0;

// Adafruit_BNO08x bno08x(BNO08X_RESET);
// sh2_SensorValue_t sensorValue;

// HardwareSerial IMUSerialPort(2);   // use UART2
// HardwareSerial RS485SerialPort(1); // use UART1

// void print_hex(uint8_t *data, uint8_t size)
// {
//     for (int i = 0; i < size; i++)
//     {
//         Serial.printf("%02x", data[i]);
//         Serial.print(" ");
//     }
//     Serial.println();
// }

// #define CRC_POLY 0x1021;
// static uint16_t calc_crc(uint8_t *data, uint8_t size)
// {
//     // print_hex(data, size);

//     int i, j;
//     uint16_t crc = 0;
//     for (i = 0; i < size; i++)
//     {
//         unsigned int xr = data[i] << 8;
//         crc = crc ^ xr;

//         for (j = 0; j < 8; j++)
//         {
//             if (crc & 0x8000)
//             {
//                 crc = (crc << 1);
//                 crc = crc ^ CRC_POLY;
//             }
//             else
//             {
//                 crc = crc << 1;
//             }
//         }
//     }
//     crc = crc & 0xFFFF;
//     return crc;
// }

// void bno08x_set_reports()
// {
//     Serial.println("Setting desired reports");
//     if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000U))
//     {
//         Serial.println("Could not enable stabilized remote vector");
//     }
//     if (!bno08x.enableReport(SH2_SHAKE_DETECTOR))
//     {
//         Serial.println("Could not enable shake detector");
//     }
//     if (!bno08x.enableReport(SH2_STABILITY_CLASSIFIER))
//     {
//         Serial.println("Could not enable stability classifier");
//     }
// }

// void quat_to_euler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees = false)
// {

//     float sqr = sq(qr);
//     float sqi = sq(qi);
//     float sqj = sq(qj);
//     float sqk = sq(qk);

//     ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
//     ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
//     ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

//     if (degrees)
//     {
//         ypr->yaw *= RAD_TO_DEG;
//         ypr->pitch *= RAD_TO_DEG;
//         ypr->roll *= RAD_TO_DEG;
//     }
// }

// void quat_to_euler_rv(sh2_RotationVectorWAcc_t *rotational_vector, euler_t *ypr, bool degrees = false)
// {
//     quat_to_euler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
// }

// void setup(void)
// {

//     Serial.begin(115200);
//     IMUSerialPort.begin(115200, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
//     RS485SerialPort.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

//     Serial.println("BNO08x IMU Remote");
//     Serial.println("Initializing...");

//     pinMode(RS485_DE_PIN, OUTPUT);
//     pinMode(LED_PIN, OUTPUT);

//     pinMode(BTNA_PIN, INPUT_PULLUP);
//     pinMode(BTNB_PIN, INPUT_PULLUP);

//     if (!bno08x.begin_UART(&IMUSerialPort))
//     {
//         Serial.println("Failed to find BNO08x chip");
//         while (1)
//         {
//             delay(10);
//         }
//     }
//     Serial.println("BNO08x Found!");

//     bno08x_set_reports();

//     Serial.println("Ready!");
//     delay(100);
// }

// void send_message(command_type_t command, uint8_t *data, uint8_t data_size = 12)
// {
//     message_t message;
//     message.command = command;
//     memset(message.data, 0, 12);
//     if (data != nullptr)
//     {
//         memcpy(message.data, data, data_size);
//     }

//     message.crc = calc_crc((uint8_t *)&message, MESSAGE_SIZE - 2);

//     digitalWrite(RS485_DE_PIN, HIGH);
//     RS485SerialPort.write((uint8_t *)&message, MESSAGE_SIZE);
//     RS485SerialPort.flush();

//     // Serial.write((uint8_t *)&message, MESSAGE_SIZE);
//     // print_hex((uint8_t *)&message, sizeof(message));

//     digitalWrite(RS485_DE_PIN, LOW);
// }

// void handle_ypr_changed()
// {
//     if (!ypr_zero_set || digitalRead(BTNB_PIN) == LOW)
//     {
//         ypr_zero = ypr;
//         ypr_zero_set = true;
//         send_message(CMD_WAS_CALIBRATED, nullptr);
//     }

//     // ypr.yaw -= ypr_zero.yaw;
//     // ypr.pitch -= ypr_zero.pitch;
//     // ypr.roll -= ypr_zero.roll;

//     send_message(CMD_YPR_DATA, (uint8_t *)&ypr, sizeof(ypr));
// }

// void loop()
// {
//     if (bno08x.wasReset())
//     {
//         Serial.print("Sensor was reset");
//         bno08x_set_reports();
//     }

//     if (bno08x.getSensorEvent(&sensorValue))
//     {
//         switch (sensorValue.sensorId)
//         {
//         case SH2_ARVR_STABILIZED_RV:
//         {
//             // The Orientation has changed
//             quat_to_euler_rv(&sensorValue.un.arvrStabilizedRV, &ypr, true);
//             handle_ypr_changed();
//             break;
//         }

//         case SH2_STABILITY_CLASSIFIER:
//         {
//             sh2_StabilityClassifier_t stability = sensorValue.un.stabilityClassifier;
//             if (last_stability_class != stability.classification)
//             {
//                 last_stability_class = stability.classification;
//                 send_message(CMD_STABILITY_CHANGED, (uint8_t *)&last_stability_class, sizeof(last_stability_class));
//             }
//             break;
//         }
//         }
//     }

//     s_button_state_t button_state = {!digitalRead(BTNA_PIN), !digitalRead(BTNB_PIN)};
//     if (button_state.btnA != last_button_state.btnA || button_state.btnB != last_button_state.btnB)
//     {
//         last_button_state = button_state;
//         send_message(CMD_BUTTONS_CHANGED, (uint8_t *)&last_button_state, sizeof(last_button_state));
//     }

//     // Read data from RS485
//     if (RS485SerialPort.available())
//     {
//         int rs485Data = RS485SerialPort.read();
//         Serial.println("Received RS485 data: " + String(rs485Data));
//     }

//     // delay(10);
// }
