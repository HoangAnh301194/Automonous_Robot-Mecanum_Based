#include <Arduino.h>
#include <HardwareSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

HardwareSerial HoverSerial(2);

#define START_FRAME         0xABCD
#define TICK_PER_METER      173.91f
#define WHEEL_BASE          0.58f
#define K_SPEED             114.28f

// ============ C?u trúc gói ============
typedef struct {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
} __attribute__((packed)) SerialCommand;

typedef struct {
  uint16_t start;
  int16_t  ticksL;
  int16_t  ticksR;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} __attribute__((packed)) SerialFeedback;

SerialCommand   Command;
SerialFeedback  Feedback, NewFeedback;
byte *p;
uint8_t idx = 0;
byte incomingByte, incomingBytePrev;

// ============ Odometry ============
long    absolute_ticks_L = 0;
long    absolute_ticks_R = 0;
int16_t last_ticks_L = 0;
int16_t last_ticks_R = 0;

int16_t target_left  = 0;
int16_t target_right = 0;

// ============ RC Receiver ============
#define RC_THROTTLE_PIN 22
#define RC_STEER_PIN    23

volatile uint32_t ch_throttle_start = 0;
volatile uint32_t ch_steer_start    = 0;
volatile uint16_t ch_throttle_val   = 1500;
volatile uint16_t ch_steer_val      = 1500;
volatile uint32_t last_rc_time      = 0;
volatile uint32_t last_pc_time      = 0;

// Ð?c bit GPIO tr?c ti?p t? thanh ghi ? an toàn trong IRAM, không ph? thu?c cache flash
#define PIN_HIGH(pin) ((GPIO.in >> (pin)) & 1U)

void IRAM_ATTR calc_throttle() {
  if (PIN_HIGH(RC_THROTTLE_PIN)) {
    ch_throttle_start = micros();
  } else {
    uint32_t pw = micros() - ch_throttle_start;
    if (pw >= 800 && pw <= 2200) {
      ch_throttle_val = (uint16_t)pw;
      last_rc_time = millis();
    }
  }
}

void IRAM_ATTR calc_steer() {
  if (PIN_HIGH(RC_STEER_PIN)) {
    ch_steer_start = micros();
  } else {
    uint32_t pw = micros() - ch_steer_start;
    if (pw >= 800 && pw <= 2200) {
      ch_steer_val = (uint16_t)pw;
      last_rc_time = millis();
    }
  }
}

// ============ BLE ============
BLEServer         *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected    = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer)    { deviceConnected = true;  }
  void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

// ============ USB Serial non-blocking parser ============
#define RX_BUF_SIZE 64
char    rxBuf[RX_BUF_SIZE];
uint8_t rxIdx = 0;

void parsePCCommand(const char *line) {
  // Ð?nh d?ng: "V <v> <omega>"
  if (line[0] == 'V' && line[1] == ' ') {
    float v = 0, omega = 0;
    if (sscanf(line + 2, "%f %f", &v, &omega) == 2) {
      float vL = v - (omega * WHEEL_BASE / 2.0f);
      float vR = v + (omega * WHEEL_BASE / 2.0f);
      target_left  = (int16_t)constrain((int)(vL * K_SPEED), -150, 150);
      target_right = (int16_t)constrain((int)(vR * K_SPEED), -150, 150);
      last_pc_time = millis();
    }
  }
}

void readSerialNonBlocking() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxIdx > 0) {
        rxBuf[rxIdx] = '\0';
        parsePCCommand(rxBuf);
        rxIdx = 0;
      }
    } else if (rxIdx < RX_BUF_SIZE - 1) {
      rxBuf[rxIdx++] = c;
    } else {
      // Tràn buffer -> reset d? tránh k?t
      rxIdx = 0;
    }
  }
}

// ============ G?i l?nh ============
void sendSpeedCmd(int16_t cmdL, int16_t cmdR) {
  Command.start    = START_FRAME;
  Command.steer    = cmdL;
  Command.speed    = cmdR;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);
  HoverSerial.write((uint8_t*)&Command, sizeof(Command));
}

// ============ Nh?n feedback (gi?i h?n byte m?i l?n) ============
bool receiveFeedback() {
  bool newData = false;
  // Gi?i h?n s? byte x? lý / l?n g?i ? tránh k?t loop khi data d?n ?p
  uint16_t budget = 64;
  while (HoverSerial.available() && budget-- > 0) {
    incomingByte = HoverSerial.read();
    uint16_t bufStartFrame = ((uint16_t)incomingByte << 8) | incomingBytePrev;

    if (bufStartFrame == START_FRAME) {
      p = (byte*)&NewFeedback;
      *p++ = incomingBytePrev;
      *p++ = incomingByte;
      idx = 2;
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {
      *p++ = incomingByte;
      idx++;
    }

    if (idx == sizeof(SerialFeedback)) {
      uint16_t checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.ticksL ^ NewFeedback.ticksR
                                     ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                                     ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp
                                     ^ NewFeedback.cmdLed);
      if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
        // Khóa ng?n d? copy (tránh d?c Feedback dang update gi?a ch?ng)
        memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
        newData = true;
      }
      idx = 0;
    }
    incomingBytePrev = incomingByte;
  }
  return newData;
}

// ============ Odometry + Debug Serial (KHÔNG g?i BLE ? dây) ============
void processOdometry() {
  int16_t delta_L = -(Feedback.ticksL - last_ticks_L);
  int16_t delta_R = +(Feedback.ticksR - last_ticks_R);

  absolute_ticks_L += delta_L;
  absolute_ticks_R += delta_R;

  last_ticks_L = Feedback.ticksL;
  last_ticks_R = Feedback.ticksR;
}

// ============ Timer vars ============
uint32_t last_debug_time   = 0;
uint32_t last_ble_send     = 0;
uint32_t last_cmd_send     = 0;
uint32_t last_feedback_rx  = 0;

long  last_debug_ticks_L = 0;
long  last_debug_ticks_R = 0;
float last_v_L = 0;
float last_v_R = 0;

void sendDebugSerial() {
  uint32_t now = millis();
  float dt = (now - last_debug_time) / 1000.0f;
  if (dt < 0.05f) return;

  long dtL = absolute_ticks_L - last_debug_ticks_L;
  long dtR = absolute_ticks_R - last_debug_ticks_R;

  float v_L = (dtL / TICK_PER_METER) / dt;
  float v_R = (dtR / TICK_PER_METER) / dt;
  float a_L = (v_L - last_v_L) / dt;
  float a_R = (v_R - last_v_R) / dt;

  float v_avg = (v_L + v_R) / 2.0f;
  float a_avg = (a_L + a_R) / 2.0f;
  float bat_v = Feedback.batVoltage / 100.0f;
  float temp_c = Feedback.boardTemp / 10.0f;

  Serial.printf("V:%.2f A:%.2f RC:%u/%u TL:%d TR:%d EL:%ld ER:%ld B:%.1f T:%.1f\n",
                v_avg, a_avg, ch_throttle_val, ch_steer_val,
                target_left, target_right,
                absolute_ticks_L, absolute_ticks_R,
                bat_v, temp_c);

  last_debug_time    = now;
  last_debug_ticks_L = absolute_ticks_L;
  last_debug_ticks_R = absolute_ticks_R;
  last_v_L = v_L;
  last_v_R = v_R;
}

// BLE notify ch?y riêng, chu k? 100ms, ch? khi dang connect
void sendBleTelemetry() {
  if (!deviceConnected) return;
  uint32_t now = millis();
  if (now - last_ble_send < 100) return;
  last_ble_send = now;

  float bat_v  = Feedback.batVoltage / 100.0f;
  float temp_c = Feedback.boardTemp / 10.0f;

  char txString[120];
  snprintf(txString, sizeof(txString),
           "V:%d EL:%ld ER:%ld B:%.1f T:%.1f RC:%u/%u CMD:%d/%d",
           (int)0, absolute_ticks_L, absolute_ticks_R,
           bat_v, temp_c, ch_throttle_val, ch_steer_val,
           target_left, target_right);

  pTxCharacteristic->setValue((uint8_t*)txString, strlen(txString));
  pTxCharacteristic->notify();
}

// ============ Setup ============
void setup() {
  Serial.begin(115200);
  HoverSerial.begin(115200, SERIAL_8N1, 16, 17);

  pinMode(RC_THROTTLE_PIN, INPUT_PULLDOWN);
  pinMode(RC_STEER_PIN,    INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(RC_THROTTLE_PIN), calc_throttle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_STEER_PIN),    calc_steer,    CHANGE);

  BLEDevice::init("ESP32_Hoverbot");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();

  // Kh?i t?o timer d? dt d?u tiên không b? sai
  uint32_t now = millis();
  last_debug_time = now;
  last_ble_send   = now;
  last_cmd_send   = now;
  last_pc_time    = 0;       // chua có l?nh PC
  last_rc_time    = 0;       // chua có tín hi?u RC
  last_feedback_rx = now;

  Serial.println("=========================================");
  Serial.println("  ESP32 HOVERBOT - RC ONLY + BLE DEBUG   ");
  Serial.println("=========================================");
}

// ============ Loop ============
void loop() {
  // 1. Ð?c feedback (non-blocking, có budget byte)
  bool gotFB = receiveFeedback();
  if (gotFB) last_feedback_rx = millis();

  // 2. X? lý odometry khi có feedback
  if (gotFB) processOdometry();

  // 3. Ð?c l?nh PC (non-blocking)
  readSerialNonBlocking();

  // 4. BLE reconnect (non-blocking)
  if (!deviceConnected && oldDeviceConnected) {
    static uint32_t ble_reconnect_time = 0;
    if (ble_reconnect_time == 0) ble_reconnect_time = millis();
    if (millis() - ble_reconnect_time >= 500) {
      pServer->startAdvertising();
      Serial.println("BLE: Disconnected. Restarting advertising...");
      oldDeviceConnected = deviceConnected;
      ble_reconnect_time = 0;
    }
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    Serial.println("BLE: Client Connected!");
  }

  // 5. Debug Serial & BLE telemetry (theo timer riêng)
  sendDebugSerial();
  sendBleTelemetry();

  // 6. Safety: m?t feedback > 300ms -> d?ng xe
  if (millis() - last_feedback_rx > 300) {
    target_left  = 0;
    target_right = 0;
  }

  // 7. G?i l?nh di?u khi?n m?i 50ms
  uint32_t now = millis();
  if (now - last_cmd_send >= 50) {
    last_cmd_send = now;

    int16_t throttle = 0;
    int16_t steer    = 0;
    bool rc_moving   = false;

    bool rc_active   = (last_rc_time != 0) && ((uint32_t)(now - last_rc_time) < 500);
    bool pc_active   = (last_pc_time != 0) && ((uint32_t)(now - last_pc_time) < 1000);

    if (rc_active) {
      throttle = map(constrain(ch_throttle_val, 1000, 2000), 1000, 2000, -150, 150);
      steer    = map(constrain(ch_steer_val,    1000, 2000), 1000, 2000, -150, 150);
      if (abs(throttle) < 15) throttle = 0;
      if (abs(steer)    < 15) steer    = 0;
      if (throttle != 0 || steer != 0) rc_moving = true;
    }

    if (rc_moving) {
      // Uu tiên 1: RC dang g?t
      if (throttle == 0) {
        target_left  = steer;
        target_right = -steer;
      } else {
        if (steer > 0) {
          target_left  = throttle;
          target_right = (int16_t)(throttle * (1.0f - (float)steer / 75.0f));
        } else {
          target_left  = (int16_t)(throttle * (1.0f + (float)steer / 75.0f));
          target_right = throttle;
        }
      }
      target_left  = constrain(target_left,  -150, 150);
      target_right = constrain(target_right, -150, 150);
    }
    else if (pc_active) {
      // Uu tiên 2: l?nh PC còn hi?u l?c (gi? target dã parse)
    }
    else if (rc_active) {
      // Uu tiên 3: RC online nhung dang trung l?p -> d?ng yên
      target_left  = 0;
      target_right = 0;
    }
    else {
      // Không có tín hi?u nào -> d?ng an toàn
      target_left  = 0;
      target_right = 0;
    }

    sendSpeedCmd(target_left, target_right);
  }
}