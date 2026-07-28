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

// ============ Gi?i h?n di?u khi?n ============
constexpr int16_t CMD_MAX = 150;
constexpr int16_t RC_DEADBAND = 15;
constexpr uint32_t CONTROL_PERIOD_MS = 50;

// Ðon v?: command/second.
// 0 -> 150 m?t kho?ng 1.25 s; 150 -> 0 m?t kho?ng 0.83 s.
constexpr float CMD_ACCEL_RATE = 120.0f;
constexpr float CMD_DECEL_RATE = 180.0f;

// ============ C?u trúc gói ============
// Hoverboard dã ch?y tank mode: hai tru?ng 16-bit là l?nh bánh trái/ph?i.
typedef struct {
  uint16_t start;
  int16_t  left;
  int16_t  right;
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

// L?nh PC du?c luu riêng, không dùng chung v?i RC.
int16_t pc_target_left  = 0;
int16_t pc_target_right = 0;

// L?nh mong mu?n sau khi ch?n ngu?n RC/PC.
int16_t desired_left  = 0;
int16_t desired_right = 0;

// Tr?ng thái profile hình thang. Dùng float d? tránh m?t ph?n l? m?i chu k?.
float applied_left_f  = 0.0f;
float applied_right_f = 0.0f;
int16_t applied_left  = 0;
int16_t applied_right = 0;

// ============ RC Receiver ============
#define RC_THROTTLE_PIN 22
#define RC_STEER_PIN    23

volatile uint32_t ch_throttle_start = 0;
volatile uint32_t ch_steer_start    = 0;
volatile uint16_t ch_throttle_val   = 1500;
volatile uint16_t ch_steer_val      = 1500;
volatile uint32_t last_rc_time      = 0;
uint32_t last_pc_time               = 0;

// Khai báo tru?c vì emergencyStopImmediately() g?i hàm này.
void sendSpeedCmd(int16_t cmdL, int16_t cmdR);

// Ð?c bit GPIO tr?c ti?p t? thanh ghi d? an toàn trong IRAM.
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

// ============ Hàm h? tr? di?u khi?n ============
void normalizeWheelPair(float &left, float &right) {
  float peak = max(fabsf(left), fabsf(right));
  if (peak > (float)CMD_MAX) {
    float scale = (float)CMD_MAX / peak;
    left  *= scale;
    right *= scale;
  }
}

// Deadband có remap: ngay ngoài vùng ch?t, l?nh b?t d?u g?n 0 thay vì nh?y lên 15.
int16_t applyDeadbandAndRemap(int16_t input) {
  int16_t magnitude = abs(input);

  if (magnitude <= RC_DEADBAND) {
    return 0;
  }

  float normalized =
      (float)(magnitude - RC_DEADBAND) /
      (float)(CMD_MAX - RC_DEADBAND);

  int16_t output = (int16_t)lroundf(normalized * CMD_MAX);
  return input > 0 ? output : -output;
}

// Arcade/tank mixing liên t?c:
//   left  = throttle + steer
//   right = throttle - steer
// N?u robot quay ngu?c hu?ng tay lái, d?i d?u steer ? hai dòng du?i.
void mixTankDrive(int16_t throttle, int16_t steer,
                  int16_t &leftOut, int16_t &rightOut) {
  float left  = (float)throttle + (float)steer;
  float right = (float)throttle - (float)steer;

  // Scale chung d? gi? t? l? hai bánh khi bão hòa.
  normalizeWheelPair(left, right);

  leftOut  = (int16_t)lroundf(left);
  rightOut = (int16_t)lroundf(right);
}

bool oppositeSigns(float a, float b) {
  return (a > 0.0f && b < 0.0f) || (a < 0.0f && b > 0.0f);
}

float effectiveTargetBeforeReverse(float current, float desired) {
  // Không d?i chi?u tr?c ti?p. Tru?c tiên gi?m v? 0, chu k? sau m?i tang chi?u ngu?c.
  if (oppositeSigns(current, desired)) {
    return 0.0f;
  }
  return desired;
}

float allowedStep(float current, float target, float dt) {
  bool increasingMagnitude = fabsf(target) > fabsf(current);
  float rate = increasingMagnitude ? CMD_ACCEL_RATE : CMD_DECEL_RATE;
  return rate * dt;
}

// C?p nh?t d?ng th?i hai bánh theo m?t h? s? chung.
// Nh? dó qu? d?o chuy?n ti?p trong không gian (left, right) không b? méo quá m?nh.
void updateTrapezoidalProfile(int16_t targetLeft,
                              int16_t targetRight,
                              float dt) {
  // Tránh bu?c nh?y l?n n?u loop b? tr? b?t thu?ng.
  dt = constrain(dt, 0.0f, 0.20f);

  float effectiveLeft =
      effectiveTargetBeforeReverse(applied_left_f, (float)targetLeft);
  float effectiveRight =
      effectiveTargetBeforeReverse(applied_right_f, (float)targetRight);

  float deltaLeft  = effectiveLeft  - applied_left_f;
  float deltaRight = effectiveRight - applied_right_f;

  float stepLeft  = allowedStep(applied_left_f, effectiveLeft, dt);
  float stepRight = allowedStep(applied_right_f, effectiveRight, dt);

  float scale = 1.0f;

  if (fabsf(deltaLeft) > stepLeft && fabsf(deltaLeft) > 1e-6f) {
    scale = min(scale, stepLeft / fabsf(deltaLeft));
  }
  if (fabsf(deltaRight) > stepRight && fabsf(deltaRight) > 1e-6f) {
    scale = min(scale, stepRight / fabsf(deltaRight));
  }

  applied_left_f  += deltaLeft  * scale;
  applied_right_f += deltaRight * scale;

  // Kh? sai s? float quanh 0 d? quá trình d?o chi?u không b? treo.
  if (effectiveLeft == 0.0f && fabsf(applied_left_f) < 0.5f) {
    applied_left_f = 0.0f;
  }
  if (effectiveRight == 0.0f && fabsf(applied_right_f) < 0.5f) {
    applied_right_f = 0.0f;
  }

  applied_left_f  = constrain(applied_left_f,  -(float)CMD_MAX, (float)CMD_MAX);
  applied_right_f = constrain(applied_right_f, -(float)CMD_MAX, (float)CMD_MAX);

  applied_left  = (int16_t)lroundf(applied_left_f);
  applied_right = (int16_t)lroundf(applied_right_f);
}

void emergencyStopImmediately() {
  desired_left = desired_right = 0;
  applied_left_f = applied_right_f = 0.0f;
  applied_left = applied_right = 0;
  sendSpeedCmd(0, 0);
}

// ============ USB Serial non-blocking parser ============
#define RX_BUF_SIZE 64
char    rxBuf[RX_BUF_SIZE];
uint8_t rxIdx = 0;

void parsePCCommand(const char *line) {
  // Ð?nh d?ng: "V <v> <omega>"
  if (line[0] == 'V' && line[1] == ' ') {
    float v = 0.0f;
    float omega = 0.0f;

    if (sscanf(line + 2, "%f %f", &v, &omega) == 2) {
      float vL = v - (omega * WHEEL_BASE / 2.0f);
      float vR = v + (omega * WHEEL_BASE / 2.0f);

      float cmdL = vL * K_SPEED;
      float cmdR = vR * K_SPEED;

      // Scale chung thay vì constrain t?ng bánh d?c l?p.
      normalizeWheelPair(cmdL, cmdR);

      pc_target_left  = (int16_t)lroundf(cmdL);
      pc_target_right = (int16_t)lroundf(cmdR);
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
      rxIdx = 0;
    }
  }
}

// ============ G?i l?nh ============
void sendSpeedCmd(int16_t cmdL, int16_t cmdR) {
  Command.start    = START_FRAME;
  Command.left     = cmdL;
  Command.right    = cmdR;
  Command.checksum =
      (uint16_t)(Command.start ^ Command.left ^ Command.right);

  HoverSerial.write((uint8_t*)&Command, sizeof(Command));
}

// ============ Nh?n feedback ============
bool receiveFeedback() {
  bool newData = false;
  uint16_t budget = 64;

  while (HoverSerial.available() && budget-- > 0) {
    incomingByte = HoverSerial.read();
    uint16_t bufStartFrame =
        ((uint16_t)incomingByte << 8) | incomingBytePrev;

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
      uint16_t checksum =
          (uint16_t)(NewFeedback.start ^
                     NewFeedback.ticksL ^
                     NewFeedback.ticksR ^
                     NewFeedback.speedR_meas ^
                     NewFeedback.speedL_meas ^
                     NewFeedback.batVoltage ^
                     NewFeedback.boardTemp ^
                     NewFeedback.cmdLed);

      if (NewFeedback.start == START_FRAME &&
          checksum == NewFeedback.checksum) {
        memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
        newData = true;
      }
      idx = 0;
    }

    incomingBytePrev = incomingByte;
  }

  return newData;
}

// ============ Odometry ============
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
bool feedback_fault_active = false;

long  last_debug_ticks_L = 0;
long  last_debug_ticks_R = 0;
float last_v_L = 0.0f;
float last_v_R = 0.0f;

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

  // Gi? nguyên protocol USB Serial cu d? node ROS 2 parse du?c.
  // TL/TR là l?nh th?c t? sau profile, dúng v?i giá tr? g?i xu?ng hoverboard.
  Serial.printf("V:%.2f A:%.2f RC:%u/%u TL:%d TR:%d EL:%ld ER:%ld B:%.1f T:%.1f\n",
                v_avg, a_avg, ch_throttle_val, ch_steer_val,
                applied_left, applied_right,
                absolute_ticks_L, absolute_ticks_R,
                bat_v, temp_c);

  last_debug_time    = now;
  last_debug_ticks_L = absolute_ticks_L;
  last_debug_ticks_R = absolute_ticks_R;
  last_v_L = v_L;
  last_v_R = v_R;
}

void sendBleTelemetry() {
  if (!deviceConnected) return;

  uint32_t now = millis();
  if (now - last_ble_send < 100) return;
  last_ble_send = now;

  float bat_v  = Feedback.batVoltage / 100.0f;
  float temp_c = Feedback.boardTemp / 10.0f;

  char txString[150];
  // Gi? BLE telemetry tuong thích v?i format cu.
  snprintf(txString, sizeof(txString),
           "V:%d EL:%ld ER:%ld B:%.1f T:%.1f RC:%u/%u CMD:%d/%d",
           (int)0, absolute_ticks_L, absolute_ticks_R,
           bat_v, temp_c, ch_throttle_val, ch_steer_val,
           applied_left, applied_right);

  pTxCharacteristic->setValue((uint8_t*)txString, strlen(txString));
  pTxCharacteristic->notify();
}

// ============ Setup ============
void setup() {
  Serial.begin(115200);
  HoverSerial.begin(115200, SERIAL_8N1, 16, 17);

  pinMode(RC_THROTTLE_PIN, INPUT_PULLDOWN);
  pinMode(RC_STEER_PIN,    INPUT_PULLDOWN);
  attachInterrupt(
      digitalPinToInterrupt(RC_THROTTLE_PIN), calc_throttle, CHANGE);
  attachInterrupt(
      digitalPinToInterrupt(RC_STEER_PIN), calc_steer, CHANGE);

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

  uint32_t now = millis();
  last_debug_time  = now;
  last_ble_send    = now;
  last_cmd_send    = now;
  last_pc_time     = 0;
  last_rc_time     = 0;
  last_feedback_rx = now;

  Serial.println("============================================");
  Serial.println(" ESP32 HOVERBOT - TANK + TRAPEZOID PROFILE ");
  Serial.println("============================================");
}

// ============ Loop ============
void loop() {
  // 1. Nh?n feedback.
  bool gotFB = receiveFeedback();
  if (gotFB) {
    last_feedback_rx = millis();
    processOdometry();
  }

  // 2. Nh?n l?nh PC.
  readSerialNonBlocking();

  // 3. BLE reconnect.
  if (!deviceConnected && oldDeviceConnected) {
    static uint32_t ble_reconnect_time = 0;
    if (ble_reconnect_time == 0) {
      ble_reconnect_time = millis();
    }

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

  // 4. Telemetry.
  sendDebugSerial();
  sendBleTelemetry();

  uint32_t now = millis();

  // 5. Safety override: m?t feedback thì d?ng ngay, không qua profile ch?m.
  if ((uint32_t)(now - last_feedback_rx) > 300) {
    // G?i ngay ? l?n d?u phát hi?n l?i, sau dó duy trì gói zero m?i 50 ms.
    if (!feedback_fault_active ||
        (uint32_t)(now - last_cmd_send) >= CONTROL_PERIOD_MS) {
      emergencyStopImmediately();
      last_cmd_send = now;
    }
    feedback_fault_active = true;
    return;
  }
  feedback_fault_active = false;

  // 6. Ði?u khi?n m?i 50 ms.
  if ((uint32_t)(now - last_cmd_send) >= CONTROL_PERIOD_MS) {
    float dt = (now - last_cmd_send) / 1000.0f;
    last_cmd_send = now;

    // Ch?p bi?n RC volatile v? local d? dùng nh?t quán trong chu k? này.
    uint16_t throttlePulse;
    uint16_t steerPulse;
    uint32_t rcTimestamp;

    noInterrupts();
    throttlePulse = ch_throttle_val;
    steerPulse = ch_steer_val;
    rcTimestamp = last_rc_time;
    interrupts();

    bool rc_active =
        (rcTimestamp != 0) && ((uint32_t)(now - rcTimestamp) < 500);
    bool pc_active =
        (last_pc_time != 0) && ((uint32_t)(now - last_pc_time) < 1000);

    int16_t throttle = 0;
    int16_t steer = 0;
    bool rc_moving = false;

    if (rc_active) {
      throttle = (int16_t)map(
          constrain((int)throttlePulse, 1000, 2000),
          1000, 2000, -CMD_MAX, CMD_MAX);

      steer = (int16_t)map(
          constrain((int)steerPulse, 1000, 2000),
          1000, 2000, -CMD_MAX, CMD_MAX);

      throttle = applyDeadbandAndRemap(throttle);
      steer = applyDeadbandAndRemap(steer);
      rc_moving = (throttle != 0 || steer != 0);
    }

    if (rc_moving) {
      // Uu tiên 1: RC dang du?c g?t.
      mixTankDrive(throttle, steer, desired_left, desired_right);
    }
    else if (pc_active) {
      // Uu tiên 2: l?nh PC còn hi?u l?c.
      desired_left  = pc_target_left;
      desired_right = pc_target_right;
    }
    else {
      // RC ? gi?a ho?c m?t toàn b? ngu?n l?nh: gi?m t?c theo profile v? 0.
      desired_left  = 0;
      desired_right = 0;
    }

    // 7. Sinh profile hình thang r?i g?i l?nh dã làm mu?t.
    updateTrapezoidalProfile(desired_left, desired_right, dt);
    sendSpeedCmd(applied_left, applied_right);
  }
}