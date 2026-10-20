/**
 * ============================================================================
 * ESP32 ULTRA-FAST HARDWARE BRIDGE & SAFETY CONTROLLER WITH RC OVERRIDE
 * Dành cho hệ thống Xe tự hành (AGV / Mobile Robot)
 * 
 * SƠ ĐỒ KẾT NỐI:
 * [PC / Jetson / Pi (ROS 2)] <--- Cáp USB ---> [ESP32] <--- UART2 (3 dây) ---> [Hoverboard STM32]
 *                                                ▲
 *                                                │ (Đọc xung PWM 2 kênh)
 *                                         [Bộ thu RC Receiver]
 * 
 * 1. Chân kết nối ESP32 với bo Hoverboard (USART2):
 *    - ESP32 GPIO 16 (RX2)  <--->  Hoverboard TX (Cáp cảm biến trái)
 *    - ESP32 GPIO 17 (TX2)  <--->  Hoverboard RX (Cáp cảm biến trái)
 *    - ESP32 GND            <--->  Hoverboard GND (Dây đen)
 *    (LƯU Ý: TUYỆT ĐỐI KHÔNG NỐI DÂY NGUỒN ĐỎ TỪ HOVERBOARD SANG ESP32)
 * 
 * 2. Chân kết nối Bộ thu RC (RC Receiver):
 *    - ESP32 GPIO 22 (D22)  <--->  Kênh 1 RC (CH1 - Steering / Lái)
 *    - ESP32 GPIO 23 (D23)  <--->  Kênh 2 RC (CH2 - Speed / Ga)
 *    - ESP32 GND            <--->  GND Bộ thu RC
 *    - ESP32 5V (VIN)       <--->  VCC Bộ thu RC (5V)
 * 
 * 3. Chân nút dừng khẩn cấp (E-Stop):
 *    - ESP32 GPIO 4         <--->  Nối nút nhấn xuống GND (Active LOW)
 * ============================================================================
 */

#include <Arduino.h>

// Định nghĩa chân UART và an toàn
#define PC_BAUD            115200     // Khớp với ROS 2 Humble driver
#define HOVER_BAUD         115200     // Khớp với Hoverboard firmware
#define HOVER_RX_PIN       16         // Nhận từ Hoverboard TX
#define HOVER_TX_PIN       17         // Truyền sang Hoverboard RX
#define ESTOP_PIN          4          // Nút dừng khẩn cấp E-Stop

// Định nghĩa chân đọc xung RC 2 kênh
#define RC_CH1_PIN         22         // Kênh 1: Lái (Steer)
#define RC_CH2_PIN         23         // Kênh 2: Ga / Tốc độ (Speed)

#define BUFFER_SIZE        1024       // Ring Buffer UART
#define SAFETY_TIMEOUT_MS  200        // Watchdog ngắt nếu mất lệnh PC quá 200ms
#define RC_TIMEOUT_MS      250        // Mất sóng RC quá 250ms -> coi như tắt tay điều khiển

#define START_FRAME        0xABCD

// Gói lệnh điều khiển chuẩn Hoverboard FOC
typedef struct {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
} __attribute__((packed)) SerialCommand;

static SerialCommand stopCommand;
static SerialCommand rcCommand;

static uint32_t lastPcPacketTime = 0;
static bool emergencyStopped = false;

// Biến lưu độ rộng xung PWM từ RC (đơn vị: micro-giây)
volatile uint32_t rc_ch1_start = 0;
volatile uint16_t rc_ch1_pwm   = 1500;
volatile uint32_t rc_ch2_start = 0;
volatile uint16_t rc_ch2_pwm   = 1500;
volatile uint32_t rc_last_time  = 0;

// Ngắt đo xung Kênh 1 (GPIO 22 - Steer)
void IRAM_ATTR isr_rc_ch1() {
  uint32_t now = micros();
  if (digitalRead(RC_CH1_PIN) == HIGH) {
    rc_ch1_start = now;
  } else {
    uint32_t pulse = now - rc_ch1_start;
    if (pulse >= 900 && pulse <= 2100) {
      rc_ch1_pwm = pulse;
      rc_last_time = millis();
    }
  }
}

// Ngắt đo xung Kênh 2 (GPIO 23 - Speed)
void IRAM_ATTR isr_rc_ch2() {
  uint32_t now = micros();
  if (digitalRead(RC_CH2_PIN) == HIGH) {
    rc_ch2_start = now;
  } else {
    uint32_t pulse = now - rc_ch2_start;
    if (pulse >= 900 && pulse <= 2100) {
      rc_ch2_pwm = pulse;
      rc_last_time = millis();
    }
  }
}

void setup() {
  // 1. Cấu hình chân E-Stop
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  // 2. Cấu hình chân ngắt đọc RC
  pinMode(RC_CH1_PIN, INPUT_PULLUP);
  pinMode(RC_CH2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RC_CH1_PIN), isr_rc_ch1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH2_PIN), isr_rc_ch2, CHANGE);

  // 3. Chuẩn bị gói tin dừng an toàn (speed = 0, steer = 0)
  stopCommand.start    = START_FRAME;
  stopCommand.steer    = 0;
  stopCommand.speed    = 0;
  stopCommand.checksum = (uint16_t)(stopCommand.start ^ stopCommand.steer ^ stopCommand.speed);

  rcCommand.start      = START_FRAME;

  // 4. Khởi tạo UART kết nối PC (USB)
  Serial.begin(PC_BAUD);
  Serial.setRxBufferSize(BUFFER_SIZE);

  // 5. Khởi tạo UART2 kết nối Bo Hoverboard
  Serial2.begin(HOVER_BAUD, SERIAL_8N1, HOVER_RX_PIN, HOVER_TX_PIN);
  Serial2.setRxBufferSize(BUFFER_SIZE);

  lastPcPacketTime = millis();
}

void loop() {
  uint8_t buffer[256];
  size_t bytesRead;

  // 1. Kiểm tra nút E-Stop
  bool estopTriggered = (digitalRead(ESTOP_PIN) == LOW);

  // 2. Kiểm tra tín hiệu từ Tay điều khiển RC
  bool rcAvailable = (millis() - rc_last_time < RC_TIMEOUT_MS);
  int16_t rcSteer = 0;
  int16_t rcSpeed = 0;
  bool rcActive = false;

  if (rcAvailable) {
    // Vùng chết (Deadband) quanh điểm giữa 1500us
    int16_t rawSteer = (int16_t)rc_ch1_pwm - 1500;
    int16_t rawSpeed = (int16_t)rc_ch2_pwm - 1500;

    // Lọc deadband +-40us (tương đương cần ở giữa vị trí nghỉ)
    if (abs(rawSteer) > 40) {
      rcSteer = constrain(map((int)rc_ch1_pwm, 1000, 2000, -800, 800), -800, 800);
      rcActive = true;
    }
    if (abs(rawSpeed) > 40) {
      rcSpeed = constrain(map((int)rc_ch2_pwm, 1000, 2000, -800, 800), -800, 800);
      rcActive = true;
    }
  }

  // ==========================================================
  // XỬ LÝ LỆNH ĐIỀU KHIỂN (ƯU TIÊN: E-Stop > RC Override > PC ROS2)
  // ==========================================================
  if (estopTriggered) {
    // Dừng khẩn cấp
    if (!emergencyStopped) {
      Serial2.write((uint8_t*)&stopCommand, sizeof(stopCommand));
      emergencyStopped = true;
    }
    // Xóa bộ đệm PC để không tích lũy lệnh
    while (Serial.available()) Serial.read();
  }
  else if (rcActive) {
    // Người điều khiển đang gạt cần RC -> Ưu tiên tuyệt đối lái bằng tay!
    rcCommand.steer = rcSteer;
    rcCommand.speed = rcSpeed;
    rcCommand.checksum = (uint16_t)(rcCommand.start ^ rcCommand.steer ^ rcCommand.speed);

    // Gửi lệnh RC xuống Hoverboard chu kỳ ~20ms (50Hz)
    static uint32_t lastRcSend = 0;
    if (millis() - lastRcSend >= 20) {
      lastRcSend = millis();
      Serial2.write((uint8_t*)&rcCommand, sizeof(rcCommand));
    }
    emergencyStopped = false;

    // Xả bỏ lệnh từ PC để tránh giật khi nhả cần RC
    while (Serial.available()) {
      Serial.read();
      lastPcPacketTime = millis();
    }
  }
  else {
    // Tay RC ở giữa hoặc tắt -> Chạy chế độ Tự hành từ PC (ROS 2)
    bytesRead = Serial.available();
    if (bytesRead > 0) {
      if (bytesRead > sizeof(buffer)) bytesRead = sizeof(buffer);
      Serial.readBytes(buffer, bytesRead);
      lastPcPacketTime = millis();

      // Chuyển tiếp tức thì xuống Hoverboard
      Serial2.write(buffer, bytesRead);
      emergencyStopped = false;
    }

    // Safety Watchdog PC: mất kết nối PC quá 200ms -> ngắt động cơ
    if (millis() - lastPcPacketTime > SAFETY_TIMEOUT_MS) {
      if (!emergencyStopped) {
        Serial2.write((uint8_t*)&stopCommand, sizeof(stopCommand));
        emergencyStopped = true;
      }
    }
  }

  // ==========================================================
  // HƯỚNG PHẢN HỒI: HOVERBOARD -> PC (Odometry, Vận tốc, Pin)
  // (Luôn luôn gửi về ROS 2 kể cả khi đang lái bằng tay RC!)
  // ==========================================================
  bytesRead = Serial2.available();
  if (bytesRead > 0) {
    if (bytesRead > sizeof(buffer)) bytesRead = sizeof(buffer);
    Serial2.readBytes(buffer, bytesRead);

    // Bắn thẳng lên ROS 2 Driver
    Serial.write(buffer, bytesRead);
  }
}

