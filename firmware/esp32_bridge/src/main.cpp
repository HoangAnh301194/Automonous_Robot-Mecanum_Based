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
#include <IRsend.h>

// Định nghĩa chân UART và an toàn
#define PC_BAUD            115200     // Khớp với ROS 2 Humble driver
#define HOVER_BAUD         115200     // Khớp với Hoverboard firmware
#define HOVER_RX_PIN       16         // Nhận từ Hoverboard TX
#define HOVER_TX_PIN       17         // Truyền sang Hoverboard RX
#define ESTOP_PIN          4          // Nút dừng khẩn cấp E-Stop

// Định nghĩa chân phát hồng ngoại (IR) điều khiển ấm nước
#define IR_PIN             18         // Chân phát IR điều khiển transistor 2N2222
#define IR_CODE_POWER      0xD81      // Bật/Tắt nguồn ấm (Symphony 12-bit)
#define IR_CODE_BOIL       0xD82      // Đun sôi nước
#define IR_CODE_PUMP       0xDC3      // Bơm nước lên ấm
#define IR_CODE_MODE       0xD84      // Giữ ấm / hãm trà

// Định nghĩa chân đọc xung RC 2 kênh
#define RC_CH1_PIN         22         // Kênh 1: Lái (Steer)
#define RC_CH2_PIN         23         // Kênh 2: Ga / Tốc độ (Speed)

#define BUFFER_SIZE        1024       // Ring Buffer UART
#define SAFETY_TIMEOUT_MS  200        // Watchdog ngắt nếu mất lệnh PC quá 200ms
#define RC_TIMEOUT_MS      250        // Mất sóng RC quá 250ms -> coi như tắt tay điều khiển

#define START_FRAME        0xABCD
#define START_FRAME_KETTLE 0xABCE

enum KettleCommandId : uint16_t {
  KETTLE_CMD_NONE  = 0,
  KETTLE_CMD_POWER = 1,
  KETTLE_CMD_BOIL  = 2,
  KETTLE_CMD_PUMP  = 3,
  KETTLE_CMD_MODE  = 4
};

// Gói lệnh điều khiển chuẩn Hoverboard FOC (8 byte)
typedef struct {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
} __attribute__((packed)) SerialCommand;

// Gói lệnh điều khiển ấm nước nội bộ ESP32 (8 byte)
typedef struct {
  uint16_t start;     // 0xABCE
  uint16_t cmd_id;    // KettleCommandId
  uint16_t param;     // Dự phòng
  uint16_t checksum;  // start ^ cmd_id ^ param
} __attribute__((packed)) SerialKettleCommand;

static SerialCommand stopCommand;
static SerialCommand rcCommand;

static uint32_t lastPcPacketTime = 0;
static bool emergencyStopped = false;

// Đối tượng phát hồng ngoại và hàng đợi xử lý IR trên Core 0
static IRsend irsend(IR_PIN);
static QueueHandle_t kettleQueue = nullptr;

// Bộ đệm nhận byte stream từ PC để tách frame motor và frame ấm nước
static uint8_t pcRxBuf[8];
static uint8_t pcRxIdx = 0;

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

// Task FreeRTOS chạy độc lập trên Core 0 phát tín hiệu IR điều khiển ấm nước
void ir_kettle_task(void *pvParameters) {
  (void)pvParameters;
  uint16_t cmd_id = 0;
  uint32_t last_ir_send = 0;

  for (;;) {
    if (xQueueReceive(kettleQueue, &cmd_id, portMAX_DELAY) == pdTRUE) {
      uint32_t now = millis();
      // Giới hạn tần suất gửi IR: giãn cách tối thiểu 800ms giữa 2 lần phát
      if (now - last_ir_send < 800) {
        vTaskDelay(pdMS_TO_TICKS(800 - (now - last_ir_send)));
      }

      uint32_t ir_code = 0;
      switch (cmd_id) {
        case KETTLE_CMD_POWER:
          ir_code = IR_CODE_POWER;
          break;
        case KETTLE_CMD_BOIL:
          ir_code = IR_CODE_BOIL;
          break;
        case KETTLE_CMD_PUMP:
          ir_code = IR_CODE_PUMP;
          break;
        case KETTLE_CMD_MODE:
          ir_code = IR_CODE_MODE;
          break;
        default:
          break;
      }

      if (ir_code != 0) {
        // Giao thức Symphony 12-bit, repeat = 0 (phát đúng 1 lần, đặc biệt an toàn cho toggle POWER)
        irsend.sendSymphony(ir_code, 12, 0);
        last_ir_send = millis();
      }
    }
  }
}

// Hàm phân tích byte stream từ PC để tách frame motor 0xABCD và frame ấm nước 0xABCE
void processPcByte(uint8_t b, bool estopTriggered, bool rcActive) {
  pcRxBuf[pcRxIdx++] = b;

  if (pcRxIdx == 1) {
    if (pcRxBuf[0] != 0xCD && pcRxBuf[0] != 0xCE) {
      pcRxIdx = 0;
    }
    return;
  }
  if (pcRxIdx == 2) {
    if (pcRxBuf[1] != 0xAB) {
      if (pcRxBuf[1] == 0xCD || pcRxBuf[1] == 0xCE) {
        pcRxBuf[0] = pcRxBuf[1];
        pcRxIdx = 1;
      } else {
        pcRxIdx = 0;
      }
    }
    return;
  }

  if (pcRxIdx < 8) {
    return;
  }

  // Đủ 8 byte khung gói tin
  uint16_t frame_start = (uint16_t)pcRxBuf[0] | ((uint16_t)pcRxBuf[1] << 8);
  bool valid_frame = false;

  if (frame_start == START_FRAME) {
    // Gói lệnh điều khiển động cơ 0xABCD
    SerialCommand *cmd = (SerialCommand*)pcRxBuf;
    uint16_t calc_cs = (uint16_t)(cmd->start ^ cmd->steer ^ cmd->speed);
    if (calc_cs == cmd->checksum) {
      valid_frame = true;
      // Chỉ gói động cơ hợp lệ mới cập nhật Watchdog
      lastPcPacketTime = millis();
      if (!estopTriggered && !rcActive) {
        Serial2.write(pcRxBuf, 8);
        emergencyStopped = false;
      }
    }
  } else if (frame_start == START_FRAME_KETTLE) {
    // Gói lệnh điều khiển ấm nước 0xABCE
    SerialKettleCommand *kcmd = (SerialKettleCommand*)pcRxBuf;
    uint16_t calc_cs = (uint16_t)(kcmd->start ^ kcmd->cmd_id ^ kcmd->param);
    if (calc_cs == kcmd->checksum) {
      valid_frame = true;
      // TUYỆT ĐỐI KHÔNG cập nhật lastPcPacketTime (tránh làm sai lệch Watchdog motor)
      // TUYỆT ĐỐI KHÔNG chuyển xuống Serial2 (Hoverboard STM32)
      // Đẩy vào hàng đợi FreeRTOS để Core 0 phát IR không chặn luồng motor
      if (kettleQueue != nullptr) {
        uint16_t cid = kcmd->cmd_id;
        xQueueSend(kettleQueue, &cid, 0);
      }
    }
  }

  if (valid_frame) {
    pcRxIdx = 0;
  } else {
    // Gói lỗi checksum hoặc lệch byte -> quét tìm byte start tiếp theo trong đệm
    uint8_t new_idx = 0;
    for (uint8_t i = 1; i < 8; i++) {
      if (pcRxBuf[i] == 0xCD || pcRxBuf[i] == 0xCE) {
        uint8_t rem = 8 - i;
        memmove(pcRxBuf, pcRxBuf + i, rem);
        new_idx = rem;
        break;
      }
    }
    pcRxIdx = new_idx;
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

  // 4. Khởi tạo phát IR ấm nước và Task FreeRTOS trên Core 0
  irsend.begin();
  kettleQueue = xQueueCreate(5, sizeof(uint16_t));
  xTaskCreatePinnedToCore(
    ir_kettle_task,
    "ir_kettle_task",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  // 5. Khởi tạo UART kết nối PC (USB)
  Serial.begin(PC_BAUD);
  Serial.setRxBufferSize(BUFFER_SIZE);

  // 6. Khởi tạo UART2 kết nối Bo Hoverboard
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
    pcRxIdx = 0;
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
    pcRxIdx = 0;
  }
  else {
    // Tay RC ở giữa hoặc tắt -> Chạy chế độ Tự hành từ PC (ROS 2)
    while (Serial.available()) {
      uint8_t b = (uint8_t)Serial.read();
      processPcByte(b, estopTriggered, rcActive);
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

