# Firmware Repository (Robot Hardware Layer)

Thư mục này quản lý toàn bộ mã nguồn firmware nạp cho các vi điều khiển trên xe robot:

---

## 1. `esp32_bridge/` (ESP32 Safety Bridge & RC Controller)
* **Vi điều khiển**: ESP32 Dev Module / ESP32-WROOM-32.
* **Môi trường nạp**: PlatformIO (`pio run -t upload`).
* **Nhiệm vụ**:
  * Kết nối USB Serial với máy tính / Jetson (Baudrate `115200`).
  * Giao tiếp UART2 (GPIO 16 RX, GPIO 17 TX) với bo mạch Hoverboard STM32.
  * Nhận/chuyển tiếp gói nhị phân `0xABCD` giữa ROS 2 `hoverboard-driver-humble` và Hoverboard.
  * **Bộ ngắt an toàn phần cứng (Safety Controller)**:
    * Nút dừng khẩn cấp **E-Stop** (GPIO 4 - Active LOW).
    * Tay điều khiển từ xa **RC Receiver** (CH1 Steer GPIO 22, CH2 Speed GPIO 23) với chế độ ghi đè ưu tiên tuyệt đối khi người dùng gạt cần lái.

---

## 2. `hoverboard_foc/` (Hoverboard STM32 FOC Hack Firmware)
* **Vi điều khiển**: STM32F103RCT6 trên bo xe điện cân bằng.
* **Môi trường nạp**: Mạch nạp **ST-Link v2** qua PlatformIO hoặc Keil uVision.
* **Cấu hình phần cứng chuẩn đã nạp**:
  * Hệ Pin: **24V** (`BAT_CELLS = 6`).
  * Chế độ điều khiển: **FOC Closed-Loop Speed Mode** (`CTRL_MOD_REQ = SPD_MODE`).
  * Giao thức giao tiếp: **UART Serial** (`default_envs = VARIANT_USART`).
  * File nhị phân đã build sẵn: `.pio/build/VARIANT_USART/firmware.bin`.
