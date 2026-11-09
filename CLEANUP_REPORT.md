# 🧹 BÁO CÁO RÀ SOÁT VÀ DỌN DẸP DỰ ÁN

**Ngày:** 2026-11-07  
**Tình trạng:** Dự án đang bị rối, nhiều package và code duplicate

---

## 📊 TỔNG QUAN SỐ LIỆU

| Chỉ số | Số lượng | Ghi chú |
|--------|----------|---------|
| **Packages ROS2** | 20 | Nhiều package có description "TODO" |
| **Files Python** | 177 | Khá nhiều cho 1 workspace |
| **GUI files duplicate** | 8 files | Nhiều version GUI trùng lặp |
| **Files modified (git)** | 30+ | Nhiều thay đổi chưa commit |
| **Dung lượng lớn nhất** | 349MB | `mo_hinh/` package |
| **Submodules** | 2 | `ros2_astra_camera`, `ros_gz` (1 bị missing) |

---

## 🔴 VẤN ĐỀ NGHIÊM TRỌNG

### 1. **FILES GUI TRÙNG LẶP TRONG `/src/`** ⚠️

Files GUI nằm sai vị trí (trong `src/` thay vì trong package):

```
src/gui.py                                (64 bytes)  - Wrapper import gui_v2
src/gui_v2.py                            (1936 dòng) - GUI version 2
src/gui_v3.py                            (1574 dòng) - GUI version 3  ← ACTIVE
src/gui_teleop.py                         (20K)      - GUI teleop
src/new_gui_part.py                       (20K)      - GUI parts
src/robot_hmi_gui.py                      (714 dòng) - Old HMI GUI
src/robot_hmi_gui_new_spec_backup.py      (785 dòng) - Backup HMI
src/map_editor.py                         (8.1K)     - Map editor
```

**Tổng cộng: ~5000 dòng code GUI duplicate!**

**Vấn đề:**
- Không rõ file nào đang được dùng
- Nằm sai vị trí (nên trong `robot_ui` package)
- Có `__pycache__/` compile files → đang được import bừa bãi
- Khó maintain, dễ nhầm lẫn

---

### 2. **SUBMODULE `ros_gz` BỊ MISSING** ⚠️

```bash
-9d7f8c721c233a9ac8b43950129d51e67905523e ros_gz
```

Dấu `-` nghĩa là submodule chưa được checkout → có thể gây lỗi build Gazebo.

---

### 3. **PACKAGE `mo_hinh` QUÁ LỚN VÀ CHƯA RÕ MỤC ĐÍCH** ⚠️

- Dung lượng: **349MB** (lớn nhất trong workspace)
- Description: "TODO: Package description" → chưa document
- Chứa: meshes, models, worlds, maps, urdf, configs
- **Vấn đề:** Có vẻ là package cho simulation/Gazebo nhưng tên không rõ ràng

Nội dung:
```
mo_hinh/
├── config/      - Configs
├── launch/      - Launch files  
├── maps/        - Map files
├── meshes/      - 3D meshes (chiếm nhiều dung lượng)
├── models/      - Gazebo models
├── scripts/     - Scripts
├── urdf/        - Robot URDF
└── worlds/      - Gazebo worlds
```

**Đề xuất:** Đổi tên thành `robot_simulation` hoặc `gazebo_world` để rõ ràng hơn.

---

### 4. **NHIỀU PACKAGE CÓ DESCRIPTION "TODO"** ⚠️

Các package chưa được document đúng cách:

```
nhiemvuboss              - TODO: Package description
robot_config_manager     - TODO: Package description  
robot_mission_control    - TODO: Package description
robot_mapping_nav        - TODO: Package description
robot_navigation_manager - TODO: Package description
robot_system_monitor     - TODO: Package description
robot_bringup            - TODO: Package description
mo_hinh                  - TODO: Package description
```

→ **8/20 packages** chưa có description rõ ràng!

---

### 5. **FILES CHƯA COMMIT (30+ FILES MODIFIED)** ⚠️

Git status cho thấy rất nhiều thay đổi chưa commit:
- HandWaveDetection_Pose: 15+ files
- bno055_imu: 3 files
- depth_obstacle_detector: 3 files
- yolo_ros: nhiều files
- Và nhiều packages khác...

**Rủi ro:** Dễ mất code khi làm việc, khó rollback.

---

### 6. **CẤU TRÚC PACKAGES OVERLAP/DUPLICATE** ⚠️

Có nhiều packages có chức năng gần giống nhau:

**Navigation:**
- `robot_navigation_manager` (52K)
- `robot_mapping_nav` (44K)  
- `nav2_custom_plugins` (52K)

→ **Có thể merge thành 1-2 packages?**

**Bringup:**
- `robot_bringup` (48K)
- `layer1_bringup` (68K - có description rõ)

→ **Cần xem có thể merge không?**

---

## 📁 PHÂN TÍCH CHI TIẾT CÁC PACKAGES

### ✅ **Core Packages (Cần giữ)**

| Package | Dung lượng | Mục đích | Trạng thái |
|---------|-----------|----------|-----------|
| `yolo_ros` | 276K | YOLO detection & tracking | ✅ Active |
| `HandWaveDetection_Pose` | 240K | RTMPose + hand wave | ✅ Active |
| `robot_ui` | 25M | Native UI + Web UI | ✅ Active |
| `robot_urdf` | 15M | Robot URDF description | ✅ Need |
| `robot_interfaces` | 80K | Custom ROS messages/services | ✅ Need |
| `layer1_bringup` | 68K | ESP32 + EKF fusion | ✅ Well documented |

### ⚠️ **Hardware Drivers (Cần giữ nhưng cần kiểm tra)**

| Package | Dung lượng | Mục đích | Ghi chú |
|---------|-----------|----------|---------|
| `ros2_astra_camera` | 6.8M | Astra camera driver | Submodule ✅ |
| `sllidar_ros2-main` | 788K | LiDAR driver | ✅ |
| `hoverboard-driver-humble` | 200K | Hoverboard motor driver | ✅ |
| `bno055_imu` | 48K | IMU driver | ✅ |
| `imu_tools` | 588K | IMU filtering | ✅ |

### ⚠️ **Odometry/Localization (Có thể overlap)**

| Package | Dung lượng | Mục đích |
|---------|-----------|----------|
| `rf2o_laser_odometry-ros2` | 128K | Laser odometry |
| `laser_filters` | 1.5M | Laser filtering |
| `depth_obstacle_detector` | 196K | Depth obstacle detection |

### ❓ **Packages cần review (TODO description)**

| Package | Dung lượng | Cần làm |
|---------|-----------|---------|
| `nhiemvuboss` | 140K | Kiểm tra mục đích, rename hoặc xóa |
| `mo_hinh` | 349M | Rename → `robot_simulation` |
| `robot_config_manager` | 48K | Review chức năng |
| `robot_mission_control` | 68K | Review có trùng với navigation không |
| `robot_mapping_nav` | 44K | Review có thể merge với navigation_manager |
| `robot_navigation_manager` | 52K | Review có thể merge với mapping_nav |
| `robot_system_monitor` | 44K | Review mục đích |
| `robot_bringup` | 48K | Review có thể merge với layer1_bringup |

### ❌ **Packages có thể xóa/archive**

| Package | Dung lượng | Lý do |
|---------|-----------|-------|
| `ros_gz` | 4K | Submodule missing, có thể không dùng |
| `nav2_custom_plugins` | 52K | Kiểm tra có đang dùng không |

---

## 🎯 KẾ HOẠCH DỌN DẸP (4 GIAI ĐOẠN)

### **GIAI ĐOẠN 1: BACKUP & COMMIT (ƯU TIÊN CAO)** 🔴

**Mục tiêu:** Đảm bảo an toàn trước khi dọn dẹp

```bash
# 1. Commit tất cả thay đổi hiện tại
cd /home/a/ros2_ws
git add -A
git commit -m "chore: backup state before cleanup - FPS optimization WIP"

# 2. Tạo backup branch
git branch backup-before-cleanup-2026-11-07
git push origin backup-before-cleanup-2026-11-07

# 3. Backup toàn bộ workspace
tar -czf ~/ros2_ws_backup_$(date +%Y%m%d_%H%M%S).tar.gz \
    --exclude=build --exclude=install --exclude=log \
    /home/a/ros2_ws
```

**Thời gian:** 10 phút  
**Rủi ro:** Thấp

---

### **GIAI ĐOẠN 2: DỌN DẸP GUI FILES (ƯU TIÊN CAO)** 🟠

**Mục tiêu:** Loại bỏ GUI duplicate, chỉ giữ file active

#### 2.1. Xác định file GUI nào đang được dùng

```bash
# Kiểm tra file nào được import trong launch files
grep -r "gui_v[0-9]\|robot_hmi_gui" ros2_ws/src/*/launch/
grep -r "gui_v[0-9]\|robot_hmi_gui" ros2_ws/*.sh
```

#### 2.2. Di chuyển GUI vào đúng package

```bash
# Giả sử gui_v3.py là version active
mkdir -p ros2_ws/archive/old_gui_files
mv ros2_ws/src/gui.py ros2_ws/archive/old_gui_files/
mv ros2_ws/src/gui_v2.py ros2_ws/archive/old_gui_files/
mv ros2_ws/src/gui_teleop.py ros2_ws/archive/old_gui_files/
mv ros2_ws/src/robot_hmi_gui.py ros2_ws/archive/old_gui_files/
mv ros2_ws/src/robot_hmi_gui_new_spec_backup.py ros2_ws/archive/old_gui_files/
mv ros2_ws/src/new_gui_part.py ros2_ws/archive/old_gui_files/

# Giữ lại gui_v3.py và map_editor.py (nếu đang dùng)
# Di chuyển vào robot_ui package (nếu cần)
```

#### 2.3. Xóa __pycache__

```bash
find ros2_ws/src -type d -name "__pycache__" -exec rm -rf {} +
find ros2_ws/src -type f -name "*.pyc" -delete
```

**Thời gian:** 15 phút  
**Rủi ro:** Trung bình (cần backup trước)

---

### **GIAI ĐOẠN 3: SỬA SUBMODULE & RENAME PACKAGES (ƯU TIÊN TRUNG BÌNH)** 🟡

#### 3.1. Fix submodule ros_gz

```bash
cd ros2_ws/src
git submodule update --init --recursive
# Hoặc xóa nếu không dùng:
# git submodule deinit -f ros_gz
# rm -rf ros_gz
# git rm -f ros_gz
```

#### 3.2. Rename package `mo_hinh` → `robot_simulation`

```bash
cd ros2_ws/src
mv mo_hinh robot_simulation

# Sửa package.xml
sed -i 's/<name>mo_hinh<\/name>/<name>robot_simulation<\/name>/' \
    robot_simulation/package.xml
sed -i 's/TODO: Package description/Gazebo simulation world, models, and URDF for the robot/' \
    robot_simulation/package.xml

# Update tất cả references trong launch files
grep -r "mo_hinh" . --include="*.py" --include="*.xml" --include="*.yaml"
# Sửa thủ công các file tìm thấy
```

#### 3.3. Cập nhật package descriptions

Sửa `package.xml` của các packages có "TODO":

```bash
# Ví dụ:
# nhiemvuboss → "Mission and task management for autonomous robot"
# robot_config_manager → "Centralized configuration manager for robot parameters"
# ...
```

**Thời gian:** 30 phút  
**Rủi ro:** Trung bình (phải rebuild workspace sau khi rename)

---

### **GIAI ĐOẠN 4: REVIEW & MERGE PACKAGES (ƯU TIÊN THẤP)** 🟢

**Mục tiêu:** Giảm số lượng packages bằng cách merge các packages liên quan

#### 4.1. Review và merge navigation packages

Kiểm tra xem có thể merge:
- `robot_navigation_manager` + `robot_mapping_nav` → `robot_navigation`?

#### 4.2. Review bringup packages  

Kiểm tra xem có thể merge:
- `robot_bringup` vào `layer1_bringup`?

#### 4.3. Xóa packages không dùng

Sau khi test kỹ:
```bash
# Di chuyển vào archive thay vì xóa ngay
mv ros2_ws/src/package_name ros2_ws/archive/
```

**Thời gian:** 1-2 giờ  
**Rủi ro:** Cao (cần test kỹ sau khi merge)

---

## 📋 CHECKLIST THỰC HIỆN

### Trước khi bắt đầu:
- [ ] Đọc kỹ toàn bộ báo cáo này
- [ ] Backup toàn bộ workspace (tar.gz)
- [ ] Commit tất cả thay đổi hiện tại
- [ ] Tạo branch backup
- [ ] Test build workspace hiện tại: `colcon build`
- [ ] Test chạy robot để biết baseline hoạt động

### Giai đoạn 1 - Backup:
- [ ] Git commit all changes
- [ ] Create backup branch
- [ ] Create tar.gz backup
- [ ] Verify backup exists

### Giai đoạn 2 - GUI cleanup:
- [ ] Xác định file GUI active (gui_v3.py?)
- [ ] Archive old GUI files
- [ ] Remove __pycache__
- [ ] Test GUI vẫn chạy được
- [ ] Commit changes

### Giai đoạn 3 - Submodule & Rename:
- [ ] Fix ros_gz submodule hoặc remove
- [ ] Rename mo_hinh → robot_simulation
- [ ] Update all references
- [ ] Rebuild workspace
- [ ] Update package descriptions
- [ ] Commit changes

### Giai đoạn 4 - Merge packages (optional):
- [ ] Review navigation packages
- [ ] Review bringup packages
- [ ] Test thoroughly before merge
- [ ] Commit changes

---

## 🎯 TIÊU CHÍ THÀNH CÔNG

Sau khi hoàn thành cleanup:

| Tiêu chí | Hiện tại | Mục tiêu |
|----------|----------|----------|
| GUI files trong /src/ | 8 files | 1-2 files (hoặc 0, move vào package) |
| Packages có "TODO" description | 8/20 | 0/20 |
| __pycache__ trong src/ | Có | Không |
| Files uncommitted | 30+ | 0 |
| Submodules broken | 1 | 0 |
| Package names unclear | 2-3 | 0 |
| Total packages | 20 | 15-18 (sau merge) |

---

## ⚠️ LƯU Ý AN TOÀN

1. **LUÔN BACKUP TRƯỚC KHI XÓA BẤT KỲ THỨ GÌ**
2. **Di chuyển vào `archive/` thay vì xóa ngay**
3. **Commit từng bước nhỏ, không commit hàng loạt**
4. **Test sau mỗi thay đổi lớn**
5. **Giữ backup branch ít nhất 1 tháng**
6. **Không làm giai đoạn 4 trừ khi thực sự cần**

---

## 📞 HỎI ĐÁP

**Q: Có nên xóa các file GUI backup ngay không?**  
A: KHÔNG. Di chuyển vào `archive/` trước. Sau 1-2 tuần chạy ổn định mới xóa.

**Q: Package nào quan trọng nhất không được động vào?**  
A: `yolo_ros`, `HandWaveDetection_Pose`, `robot_ui`, `layer1_bringup`, `robot_interfaces`

**Q: Rename package có rủi ro gì?**  
A: Phải update tất cả launch files, configs, imports. Cần rebuild workspace và test kỹ.

**Q: Có nên làm cleanup ngay bây giờ không?**  
A: NÊN làm Giai đoạn 1 & 2 (backup + GUI cleanup) NGAY.  
   Giai đoạn 3 & 4 làm SAU KHI fix xong vấn đề FPS.

---

**Tạo bởi:** Zed AI Agent  
**File:** `/home/a/ros2_ws/CLEANUP_REPORT.md`
