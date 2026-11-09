#!/bin/bash
# Giai đoạn 3: SỬA SUBMODULE VÀ RENAME PACKAGES
set -e

echo "========================================="
echo "  GIAI ĐOẠN 3: SUBMODULE & RENAME"
echo "========================================="
echo ""

# Màu sắc
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

WORKSPACE="/home/a/ros2_ws"
SRC_DIR="$WORKSPACE/src"

cd "$WORKSPACE"

echo -e "${YELLOW}⚠️  CẢNH BÁO: Giai đoạn này có rủi ro trung bình!${NC}"
echo "   - Rename package cần rebuild workspace"
echo "   - Phải update tất cả launch files và imports"
echo ""
read -p "Bạn có chắc muốn tiếp tục? (yes/no): " -r
if [[ ! $REPLY =~ ^[Yy][Ee][Ss]$ ]]; then
    echo "Đã hủy."
    exit 0
fi
echo ""

# 1. Fix ros_gz submodule
echo -e "${YELLOW}[1/3] Kiểm tra ros_gz submodule...${NC}"
cd "$WORKSPACE"

if git submodule status | grep -q "^-.*ros_gz"; then
    echo -e "${RED}⚠️  ros_gz submodule chưa được init${NC}"
    read -p "Bạn có dùng Gazebo Harmonic không? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Đang init submodule..."
        git submodule update --init --recursive src/ros_gz
        echo -e "${GREEN}✅ Đã init ros_gz${NC}"
    else
        echo "Xóa ros_gz submodule..."
        git submodule deinit -f src/ros_gz
        git rm -f src/ros_gz
        rm -rf src/ros_gz
        echo -e "${GREEN}✅ Đã xóa ros_gz${NC}"
    fi
else
    echo -e "${GREEN}✅ ros_gz submodule OK${NC}"
fi
echo ""

# 2. Rename mo_hinh -> robot_simulation
echo -e "${YELLOW}[2/3] Rename package mo_hinh -> robot_simulation...${NC}"
if [ -d "$SRC_DIR/mo_hinh" ]; then
    echo "Package mo_hinh hiện tại: 349MB (simulation worlds, meshes, models)"
    read -p "Rename thành robot_simulation? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Đang rename..."
        
        # Rename directory
        mv "$SRC_DIR/mo_hinh" "$SRC_DIR/robot_simulation"
        
        # Update package.xml
        sed -i 's/<name>mo_hinh<\/name>/<name>robot_simulation<\/name>/' \
            "$SRC_DIR/robot_simulation/package.xml"
        sed -i 's/TODO: Package description/Gazebo simulation worlds, 3D models, meshes, and URDF for robot/' \
            "$SRC_DIR/robot_simulation/package.xml"
        
        echo -e "${GREEN}✅ Đã rename package${NC}"
        echo ""
        echo -e "${YELLOW}⚠️  CẦN CẬP NHẬT REFERENCES!${NC}"
        echo "Tìm kiếm 'mo_hinh' trong tất cả files:"
        grep -r "mo_hinh" "$SRC_DIR" --include="*.py" --include="*.xml" --include="*.yaml" --include="*.launch" 2>/dev/null || echo "Không tìm thấy"
        echo ""
        echo "Bạn cần thủ công sửa các file trên, thay 'mo_hinh' -> 'robot_simulation'"
        read -p "Nhấn Enter khi đã sửa xong..." 
    else
        echo "Bỏ qua rename."
    fi
else
    echo -e "${BLUE}ℹ️  Package mo_hinh không tồn tại hoặc đã được rename${NC}"
fi
echo ""

# 3. Update package descriptions
echo -e "${YELLOW}[3/3] Cập nhật package descriptions...${NC}"
echo ""

update_description() {
    local pkg_name=$1
    local new_desc=$2
    local pkg_xml="$SRC_DIR/$pkg_name/package.xml"
    
    if [ -f "$pkg_xml" ]; then
        if grep -q "TODO: Package description" "$pkg_xml"; then
            echo -e "${YELLOW}📝 $pkg_name${NC}"
            echo "   Mô tả mới: $new_desc"
            sed -i "s/TODO: Package description/$new_desc/" "$pkg_xml"
            echo -e "${GREEN}   ✅ Đã cập nhật${NC}"
        else
            echo -e "${BLUE}   ⏭️  $pkg_name đã có description${NC}"
        fi
    fi
}

# Cập nhật descriptions (dựa trên phân tích)
update_description "robot_config_manager" "Centralized configuration manager for robot parameters and settings"
update_description "robot_mission_control" "Mission planning and task management for autonomous operations"
update_description "robot_mapping_nav" "SLAM mapping and navigation integration for autonomous robot"
update_description "robot_navigation_manager" "High-level navigation coordinator and goal management"
update_description "robot_system_monitor" "System health monitoring and diagnostics for robot hardware"
update_description "robot_bringup" "Launch configurations for robot hardware initialization"

echo ""
echo -e "${GREEN}✅ Đã cập nhật descriptions${NC}"
echo ""

# Rebuild workspace
echo -e "${YELLOW}Bạn CẦN rebuild workspace sau những thay đổi này!${NC}"
read -p "Rebuild ngay bây giờ? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    cd "$WORKSPACE"
    echo "Đang clean..."
    rm -rf build/ install/ log/
    echo "Đang build..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Build thành công!${NC}"
        echo "Source workspace: source install/setup.bash"
    else
        echo -e "${RED}❌ Build thất bại! Kiểm tra lỗi ở trên.${NC}"
        echo "Có thể cần revert changes nếu có lỗi không sửa được."
    fi
fi
echo ""

# Commit
echo -e "${YELLOW}Commit changes?${NC}"
git add -A
git status --short
read -p "Commit? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    git commit -m "chore(cleanup): fix submodule, rename mo_hinh, update descriptions

- Fix/remove ros_gz submodule
- Rename mo_hinh -> robot_simulation for clarity
- Update package descriptions from TODO to actual descriptions
- Requires workspace rebuild
"
    echo -e "${GREEN}✅ Đã commit!${NC}"
fi
echo ""

echo "========================================="
echo -e "${GREEN}  GIAI ĐOẠN 3 HOÀN THÀNH!${NC}"
echo "========================================="
echo ""
echo "⚠️  LƯU Ý:"
echo "  • Test kỹ tất cả launch files"
echo "  • Verify robot vẫn hoạt động bình thường"
echo "  • Nếu có lỗi, restore từ backup"
echo ""
