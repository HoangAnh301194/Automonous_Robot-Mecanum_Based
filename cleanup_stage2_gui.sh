#!/bin/bash
# Giai đoạn 2: DỌN DẸP GUI FILES
set -e

echo "========================================="
echo "  GIAI ĐOẠN 2: DỌN DẸP GUI FILES"
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
ARCHIVE_DIR="$WORKSPACE/archive/old_gui_files"

cd "$WORKSPACE"

# Kiểm tra đã backup chưa
if [ ! -f "$HOME/ros2_ws_backup_"*.tar.gz ]; then
    echo -e "${RED}❌ CHƯA CÓ BACKUP!${NC}"
    echo "Vui lòng chạy cleanup_stage1_backup.sh trước!"
    exit 1
fi

echo -e "${GREEN}✅ Đã có backup, tiếp tục...${NC}"
echo ""

# 1. Xác định file GUI nào đang được dùng
echo -e "${YELLOW}[1/6] Xác định file GUI đang được dùng...${NC}"
echo ""
echo "Các file GUI hiện tại trong src/:"
ls -lh "$SRC_DIR"/*.py 2>/dev/null | grep -E "gui|hmi|map_editor" || true
echo ""

echo "Tìm kiếm references trong launch files và scripts:"
echo -e "${BLUE}--- Launch files ---${NC}"
grep -r "gui_v[0-9]\|robot_hmi_gui\|map_editor" "$SRC_DIR"/*/launch/ 2>/dev/null || echo "Không tìm thấy"
echo ""
echo -e "${BLUE}--- Shell scripts ---${NC}"
grep -r "gui_v[0-9]\|robot_hmi_gui\|map_editor" "$WORKSPACE"/*.sh 2>/dev/null || echo "Không tìm thấy"
echo ""
echo -e "${BLUE}--- Python imports ---${NC}"
grep -r "import.*gui\|from.*gui" "$SRC_DIR" --include="*.py" 2>/dev/null | grep -v __pycache__ || echo "Không tìm thấy"
echo ""

read -p "File GUI nào đang ACTIVE? (gui_v3/gui_v2/robot_hmi_gui/other): " ACTIVE_GUI
echo ""

# 2. Tạo thư mục archive
echo -e "${YELLOW}[2/6] Tạo thư mục archive...${NC}"
mkdir -p "$ARCHIVE_DIR"
echo -e "${GREEN}✅ Đã tạo: $ARCHIVE_DIR${NC}"
echo ""

# 3. Di chuyển files GUI cũ vào archive
echo -e "${YELLOW}[3/6] Di chuyển files GUI không dùng vào archive...${NC}"

FILES_TO_ARCHIVE=(
    "gui.py"
    "gui_v2.py"
    "gui_v3.py"
    "gui_teleop.py"
    "new_gui_part.py"
    "robot_hmi_gui.py"
    "robot_hmi_gui_new_spec_backup.py"
)

for file in "${FILES_TO_ARCHIVE[@]}"; do
    if [ -f "$SRC_DIR/$file" ]; then
        # Bỏ qua file active
        if [[ "$file" == *"$ACTIVE_GUI"* ]]; then
            echo -e "${BLUE}⏭️  Giữ lại: $file (ACTIVE)${NC}"
        else
            echo -e "${YELLOW}📦 Archive: $file${NC}"
            mv "$SRC_DIR/$file" "$ARCHIVE_DIR/"
        fi
    fi
done
echo ""

# Hỏi về map_editor.py
if [ -f "$SRC_DIR/map_editor.py" ]; then
    read -p "Có giữ lại map_editor.py không? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        mv "$SRC_DIR/map_editor.py" "$ARCHIVE_DIR/
        echo -e "${GREEN}✅ Đã archive map_editor.py${NC}"
    fi
fi
echo ""

# 4. Xóa __pycache__
echo -e "${YELLOW}[4/6] Xóa tất cả __pycache__ và .pyc files...${NC}"
find "$SRC_DIR" -type d -name "__pycache__" -print -exec rm -rf {} + 2>/dev/null || true
find "$SRC_DIR" -type f -name "*.pyc" -print -delete 2>/dev/null || true
echo -e "${GREEN}✅ Đã xóa cache files${NC}"
echo ""

# 5. Add .gitignore cho __pycache__
echo -e "${YELLOW}[5/6] Cập nhật .gitignore...${NC}"
if ! grep -q "__pycache__" "$WORKSPACE/.gitignore" 2>/dev/null; then
    echo "" >> "$WORKSPACE/.gitignore"
    echo "# Python cache" >> "$WORKSPACE/.gitignore"
    echo "__pycache__/" >> "$WORKSPACE/.gitignore"
    echo "*.pyc" >> "$WORKSPACE/.gitignore"
    echo "*.pyo" >> "$WORKSPACE/.gitignore"
    echo -e "${GREEN}✅ Đã thêm __pycache__ vào .gitignore${NC}"
else
    echo -e "${BLUE}ℹ️  .gitignore đã có __pycache__${NC}"
fi
echo ""

# 6. Commit changes
echo -e "${YELLOW}[6/6] Commit changes...${NC}"
git add -A
git status --short
echo ""
read -p "Commit các thay đổi? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    git commit -m "chore(cleanup): archive old GUI files and remove __pycache__

- Archive unused GUI files to archive/old_gui_files/
- Keep $ACTIVE_GUI as active GUI
- Remove all __pycache__ directories
- Add Python cache to .gitignore
- Files archived: ${FILES_TO_ARCHIVE[*]}
"
    echo -e "${GREEN}✅ Đã commit!${NC}"
fi
echo ""

# Tóm tắt
echo "========================================="
echo -e "${GREEN}  GIAI ĐOẠN 2 HOÀN THÀNH!${NC}"
echo "========================================="
echo ""
echo "📋 Kết quả:"
ls -lh "$SRC_DIR"/*.py 2>/dev/null | grep -E "gui|hmi|map_editor" || echo "Không còn GUI files trong src/"
echo ""
echo "📦 Files đã archive:"
ls -lh "$ARCHIVE_DIR"
echo ""
echo "✅ Tiếp theo:"
echo "  1. Test GUI vẫn chạy: python3 $SRC_DIR/$ACTIVE_GUI.py"
echo "  2. Nếu OK, có thể chạy cleanup_stage3_packages.sh (optional)"
echo ""
