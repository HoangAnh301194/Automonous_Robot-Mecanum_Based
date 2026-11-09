#!/bin/bash
# Giai đoạn 1: BACKUP - Chạy script này TRƯỚC KHI làm bất kỳ thứ gì
set -e

echo "========================================="
echo "  GIAI ĐOẠN 1: BACKUP TOÀN BỘ WORKSPACE"
echo "========================================="
echo ""

# Màu sắc
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

WORKSPACE="/home/a/ros2_ws"
BACKUP_DIR="$HOME"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

cd "$WORKSPACE"

# 1. Kiểm tra git status
echo -e "${YELLOW}[1/5] Kiểm tra git status...${NC}"
if [ -n "$(git status --porcelain)" ]; then
    echo -e "${RED}⚠️  Có files chưa commit!${NC}"
    git status --short
    echo ""
    read -p "Bạn có muốn commit tất cả thay đổi? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${GREEN}Đang commit...${NC}"
        git add -A
        git commit -m "chore: backup state before cleanup - FPS optimization WIP

- HandWaveDetection_Pose: implement NearestTrackSelector and conditional RTMPose
- yolo_ros: add class filter and parameter forwarding
- GUI files: multiple versions exist (need cleanup)
- Status: Patches 1-2 code complete, need benchmark
"
        echo -e "${GREEN}✅ Đã commit!${NC}"
    else
        echo -e "${YELLOW}⚠️  Bỏ qua commit. Sẽ backup files uncommitted.${NC}"
    fi
else
    echo -e "${GREEN}✅ Không có uncommitted changes${NC}"
fi
echo ""

# 2. Tạo backup branch
echo -e "${YELLOW}[2/5] Tạo backup branch...${NC}"
BRANCH_NAME="backup-before-cleanup-$TIMESTAMP"
git branch "$BRANCH_NAME"
echo -e "${GREEN}✅ Đã tạo branch: $BRANCH_NAME${NC}"
echo ""

# 3. Push backup branch (nếu có remote)
echo -e "${YELLOW}[3/5] Push backup branch lên remote (nếu có)...${NC}"
if git remote -v | grep -q origin; then
    read -p "Bạn có muốn push backup branch lên remote? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        git push origin "$BRANCH_NAME" || echo -e "${YELLOW}⚠️  Không thể push (có thể không có quyền hoặc không có internet)${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  Không có remote repository${NC}"
fi
echo ""

# 4. Tạo tar.gz backup
echo -e "${YELLOW}[4/5] Tạo tar.gz backup...${NC}"
BACKUP_FILE="$BACKUP_DIR/ros2_ws_backup_$TIMESTAMP.tar.gz"
echo "Đang nén workspace (bỏ qua build/, install/, log/)..."
echo "Backup sẽ được lưu tại: $BACKUP_FILE"

cd "$HOME"
tar -czf "$BACKUP_FILE" \
    --exclude=ros2_ws/build \
    --exclude=ros2_ws/install \
    --exclude=ros2_ws/log \
    --exclude=ros2_ws/.git \
    --exclude='*/__pycache__' \
    --exclude='*.pyc' \
    ros2_ws

BACKUP_SIZE=$(du -h "$BACKUP_FILE" | cut -f1)
echo -e "${GREEN}✅ Backup hoàn thành: $BACKUP_FILE ($BACKUP_SIZE)${NC}"
echo ""

# 5. Verify backup
echo -e "${YELLOW}[5/5] Verify backup...${NC}"
if tar -tzf "$BACKUP_FILE" > /dev/null 2>&1; then
    echo -e "${GREEN}✅ Backup file hợp lệ!${NC}"
else
    echo -e "${RED}❌ Backup file bị lỗi!${NC}"
    exit 1
fi
echo ""

# Tóm tắt
echo "========================================="
echo -e "${GREEN}  BACKUP HOÀN THÀNH!${NC}"
echo "========================================="
echo ""
echo "📋 Thông tin backup:"
echo "  • Git branch: $BRANCH_NAME"
echo "  • Backup file: $BACKUP_FILE"
echo "  • Kích thước: $BACKUP_SIZE"
echo ""
echo "⚠️  LƯU Ý:"
echo "  • KHÔNG XÓA file backup này cho đến khi cleanup hoàn toàn thành công"
echo "  • Giữ backup ít nhất 1 tháng"
echo "  • Nếu có vấn đề, restore bằng lệnh:"
echo "    tar -xzf $BACKUP_FILE -C ~/"
echo ""
echo "✅ Bây giờ có thể chạy cleanup_stage2_gui.sh"
echo ""
