# 🚀 HƯỚNG DẪN NHANH: DỌN DẸP WORKSPACE

## TÓM TẮT TÌNH TRẠNG

Workspace của bạn đang có:
- ❌ **8 files GUI duplicate** trong `src/` (nên chỉ có 1-2 files)
- ❌ **30+ files chưa commit** (rủi ro mất code)
- ❌ **8/20 packages** có description "TODO"
- ❌ **1 submodule bị missing** (ros_gz)
- ❌ Package `mo_hinh` (349MB) tên không rõ ràng
- ❌ `__pycache__/` files rác

## 🎯 KHUYẾN NGHỊ

**Làm NGAY (an toàn, ít rủi ro):**
1. ✅ Giai đoạn 1: Backup (10 phút)
2. ✅ Giai đoạn 2: Dọn GUI files (15 phút)

**Làm SAU (khi rảnh, có rủi ro hơn):**
3. ⚠️ Giai đoạn 3: Fix submodule, rename packages (30 phút)

---

## 📝 HƯỚNG DẪN CHI TIẾT

### BƯỚC 1: ĐỌC BÁO CÁO

```bash
cd /home/a/ros2_ws
cat CLEANUP_REPORT.md  # Đọc kỹ báo cáo đầy đủ
```

### BƯỚC 2: BACKUP (BẮT BUỘC!)

```bash
cd /home/a/ros2_ws
./cleanup_stage1_backup.sh
```

Script này sẽ:
- 