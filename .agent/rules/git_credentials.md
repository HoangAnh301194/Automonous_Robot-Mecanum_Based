---
trigger: always_on
description: Ghi nhớ thông tin xác thực Git và tài khoản GitHub trong workspace
---

# THÔNG TIN XÁC THỰC GIT TRONG WORKSPACE

1. **Cấu hình Git Credentials:**
   - Hệ thống đã được cấu hình lưu thông tin xác thực vĩnh viễn qua Git credential helper:
     `git config --global credential.helper store`
   - File lưu trữ token: `~/.git-credentials` (phân quyền `600`), chứa GitHub Personal Access Token của người dùng `HoangAnh301194`.
   - Các lệnh Git tương tác với remote (như `git push`, `git fetch`, `git pull`) trên các nhánh của repository `Automonous_Robot-Mecanum_Based` đều có thể tự động xác thực và chạy ngầm mà không cần người dùng nhập lại Username/Token.

2. **Lưu ý bảo mật:**
   - Không commit file `~/.git-credentials` vào repository.
   - Khi tạo nhánh mới hoặc đẩy code lên remote origin, luôn kiểm tra branch tracking và trạng thái merge với các nhánh liên quan.
