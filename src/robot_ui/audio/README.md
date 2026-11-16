# Robot Audio Assets

Thư mục lưu trữ các file âm thanh cục bộ phục vụ phát offline trên robot.

## Cấu trúc thư mục

- `greetings/`: Các câu chào hỏi tự động (khi gặp người, chào Thầy Cô, chào khách).
- `obstacles/`: Các câu cảnh báo, xin nhường đường khi robot gặp vật cản.
- `alerts/`: Các âm thanh thông báo hệ thống (khởi động, pin yếu, hoàn thành nhiệm vụ).

## Cách sử dụng trong code ROS 2

```python
from pathlib import Path
import random
import subprocess
from ament_index_python.packages import get_package_share_directory

def play_scenario_audio(scenario: str = "greetings", filename: str = None):
    """
    Phát file âm thanh theo kịch bản:
    - Nếu có filename: phát đúng file đó.
    - Nếu không có filename: phát ngẫu nhiên 1 file trong thư mục kịch bản.
    """
    base_dir = Path(get_package_share_directory("robot_ui")) / "audio" / scenario
    if not base_dir.exists():
        return

    if filename:
        target_file = base_dir / filename
    else:
        candidates = list(base_dir.glob("*.mp3")) + list(base_dir.glob("*.wav"))
        target_file = random.choice(candidates) if candidates else None

    if target_file and target_file.exists():
        subprocess.Popen(["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", str(target_file)])
```
