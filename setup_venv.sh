#!/bin/bash
# setup_venv.sh - Create virtual environment for Automonous Robot (Mecanum Based)
# Uses --system-site-packages so ROS2 packages (rclpy, sensor_msgs, etc.) remain accessible.

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$PROJECT_DIR/.venv"

echo "Project dir : $PROJECT_DIR"
echo "Venv dir    : $VENV_DIR"
echo ""

# Step 1: Check Python
echo "[1/5] Checking Python..."
python3 --version
if ! python3 -m venv --help &>/dev/null; then
    echo "ERROR: python3-venv not found. Install it with:"
    echo "  sudo apt install python3-venv python3-pip"
    exit 1
fi

# Step 2: Create venv
echo "[2/5] Creating virtual environment..."
if [ -d "$VENV_DIR" ]; then
    read -p "Venv already exists. Recreate? (y/N): " confirm
    if [[ "$confirm" =~ ^[Yy]$ ]]; then
        rm -rf "$VENV_DIR"
    fi
fi

if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv --system-site-packages "$VENV_DIR"
    echo "Venv created at $VENV_DIR"
fi

# Step 3: Activate
echo "[3/5] Activating venv..."
source "$VENV_DIR/bin/activate"
echo "Active Python: $(which python3)"

# Step 4: Install dependencies
echo "[4/5] Installing dependencies..."
pip install --upgrade pip setuptools wheel

pip install \
    "numpy<2" \
    "opencv-python>=4.8.1.78" \
    "typing-extensions>=4.4.0" \
    "ultralytics==8.4.6" \
    "lap>=0.5.12"

pip install pyrealsense2 || echo "WARNING: pyrealsense2 failed - install Intel RealSense SDK manually if needed."

read -p "Install face-recognition (requires cmake + dlib, ~5-10 min)? (y/N): " install_face
if [[ "$install_face" =~ ^[Yy]$ ]]; then
    if ! command -v cmake &>/dev/null; then
        echo "ERROR: cmake not found. Run: sudo apt install cmake build-essential"
    else
        pip install dlib face-recognition face-recognition-models || echo "WARNING: face-recognition install failed."
    fi
fi

# Step 5: Verify
echo "[5/5] Verifying installation..."
python3 -c "
packages = {'numpy': 'numpy', 'cv2': 'opencv-python', 'ultralytics': 'ultralytics', 'lap': 'lap'}
for module, name in packages.items():
    try:
        m = __import__(module)
        print(f'  OK  {name} (v{getattr(m, \"__version__\", \"n/a\")})')
    except ImportError:
        print(f'  MISSING  {name}')

try:
    import rclpy
    print('  OK  rclpy (ROS2 - system-site-packages)')
except ImportError:
    print('  WARNING  rclpy not found - run: source /opt/ros/humble/setup.bash')

try:
    import tensorrt
    print(f'  OK  tensorrt (v{tensorrt.__version__})')
except ImportError:
    print('  N/A  tensorrt (only needed on Jetson Orin)')
"

echo ""
echo "Setup complete."
echo ""
echo "To activate the environment in future sessions:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $VENV_DIR/bin/activate"
