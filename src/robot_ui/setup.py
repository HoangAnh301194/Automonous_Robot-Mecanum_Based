from pathlib import Path

from setuptools import find_packages, setup


package_name = "robot_ui"


data_files = [
    ("share/ament_index/resource_index/packages", ["resource/robot_ui"]),
    ("share/robot_ui", ["package.xml"]),
    ("share/robot_ui/config", ["config/emotions.json", "config/tts_pronunciation.json"]),
    ("share/robot_ui/launch", ["launch/kiosk.launch.py"]),
    (
        "share/robot_ui/souta_robot",
        [
            str(p)
            for p in sorted(
                list(Path("souta_robot").glob("*.png"))
                + list(Path("souta_robot").glob("*.jpg"))
            )
        ],
    ),
    (
        "share/robot_ui/souta_robot/videomp4",
        [str(path) for path in sorted(Path("souta_robot/videomp4").glob("*.mp4"))],
    ),
]

package_data = {
    package_name: ["kiosk/rag_docs/*.txt"],
}


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data=package_data,
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orin",
    maintainer_email="orin@todo.todo",
    description="Native ROS 2 touchscreen kiosk for the service robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "robot_ui_kiosk = robot_ui.kiosk.main:main",
        ],
    },
)
