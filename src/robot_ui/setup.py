from pathlib import Path

from setuptools import find_packages, setup


package_name = "robot_ui"


def frontend_data_files():
    web_root = Path(package_name) / "web_dist"
    entries = []
    if not web_root.exists():
        return entries

    for path in web_root.rglob("*"):
        if not path.is_file():
            continue
        relative_parent = path.relative_to(web_root).parent
        install_dir = Path("share") / package_name / "web_dist" / relative_parent
        entries.append((str(install_dir), [str(path)]))
    return entries


data_files = [
    ("share/ament_index/resource_index/packages", ["resource/robot_ui"]),
    ("share/robot_ui", ["package.xml"]),
    (
        "share/robot_ui/config",
        ["config/robot_ui.yaml", "config/emotions.json"],
    ),
    (
        "share/robot_ui/launch",
        ["launch/robot_ui.launch.py", "launch/kiosk.launch.py"],
    ),
    (
        "share/robot_ui/emotion_dasaimochi",
        [
            "emotion_dasaimochi/homeIcon.png",
            "emotion_dasaimochi/locolization.png",
            "emotion_dasaimochi/chatIcon.png",
            "emotion_dasaimochi/clickGoal.png",
            "emotion_dasaimochi/location.png",
            "emotion_dasaimochi/startNavigation.png",
            "emotion_dasaimochi/stopNavigation.png",
        ],
    ),
    (
        "share/robot_ui/emotion_dasaimochi/videomp4",
        [str(path) for path in sorted(Path("emotion_dasaimochi/videomp4").glob("*.mp4"))],
    ),
]
data_files.extend(frontend_data_files())

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
    description="Native touchscreen kiosk and web diagnostics for the robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "robot_ui_server = robot_ui.server:main",
            "robot_ui_kiosk = robot_ui.kiosk.main:main",
        ],
    },
)
