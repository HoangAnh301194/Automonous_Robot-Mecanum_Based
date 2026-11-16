from pathlib import Path
from setuptools import find_packages, setup

package_name = "robot_wakeword"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["robot_wakeword/config/keywords.json"]),
        (
            "share/" + package_name + "/launch",
            [str(path) for path in sorted(Path("launch").glob("*.launch.py"))],
        ),
    ],
    package_data={package_name: ["config/*.json"]},
    include_package_data=True,
    install_requires=["setuptools", "vosk==0.3.45"],
    zip_safe=False,
    maintainer="orin",
    maintainer_email="orin@todo.todo",
    description="Standalone offline Vosk wake word detection for the service robot",
    license="MIT",
    entry_points={
        "console_scripts": [
            "wakeword_probe = robot_wakeword.probe:main",
            "wakeword_download_models = robot_wakeword.download_models:main",
        ],
    },
)
