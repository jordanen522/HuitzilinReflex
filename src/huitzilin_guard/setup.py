import os
from glob import glob
from setuptools import find_packages, setup

package_name = "huitzilin_guard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/huitzilin_guard"]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jordan",
    maintainer_email="j602eng1z@gmail.com",
    description="Guard alarm: siren and lights when a person is in the box.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "guard = huitzilin_guard.guard_node:main",
            "alert_signal = "
            "huitzilin_guard.alert_signal_node:main",
            "pose_detector = "
            "huitzilin_guard.pose_detector_node:main",
        ],
    },
)
