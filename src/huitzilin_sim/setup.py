import os
from glob import glob
from setuptools import find_packages, setup

package_name = "huitzilin_sim"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/huitzilin_sim"]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.parm")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jordan",
    maintainer_email="j602eng1z@gmail.com",
    description="HuitzilinReflex Week 2: pymavlink bridge + patrol loop in SITL.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mav_bridge = huitzilin_sim.mav_bridge_node:main",
            "patrol = huitzilin_sim.patrol_node:main",
            "telemetry_logger = huitzilin_sim.telemetry_logger:main",
        ],
    },
)
