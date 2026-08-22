import os
from glob import glob
from setuptools import find_packages, setup

package_name = "huitzilin_perception"

setup(
    name=package_name,
    version="0.4.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/huitzilin_perception"]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models", "iris_depth"),
         glob("models/iris_depth/*")),
        # iris_ar0234 is its own entry rather than a models/* glob so that
        # adding a model directory stays a deliberate act. Omitting it is
        # SILENT in the worst way: Gazebo logs "Unable to find uri" once and
        # then starts the world with no drone in it, which reads downstream as
        # a total detection failure rather than a missing install.
        (os.path.join("share", package_name, "models", "iris_ar0234"),
         glob("models/iris_ar0234/*")),
        (os.path.join("share", package_name, "models", "projectile"),
         glob("models/projectile/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jordan",
    maintainer_email="j602eng1z@gmail.com",
    description="HuitzilinReflex Weeks 3-4: perception pipeline + Kalman evasion.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector = huitzilin_perception.detector_node:main",
            "depth_noise = huitzilin_perception.depth_noise_node:main",
            "oracle_detector = huitzilin_perception.oracle_detector_node:main",
            "synthetic_depth_publisher = "
            "huitzilin_perception.synthetic_depth_publisher_node:main",
            "mono_flash_detector = "
            "huitzilin_perception.mono_flash_detector_node:main",
            "spawn_projectile = huitzilin_perception.spawn_projectile:main",
            "score_bags = huitzilin_perception.score_bags:main",
            "evasion = huitzilin_perception.evasion_node:main",
            "dodge_battery = huitzilin_perception.dodge_battery:main",
            "gz_pose_bridge = huitzilin_perception.gz_pose_bridge:main",
            "payload = huitzilin_perception.payload_node:main",
        ],
    },
)
