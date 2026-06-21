import os
from glob import glob
from setuptools import find_packages, setup

package_name = "huitzilin_perception"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/huitzilin_perception"]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models", "iris_depth"),
         glob("models/iris_depth/*")),
        (os.path.join("share", package_name, "models", "projectile"),
         glob("models/projectile/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jordan",
    maintainer_email="j602eng1z@gmail.com",
    description="HuitzilinReflex Week 3: perception pipeline (depth, TF, detection, scenarios).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector = huitzilin_perception.detector_node:main",
            "spawn_projectile = huitzilin_perception.spawn_projectile:main",
            "score_bags = huitzilin_perception.score_bags:main",
        ],
    },
)
