import os
from glob import glob
from setuptools import find_packages, setup

package_name = "huitzilin_action_escalation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/huitzilin_action_escalation"]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
        # scenarios/ is the only input path this subsystem has today, so a
        # scenario that fails to install does not fail loudly: the player
        # starts, finds nothing to publish, and the silence reads downstream
        # as a recogniser that never fires.
        (os.path.join("share", package_name, "scenarios"),
         glob("scenarios/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jordan",
    maintainer_email="j602eng1z@gmail.com",
    description="Privacy-preserving aggressive-action escalation alerting.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "action_recognizer = "
            "huitzilin_action_escalation.action_recognizer_node:main",
            "alert_signal = "
            "huitzilin_action_escalation.alert_signal_node:main",
            "scenario_player = "
            "huitzilin_action_escalation.scenario_player_node:main",
        ],
    },
)
