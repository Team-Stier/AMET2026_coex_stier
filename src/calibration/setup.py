from glob import glob

from setuptools import find_packages, setup

package_name = "calibration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Taeyun Kim",
    maintainer_email="taeyunkim@example.com",
    description="Camera and odometry calibration node.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "calibration_node = calibration.calibration_node:main",
        ],
    },
)
