from glob import glob

from setuptools import find_packages, setup

package_name = "rddf_visualizer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Taeyun Kim",
    maintainer_email="taeyunkim@example.com",
    description="RDDF path visualization node.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rddf_visualizer_node = rddf_visualizer.rddf_visualizer_node:main",
        ],
    },
)
