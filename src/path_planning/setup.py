from setuptools import find_packages, setup

package_name = "path_planning"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Taeyun Kim",
    maintainer_email="taeyunkim@example.com",
    description="RDDF-based path planning node.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "path_planning_node = path_planning.path_planning_node:main",
        ],
    },
)
