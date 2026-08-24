from setuptools import setup

package_name = "calibration"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Taeyun Kim",
    maintainer_email="taeyunkim@example.com",
    description="Empty calibration package placeholder.",
    license="Apache-2.0",
)
