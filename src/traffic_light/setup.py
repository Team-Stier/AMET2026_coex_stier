from glob import glob

from setuptools import find_packages, setup

package_name = "traffic_light"
model_directories = glob(
    "models/traffic_light_yolo26n-3/deploy/*_ncnn_model"
)

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ]
    + [
        ("share/" + package_name + "/" + path, glob(path + "/*"))
        for path in model_directories
    ],
    install_requires=["ncnn", "setuptools", "ultralytics"],
    tests_require=["pytest"],
    zip_safe=False,
    maintainer="Taeyun Kim",
    maintainer_email="taeyunkim@example.com",
    description="Camera traffic-light recognition node.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "traffic_light_node = traffic_light.traffic_light_node:main",
        ],
    },
)
