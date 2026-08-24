from setuptools import find_packages, setup


package_name = "pose_tf"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/rddf", ["../../rddf/centerline.csv"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Taeyun Kim",
    maintainer_email="taeyunkim@example.com",
    description="Publish map-frame LiDAR poses and transforms from LiDAR odometry.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["pose_tf_node = pose_tf.pose_tf_node:main"]},
)
