from setuptools import setup

package_name = "protobuf_mqtt_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Takagi, Isamu",
    maintainer_email="isamu.takagi@tier4.jp",
    description="The adaptor for applications without ROS",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
