from setuptools import find_packages, setup
from glob import glob

package_name = "golfcart_vehicle_interface"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/params", glob("params/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="habby",
    maintainer_email="a0979580915@gmail.com",
    description="Vehicle interface stub for Golf Cart (awaiting Turing Drive DBW)",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "actuator = golfcart_vehicle_interface.actuator:main",
            "velocity_report = golfcart_vehicle_interface.velocity_report:main",
        ],
    },
)
