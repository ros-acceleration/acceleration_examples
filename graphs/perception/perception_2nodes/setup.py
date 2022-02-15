from setuptools import setup
import os
from glob import glob


package_name = "perception_2nodes"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (
            "share/" + package_name + "/resource/data",
            glob("resource/data/*"),
        ),
        ("share/" + package_name + "/worlds", glob("worlds/*.world")),
        ("share/" + package_name + "/models/camera-plugin", glob("models/camera-plugin/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Víctor Mayoral-Vilches",
    author_email="v.mayoralv@gmail.com",
    maintainer="Víctor Mayoral-Vilches",
    maintainer_email="v.mayoralv@gmail.com",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="A simple perception computational graph composed by 2 Nodes.",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "image_raw_publisher = perception_2nodes.image_raw_publisher:main",
        ],
    },
)
