from setuptools import find_packages
from setuptools import setup
import os
from glob import glob

package_name = "manager"

setup(
    name=package_name,
    version="0.20.3",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files.
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Alexis Hou",
    author_email="",
    maintainer="Alexis Hou",
    maintainer_email="",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description=("Python lifecycle node demo"),
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lc_manager = manager.manager_node:main",
            "node_checker = manager.nodechecker:main",
        ],
    },
)
