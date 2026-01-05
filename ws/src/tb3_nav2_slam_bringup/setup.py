from glob import glob
import os
from setuptools import setup

package_name = "tb3_nav2_slam_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, f"{package_name}.nodes"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/assets/worlds", glob("assets/worlds/*")),
        (
            f"share/{package_name}/assets/models",
            [path for path in glob("assets/models/**/*", recursive=True) if os.path.isfile(path)],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Maintainer",
    maintainer_email="you@example.com",
    description="TurtleBot3 Gazebo Harmonic + SLAM + Nav2 bringup.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "odom_to_tf = tb3_nav2_slam_bringup.nodes.odom_to_tf:main",
        ],
    },
)
