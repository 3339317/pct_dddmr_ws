import os
from glob import glob

from setuptools import find_packages, setup

package_name = "pct_dddmr_nav"


def package_files(directory):
    data = []
    for path, _, filenames in os.walk(directory):
        if filenames:
            data.append((
                os.path.join("share", package_name, path),
                [os.path.join(path, filename) for filename in filenames],
            ))
    return data


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ] + package_files("vendor"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="if",
    maintainer_email="if@example.com",
    description="PCT global planning integrated with DDDMR local navigation.",
    license="GPL-2.0-or-later",
    entry_points={
        "console_scripts": [
            "pct_get_plan_server = pct_dddmr_nav.pct_get_plan_server:main",
            "tomogram_map_publisher = pct_dddmr_nav.tomogram_map_publisher:main",
            "test_localization_publisher = pct_dddmr_nav.test_localization_publisher:main",
            "livox_custom_to_pointcloud2 = pct_dddmr_nav.livox_custom_to_pointcloud2:main",
        ],
    },
)
