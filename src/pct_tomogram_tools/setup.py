from setuptools import find_packages, setup

package_name = "pct_tomogram_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="if",
    maintainer_email="if@example.com",
    description="ROS2 tools for converting PCD maps to PCT tomogram pickle maps.",
    license="GPL-2.0-or-later",
    entry_points={
        "console_scripts": [
            "pcd_to_tomogram = pct_tomogram_tools.pcd_to_tomogram:main",
        ],
    },
)
