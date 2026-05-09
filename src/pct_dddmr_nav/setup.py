import os
import shutil
import stat
from glob import glob

from setuptools import find_packages, setup
from setuptools.command.develop import develop

package_name = "pct_dddmr_nav"

CONSOLE_SCRIPTS = [
    ("pct_get_plan_server", "pct_dddmr_nav.pct_get_plan_server", "main"),
    ("tomogram_map_publisher", "pct_dddmr_nav.tomogram_map_publisher", "main"),
    ("test_localization_publisher", "pct_dddmr_nav.test_localization_publisher", "main"),
    ("localization_to_tf", "pct_dddmr_nav.localization_to_tf", "main"),
    ("time_sync_probe", "pct_dddmr_nav.time_sync_probe", "main"),
]


class ColconCompatibleDevelop(develop):
    """Accept legacy colcon/ament develop options removed by newer setuptools."""

    user_options = develop.user_options + [
        ("uninstall", None, "ignored compatibility option"),
        ("editable", None, "ignored compatibility option"),
        ("build-directory=", None, "ignored compatibility option"),
        ("script-dir=", None, "ignored compatibility option"),
    ]
    boolean_options = develop.boolean_options + ["uninstall", "editable"]

    def initialize_options(self):
        super().initialize_options()
        self.uninstall = False
        self.editable = False
        self.build_directory = None
        self.script_dir = None

    def run(self):
        super().run()
        install_root = os.path.abspath(
            os.path.join(os.getcwd(), "..", "..", "install", package_name))
        script_dir = self.script_dir
        if not script_dir or "$base" in script_dir:
            script_dir = os.path.join(install_root, "lib", package_name)
        os.makedirs(script_dir, exist_ok=True)
        bin_dir = os.path.join(install_root, "bin")
        if os.path.isdir(bin_dir):
            for script in os.listdir(bin_dir):
                src = os.path.join(bin_dir, script)
                dst = os.path.join(script_dir, script)
                if os.path.isfile(src):
                    shutil.copy2(src, dst)
        for name, module, function in CONSOLE_SCRIPTS:
            path = os.path.join(script_dir, name)
            with open(path, "w", encoding="utf-8") as f:
                f.write(
                    "#!/usr/bin/env python3\n"
                    "import sys\n"
                    f"from {module} import {function}\n"
                    "if __name__ == '__main__':\n"
                    f"    sys.exit({function}())\n")
            os.chmod(path, os.stat(path).st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


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
            f"{name} = {module}:{function}" for name, module, function in CONSOLE_SCRIPTS
        ],
    },
    cmdclass={"develop": ColconCompatibleDevelop},
)
