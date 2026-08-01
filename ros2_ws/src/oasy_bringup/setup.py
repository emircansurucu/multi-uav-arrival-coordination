import os
from glob import glob

from setuptools import find_packages, setup

package_name = "oasy_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "params"), glob("params/*.parm")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Emircan Surucu",
    maintainer_email="surucuemircan49@gmail.com",
    description="OASY launch dosyalari ve konfigurasyonlari",
    license="Apache-2.0",
)
