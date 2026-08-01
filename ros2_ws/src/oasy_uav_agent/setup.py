from setuptools import find_packages, setup

package_name = "oasy_uav_agent"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Emircan Surucu",
    maintainer_email="surucuemircan49@gmail.com",
    description="Merkeziyetsiz coklu IHA varis koordinasyonu icin arac agent node'u",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "agent_node = oasy_uav_agent.agent_node:main",
        ],
    },
)
