"""Uc aracin agent node'unu ayni anda baslatir.

Launch yalnizca surecleri baslatir; hicbir ucus karari uretmez. Her agent
kendi domain'ini ve rotasini kendi YAML dosyasindan okur, kararlarini
bagimsiz verir.

SITL ve XRCE agent surecleri bu dosyanin kapsaminda degildir; onlari
scripts/start_all.sh baslatir.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

VEHICLE_IDS = (1, 2, 3)


def generate_launch_description() -> LaunchDescription:
    config_dir = os.path.join(get_package_share_directory("oasy_bringup"), "config")

    return LaunchDescription([
        Node(
            package="oasy_uav_agent",
            executable="agent_node",
            name=f"ha{vehicle_id}_agent",
            output="screen",
            emulate_tty=True,
            arguments=["--config", os.path.join(config_dir, f"ha{vehicle_id}.yaml")],
        )
        for vehicle_id in VEHICLE_IDS
    ])
