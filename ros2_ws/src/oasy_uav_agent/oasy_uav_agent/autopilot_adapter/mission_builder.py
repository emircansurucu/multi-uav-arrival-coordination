"""Rota noktalarindan ArduPlane gorev listesi uretimi.

Irtifalar MAV_FRAME_GLOBAL ile MSL olarak yazilir; boylece home irtifasina
bagli relatif donusum gerekmez ve dokumandaki 400 m MSL sarti dogrudan
gorev dosyasinda gorunur.
"""
from __future__ import annotations

from typing import List, Sequence

from pymavlink import mavutil

from ..estimation.geodesy import LatLon
from .mavlink_link import MissionItem

TAKEOFF_PITCH_DEG = 15.0
# Hedef ogesinde genis kabul yaricapi kullanilirsa otopilot arac 5 m'ye
# yaklasmadan gorevi tamamlamis sayar. Varis karari bu degere bagli degildir,
# ama aracin hedefin uzerinden gecmesi icin dar tutulur.
TARGET_WP_ACCEPT_RADIUS_M = 5.0


def build_mission(
    home: LatLon,
    route: Sequence[LatLon],
    cruise_alt_msl_m: float,
    takeoff_alt_msl_m: float,
    wp_accept_radius_m: float,
) -> List[MissionItem]:
    """Home + kalkis + rota noktalarindan gorev ogelerini uretir."""
    if len(route) < 1:
        raise ValueError("rota en az bir nokta icermeli")

    items = [MissionItem(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, home.lat, home.lon, 0.0)]
    items.append(
        MissionItem(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0.0, 0.0, takeoff_alt_msl_m,
            param1=TAKEOFF_PITCH_DEG,
        )
    )

    last_index = len(route) - 1
    for index, point in enumerate(route):
        radius_m = TARGET_WP_ACCEPT_RADIUS_M if index == last_index else wp_accept_radius_m
        items.append(
            MissionItem(
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                point.lat, point.lon, cruise_alt_msl_m,
                param2=radius_m,
            )
        )
    return items
