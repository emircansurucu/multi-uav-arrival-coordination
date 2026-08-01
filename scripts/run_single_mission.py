#!/usr/bin/env python3
"""P2: Tek arac icin gorev yukleme, otomatik kalkis, rota takibi, varis ve RTL.

ROS 2 kullanmaz; zincirin MAVLink tarafini dogrulamak icindir.
Kullanim: scripts/run_single_mission.py [--address tcp:127.0.0.1:5760]
"""
from __future__ import annotations

import argparse
import logging
import math
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/oasy_uav_agent"))

from pymavlink import mavutil  # noqa: E402

from oasy_uav_agent.autopilot_adapter import mavlink_link as link  # noqa: E402
from oasy_uav_agent.estimation.geodesy import (  # noqa: E402
    LatLon,
    circle_entry_fraction,
    geodesic_distance_m,
    to_local_xy,
)

logger = logging.getLogger("p2")

# Vaka dokumani Tablo 1, HA-1 rotasi. Son nokta ortak hedeftir.
HA1_ROUTE: List[LatLon] = [
    LatLon(47.556939, -122.295004),
    LatLon(47.565332, -122.265617),
    LatLon(47.571248, -122.245860),
    LatLon(47.564550, -122.230073),
    LatLon(47.543977, -122.240829),
    LatLon(47.535683, -122.228584),
]
HA1_HOME = LatLon(47.530002, -122.302457)
TARGET = HA1_ROUTE[-1]

CRUISE_ALT_MSL_M = 400.0
# Kalkis ogesi 400 m'ye kadar tirmanirsa arac pist civarinda uzun sure kalir;
# tirmanisin geri kalani rota uzerinde surdurulur.
TAKEOFF_ALT_MSL_M = 100.0
TAKEOFF_PITCH_DEG = 15.0
# Dokuman rota noktalari icin en fazla 400 m izin veriyor; L1 takibi icin
# daha dar bir deger seciliyor.
WP_ACCEPT_RADIUS_M = 120.0
# Hedef ogesinde genis kabul yaricapi arac 5 m'ye yaklasmadan gorevi
# bitirecegi icin dar tutulur. Varis karari yine de bu degere bagli degildir.
TARGET_WP_ACCEPT_RADIUS_M = 5.0
ARRIVAL_RADIUS_M = 5.0

TELEMETRY_HZ = 10.0
PROGRESS_LOG_INTERVAL_S = 15.0
MISSION_TIMEOUT_S = 2400.0


def build_mission(home: LatLon, route: List[LatLon]) -> List[link.MissionItem]:
    """Home + kalkis + rota noktalarindan gorev listesi uretir."""
    items = [link.MissionItem(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, home.lat, home.lon, 0.0)]
    items.append(
        link.MissionItem(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0.0, 0.0, TAKEOFF_ALT_MSL_M,
            param1=TAKEOFF_PITCH_DEG,
        )
    )
    for index, point in enumerate(route):
        is_target = index == len(route) - 1
        items.append(
            link.MissionItem(
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                point.lat, point.lon, CRUISE_ALT_MSL_M,
                param2=TARGET_WP_ACCEPT_RADIUS_M if is_target else WP_ACCEPT_RADIUS_M,
            )
        )
    return items


class ArrivalDetector:
    """Hedefin kabul cemberine ilk girisi tespit eder.

    Basari kriteri cembere giristir. Ornekler arasinda kalan bir gecis
    kacirilmasin diye ardisik iki konum arasindaki segment cemberle kesistirilir.
    En yakin gecis mesafesi yalnizca teshis icin tutulur.
    """

    def __init__(self, target: LatLon, radius_m: float) -> None:
        self._target = target
        self._radius_m = radius_m
        self._previous: Optional[Tuple[Tuple[float, float], int]] = None
        self.min_distance_m = math.inf
        self.arrival_monotonic_ns: Optional[int] = None
        self.interpolated = False

    def update(self, position: LatLon, monotonic_ns: int) -> bool:
        """Yeni telemetri ornegini isler, varis olduysa True doner."""
        xy = to_local_xy(position, self._target)
        distance = math.hypot(*xy)
        self.min_distance_m = min(self.min_distance_m, distance)

        if self._previous is not None:
            previous_xy, previous_ns = self._previous
            fraction = circle_entry_fraction(previous_xy, xy, self._radius_m)
            if fraction is not None:
                self.arrival_monotonic_ns = previous_ns + int(
                    fraction * (monotonic_ns - previous_ns)
                )
                self.interpolated = distance > self._radius_m
                return True

        self._previous = (xy, monotonic_ns)
        return False


def fly_mission(conn: mavutil.mavfile) -> ArrivalDetector:
    """Rota takibini izler, hedefe varista dongoyu sonlandirir."""
    detector = ArrivalDetector(TARGET, ARRIVAL_RADIUS_M)
    deadline = time.monotonic() + MISSION_TIMEOUT_S
    next_log = 0.0

    while time.monotonic() < deadline:
        msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
        if msg is None:
            logger.warning("Telemetri kesildi, bekleniyor")
            continue

        position = LatLon(msg.lat / 1e7, msg.lon / 1e7)
        if detector.update(position, time.monotonic_ns()):
            return detector

        now = time.monotonic()
        if now >= next_log:
            next_log = now + PROGRESS_LOG_INTERVAL_S
            logger.info(
                "hedefe %.0f m | irtifa %.0f m MSL | yer hizi %.1f m/s",
                geodesic_distance_m(position, TARGET),
                msg.alt / 1000.0,
                math.hypot(msg.vx, msg.vy) / 100.0,
            )

    logger.error("Gorev zaman asimina ugradi, hedefe varilamadi")
    return detector


def report(detector: ArrivalDetector, takeoff_ns: int) -> bool:
    """Varis sonucunu ozetler ve basari durumunu doner."""
    logger.info("--- P2 sonuc ---")
    logger.info("en yakin gecis: %.2f m", detector.min_distance_m)
    if detector.arrival_monotonic_ns is None:
        logger.error("VARIS YOK: 5 m kabul yaricapina girilmedi")
        return False

    flight_s = (detector.arrival_monotonic_ns - takeoff_ns) / 1e9
    logger.info("varis tespiti: %s", "interpolasyon" if detector.interpolated else "dogrudan ornek")
    logger.info("kalkistan varisa: %.1f s", flight_s)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="P2 tek arac gorev kosucusu")
    parser.add_argument("--address", default="tcp:127.0.0.1:5760")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )

    try:
        conn = link.connect(args.address)
    except TimeoutError as exc:
        logger.error("%s", exc)
        return 1

    link.set_message_interval(conn, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, TELEMETRY_HZ)

    if not link.wait_gps_ready(conn):
        return 1
    if not link.upload_mission(conn, build_mission(HA1_HOME, HA1_ROUTE)):
        return 1
    if not link.set_mode(conn, "AUTO"):
        return 1
    if not link.arm(conn):
        return 1

    takeoff_ns = time.monotonic_ns()
    logger.info("Kalkis basladi, rota takibi izleniyor")

    detector = fly_mission(conn)
    arrived = report(detector, takeoff_ns)

    if not link.set_mode(conn, "RTL", timeout_s=10.0):
        logger.error("RTL dogrulanamadi")
        return 1
    logger.info("RTL dogrulandi")

    return 0 if arrived else 1


if __name__ == "__main__":
    sys.exit(main())
