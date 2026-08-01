#!/usr/bin/env python3
"""Tek arac icin gorev yukleme, otomatik kalkis, rota takibi, varis ve RTL.

ROS 2 kullanmaz; zincirin MAVLink tarafini bagimsiz dogrulamak icindir.
Gorev mantigi oasy_uav_agent paketindeki modullerden gelir, boylece agent
node ile ayni kod yolu sinanir.

Kullanim: scripts/run_single_mission.py [--config <yaml>] [--address <mavlink>]
"""
from __future__ import annotations

import argparse
import logging
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/oasy_uav_agent"))

from pymavlink import mavutil  # noqa: E402

from oasy_uav_agent.autopilot_adapter import mavlink_link as link  # noqa: E402
from oasy_uav_agent.autopilot_adapter.mission_builder import build_mission  # noqa: E402
from oasy_uav_agent.config_model import VehicleConfig, load_vehicle_config  # noqa: E402
from oasy_uav_agent.estimation.arrival_detector import ArrivalDetector  # noqa: E402
from oasy_uav_agent.estimation.geodesy import LatLon, geodesic_distance_m  # noqa: E402

logger = logging.getLogger("single_mission")

DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[1] / "ros2_ws/src/oasy_bringup/config/ha1.yaml"
)
TELEMETRY_HZ = 10.0
PROGRESS_LOG_INTERVAL_S = 15.0
MISSION_TIMEOUT_S = 2400.0


def fly_mission(conn: mavutil.mavfile, target: LatLon) -> ArrivalDetector:
    """Rota takibini izler, hedefe varista donguyu sonlandirir."""
    detector = ArrivalDetector(target)
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
                geodesic_distance_m(position, target),
                msg.alt / 1000.0,
                math.hypot(msg.vx, msg.vy) / 100.0,
            )

    logger.error("Gorev zaman asimina ugradi, hedefe varilamadi")
    return detector


def report(detector: ArrivalDetector, takeoff_ns: int) -> bool:
    """Varis sonucunu ozetler ve basari durumunu doner."""
    logger.info("--- sonuc ---")
    logger.info("en yakin gecis: %.2f m", detector.min_distance_m)
    if not detector.arrived:
        logger.error("VARIS YOK: kabul yaricapina girilmedi")
        return False

    flight_s = (detector.arrival_monotonic_ns - takeoff_ns) / 1e9
    logger.info("varis tespiti: %s", "interpolasyon" if detector.interpolated else "dogrudan ornek")
    logger.info("kalkistan varisa: %.1f s", flight_s)
    return True


def run(config: VehicleConfig, address: str) -> int:
    try:
        conn = link.connect(address)
    except TimeoutError as exc:
        logger.error("%s", exc)
        return 1

    link.set_message_interval(conn, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, TELEMETRY_HZ)

    mission = build_mission(
        config.home, config.route,
        config.cruise_alt_msl_m, config.takeoff_alt_msl_m, config.wp_accept_radius_m,
    )

    if not link.wait_gps_ready(conn):
        return 1
    if not link.upload_mission(conn, mission):
        return 1
    if not link.set_mode(conn, "AUTO"):
        return 1
    if not link.arm(conn):
        return 1

    takeoff_ns = time.monotonic_ns()
    logger.info("Kalkis basladi, rota takibi izleniyor")

    detector = fly_mission(conn, config.target)
    arrived = report(detector, takeoff_ns)

    if not link.set_mode(conn, "RTL", timeout_s=10.0):
        logger.error("RTL dogrulanamadi")
        return 1
    logger.info("RTL dogrulandi")

    return 0 if arrived else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Tek arac gorev kosucusu")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--address", default=None)
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )

    config = load_vehicle_config(args.config)
    return run(config, args.address or config.mavlink_address)


if __name__ == "__main__":
    sys.exit(main())
