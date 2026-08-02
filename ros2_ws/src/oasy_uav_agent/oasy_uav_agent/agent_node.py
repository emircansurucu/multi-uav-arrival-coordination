"""Arac agent node'u: iki rclpy context'i tek surecte calistirir.

Arac context'i araca ozel DDS domain'inden telemetriyi okur, koordinasyon
context'i ortak domain'de durum yayinlar ve peer'lari dinler. AP_DDS topic
isimleri araclar arasinda ayni oldugu icin bu ayrim zorunludur.
"""
from __future__ import annotations

import argparse
import logging
import signal
import threading
import time
from pathlib import Path
from typing import List, Tuple

import rclpy
from oasy_interfaces.msg import VehicleStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from .autopilot_adapter.dds_commands import GuidedPositionCommander
from .autopilot_adapter.dds_telemetry import DdsTelemetry
from .autopilot_adapter.mavlink_link import MavlinkCommander
from .config_model import VehicleConfig, load_vehicle_config
from .coordination.peer_manager import PeerManager
from .coordination.status_publisher import STATUS_TOPIC, StatusPublisher
from .estimation.geodesy import geodesic_distance_m
from .mission_manager import MissionManager, MissionState

STATUS_LOG_INTERVAL_S = 5.0
TELEMETRY_TIMEOUT_S = 3.0
SHUTDOWN_JOIN_TIMEOUT_S = 5.0
# AP_DDS topic isimleri butun araclarda ayni oldugu icin yanlis domain'e
# baglanmak sessizce baska bir aracin telemetrisini okumak demektir. Ilk
# telemetri konumu kendi kalkis noktasindan bu kadar uzaksa hata verilir.
HOME_SANITY_RADIUS_M = 1000.0


class VehicleSide:
    """Araca ozel domain: AP_DDS telemetrisi."""

    def __init__(self, context: rclpy.Context, config: VehicleConfig) -> None:
        self.node = rclpy.create_node(f"ha{config.vehicle_id}_vehicle", context=context)
        self.telemetry = DdsTelemetry(self.node)
        self.guided = GuidedPositionCommander(self.node)


class CoordinationSide:
    """Ortak domain: durum yayini ve peer dinleme."""

    def __init__(self, context: rclpy.Context, config: VehicleConfig) -> None:
        self.node = rclpy.create_node(f"ha{config.vehicle_id}_agent", context=context)
        self.publisher = StatusPublisher(self.node, config.vehicle_id)
        self.peers = PeerManager(
            config.vehicle_id, config.peer_stale_after_s, config.peer_lost_after_s
        )
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(VehicleStatus, STATUS_TOPIC, self._on_peer_status, qos)

    def _on_peer_status(self, msg: VehicleStatus) -> None:
        self.peers.update(msg, time.monotonic_ns())


def check_home_sanity(logger, position, config: VehicleConfig) -> bool:
    """Okunan telemetrinin gercekten bu araca ait oldugunu dogrular."""
    offset_m = geodesic_distance_m(position, config.home)
    if offset_m <= HOME_SANITY_RADIUS_M:
        logger.info(f"telemetri dogrulandi: kalkis noktasina {offset_m:.0f} m")
        return True
    logger.error(
        f"TELEMETRI UYUSMUYOR: okunan konum kendi kalkis noktasindan {offset_m:.0f} m uzakta. "
        f"Arac domain {config.vehicle_domain_id} baska bir araca baglanmis olabilir."
    )
    return False


def make_status_loop(
    vehicle: VehicleSide,
    coordination: CoordinationSide,
    config: VehicleConfig,
    mission: MissionManager,
):
    """Periyodik durum yayini ve ilerleme logu ureten geri cagriyi doner."""
    logger = coordination.node.get_logger()
    state = {"next_log": 0.0, "home_checked": False}

    def tick() -> None:
        now_ns = time.monotonic_ns()
        snapshot = vehicle.telemetry.snapshot()
        age_s = snapshot.age_s(now_ns)
        mission_snapshot = mission.snapshot()

        if snapshot.valid and not state["home_checked"]:
            state["home_checked"] = True
            check_home_sanity(logger, snapshot.position, config)

        coordination.publisher.publish(snapshot, mission_snapshot)

        if time.monotonic() < state["next_log"]:
            return
        state["next_log"] = time.monotonic() + STATUS_LOG_INTERVAL_S
        _log_progress(logger, snapshot, age_s, mission_snapshot, coordination, config, now_ns)

    return tick


def _log_progress(logger, snapshot, age_s, mission, coordination, config, now_ns) -> None:
    peers = coordination.peers.snapshot()
    peer_text = ", ".join(
        f"HA-{pid}: {coordination.peers.age_s(pid, now_ns):.1f} s"
        for pid in sorted(peers)
    ) or "yok"
    state_name = MissionState(mission.state).name

    if not snapshot.valid:
        logger.warn(f"{state_name} | telemetri bekleniyor | peer: {peer_text}")
        return

    if mission.target_reached:
        logger.info(
            f"{state_name} | en yakin gecis {mission.arrival_min_distance_m:.2f} m | "
            f"irtifa {snapshot.altitude_msl_m:.0f} m MSL | peer: {peer_text}"
        )
        return

    plan_error_s = (
        (now_ns + mission.eta_s * 1e9 - mission.planned_arrival_monotonic_ns) / 1e9
        if mission.arrival_committed else float("nan")
    )
    # Capa kaymasi: calisma plani ile peer'lara taahhut edilen plan arasindaki
    # fark. Degisken ruzgarda planin araci takip edip etmedigini gormek icin
    # gerekli; capa log'u yalnizca 1 saniyelik siçramalarda yaziyor ve yavas
    # birikmeyi gostermiyor.
    anchor_shift_s = (
        (mission.planned_arrival_monotonic_ns - mission.committed_plan_monotonic_ns) / 1e9
        if mission.arrival_committed else float("nan")
    )
    wind_text = (
        f"{mission.wind_speed_mps:.1f} m/s {mission.wind_from_direction_deg:.0f}d"
        if mission.wind_valid else "yok"
    )
    # Taahhut edilen plana kalan sure. Plan revizyonu yalnizca >=1 s kaymalari
    # logluyor; kucuk kaymalar birikip sessizce plani oynatabiliyor.
    committed_in_s = (
        (mission.committed_plan_monotonic_ns - now_ns) / 1e9
        if mission.arrival_committed else float("nan")
    )
    logger.info(
        f"{state_name} | WP{mission.active_wp_index} | "
        f"rota kalan {mission.remaining_distance_m:.0f} m | ETA {mission.eta_s:.0f} s | "
        f"zamanlama hatasi {plan_error_s:+.1f} s | capa {anchor_shift_s:+.1f} s | "
        f"plan T+{committed_in_s:.0f} s | "
        f"ruzgar {wind_text} | "
        f"komut {mission.commanded_airspeed_mps:.1f} m/s | "
        f"yer hizi {snapshot.groundspeed_mps:.1f} m/s | "
        f"irtifa {snapshot.altitude_msl_m:.0f} m MSL | "
        f"duz mesafe {geodesic_distance_m(snapshot.position, config.target):.0f} m | "
        f"telemetri yasi {age_s:.2f} s | peer: {peer_text}"
    )


def _start_context(domain_id: int) -> rclpy.Context:
    context = rclpy.Context()
    # Sinyal isleyicisini surec sahibi kurar; her context kendi isleyicisini
    # kurarsa SIGINT davranisi ongorulemez hale gelir.
    rclpy.init(context=context, domain_id=domain_id,
               signal_handler_options=SignalHandlerOptions.NO)
    return context


def _spin_in_thread(
    executor: SingleThreadedExecutor, name: str, errors: List[str]
) -> threading.Thread:
    def run() -> None:
        try:
            executor.spin()
        except Exception as exc:  # noqa: BLE001 - thread olurse sistem sessizce durur
            # Hatayi kapanisa saklamak, executor'un olusunu gorunmez kiliyor:
            # yayin durur ama surec calismaya devam eder. Hemen bildirilmeli.
            logging.getLogger(__name__).exception("%s executor'u durdu", name)
            errors.append(f"{name}: {exc}")

    thread = threading.Thread(target=run, daemon=False, name=f"{name}_executor")
    thread.start()
    return thread


def _shutdown(
    executors: Tuple[SingleThreadedExecutor, ...],
    nodes: Tuple[Node, ...],
    contexts: Tuple[rclpy.Context, ...],
    threads: List[threading.Thread],
) -> None:
    for executor in executors:
        executor.shutdown()
    for node in nodes:
        node.destroy_node()
    for context in contexts:
        rclpy.shutdown(context=context)
    for thread in threads:
        thread.join(timeout=SHUTDOWN_JOIN_TIMEOUT_S)


def main() -> int:
    parser = argparse.ArgumentParser(description="OASY arac agent node'u")
    parser.add_argument("--config", required=True, type=Path)
    # ros2 launch, Node eylemine --ros-args ekliyor; bunlari yok sayiyoruz.
    args, _ = parser.parse_known_args()

    config = load_vehicle_config(args.config)
    # Gorev yoneticisi ve MAVLink katmani standart logging kullanir; rclpy
    # bunlari yapilandirmadigi icin burada acikca kuruluyor.
    logging.basicConfig(
        level=logging.INFO,
        format=f"%(asctime)s HA-{config.vehicle_id} %(name)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )

    vehicle_context = _start_context(config.vehicle_domain_id)
    coordination_context = _start_context(config.coordination_domain_id)

    vehicle = VehicleSide(vehicle_context, config)
    coordination = CoordinationSide(coordination_context, config)

    # Gorev yoneticisi kendi thread'inde calisir: MAVLink cagrilari
    # bloklayici oldugu icin executor thread'lerinden cagrilamaz.
    mission = MissionManager(
        config,
        MavlinkCommander(config.mavlink_address),
        vehicle.telemetry,
        coordination.peers.committed_arrivals,
        coordination.peers.feasible_arrivals,
        vehicle.guided,
        coordination.peers.settled_wind,
    )
    coordination.node.create_timer(
        1.0 / config.status_publish_hz,
        make_status_loop(vehicle, coordination, config, mission),
    )
    coordination.node.get_logger().info(
        f"HA-{config.vehicle_id} baslatildi | arac domain {config.vehicle_domain_id} | "
        f"koordinasyon domain {config.coordination_domain_id}"
    )

    vehicle_executor = SingleThreadedExecutor(context=vehicle_context)
    vehicle_executor.add_node(vehicle.node)
    coordination_executor = SingleThreadedExecutor(context=coordination_context)
    coordination_executor.add_node(coordination.node)

    errors: List[str] = []
    threads = [
        _spin_in_thread(vehicle_executor, "arac", errors),
        _spin_in_thread(coordination_executor, "koordinasyon", errors),
    ]
    mission.start()

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    stop.wait()

    coordination.node.get_logger().info("kapatiliyor")
    mission.stop()
    _shutdown(
        (vehicle_executor, coordination_executor),
        (vehicle.node, coordination.node),
        (vehicle_context, coordination_context),
        threads,
    )

    for error in errors:
        print(f"executor hatasi: {error}")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
