"""araç telemetrisi ile koordinasyonu ayrı ROS domain'lerinde çalıştırır"""
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

STATUS_LOG_INTERVAL_S = 5.0  # durum loglarının yazılma aralığı
TELEMETRY_TIMEOUT_S = 3.0  # telemetrinin geçerli kalma süresi
SHUTDOWN_JOIN_TIMEOUT_S = 5.0  # kapanışta thread bekleme süresi
HOME_SANITY_RADIUS_M = 1000.0  # yanlış araç domain'ini yakalamak için pist sınırı


class VehicleSide:
    """araca özel domain'deki AP_DDS bağlantısını tutar"""

    def __init__(self, context: rclpy.Context, config: VehicleConfig) -> None:
        """araç tarafındaki ros düğümünü ve dds bağlantılarını hazırlar"""
        self.node = rclpy.create_node(f"ha{config.vehicle_id}_vehicle", context=context)
        self.telemetry = DdsTelemetry(self.node)
        self.guided = GuidedPositionCommander(self.node)


class CoordinationSide:
    """ortak domain'deki durum yayınını ve araç dinlemeyi yönetir"""

    def __init__(self, context: rclpy.Context, config: VehicleConfig) -> None:
        """koordinasyon düğümünü ve araçlar arası durum akışını hazırlar"""
        self.node = rclpy.create_node(f"ha{config.vehicle_id}_agent", context=context)
        self.publisher = StatusPublisher(self.node, config.vehicle_id)
        self.peers = PeerManager(
            config.vehicle_id, config.peer_stale_after_s, config.peer_lost_after_s
        )
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(VehicleStatus, STATUS_TOPIC, self._on_peer_status, qos)

    def _on_peer_status(self, msg: VehicleStatus) -> None:
        """gelen araç durumunu güncel zaman bilgisiyle kaydeder"""
        self.peers.update(msg, time.monotonic_ns())


def check_home_sanity(logger, position, config: VehicleConfig) -> bool:
    """telemetrinin doğru araca ait olup olmadığını kontrol eder"""
    offset_m = geodesic_distance_m(position, config.home)
    if offset_m <= HOME_SANITY_RADIUS_M:
        logger.info(f"telemetri doğrulandı: kalkış noktasına {offset_m:.0f} m")
        return True
    logger.error(
        f"TELEMETRİ UYUŞMUYOR: okunan konum kendi kalkış noktasından {offset_m:.0f} m uzakta. "
        f"Araç domain {config.vehicle_domain_id} başka bir araca bağlanmış olabilir."
    )
    return False


def make_status_loop(
    vehicle: VehicleSide,
    coordination: CoordinationSide,
    config: VehicleConfig,
    mission: MissionManager,
):
    """durum yayını ve ilerleme logu için zamanlayıcı işlevini oluşturur"""
    logger = coordination.node.get_logger()
    state = {"next_log": 0.0, "home_checked": False}

    def tick() -> None:
        """telemetriyi işler ve güncel araç durumunu yayınlar"""
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
    """görevin güncel ilerleme bilgisini tek satırda loglar"""
    peers = coordination.peers.snapshot()
    peer_text = ", ".join(
        f"HA-{pid}: {coordination.peers.age_s(pid, now_ns):.1f} s"
        for pid in sorted(peers)
    ) or "yok"
    state_name = MissionState(mission.state).name

    if not snapshot.valid:
        logger.warn(f"{state_name} | telemetri bekleniyor | diğer araçlar: {peer_text}")
        return

    if mission.target_reached:
        logger.info(
            f"{state_name} | en yakın geçiş {mission.arrival_min_distance_m:.2f} m | "
            f"irtifa {snapshot.altitude_msl_m:.0f} m MSL | diğer araçlar: {peer_text}"
        )
        return

    plan_error_s = (
        (now_ns + mission.eta_s * 1e9 - mission.planned_arrival_monotonic_ns) / 1e9
        if mission.arrival_committed else float("nan")
    )
    anchor_shift_s = (
        (mission.planned_arrival_monotonic_ns - mission.committed_plan_monotonic_ns) / 1e9
        if mission.arrival_committed else float("nan")
    )  # çalışma planının taahhüt edilen plandan farkı
    wind_text = (
        f"{mission.wind_speed_mps:.1f} m/s {mission.wind_from_direction_deg:.0f}d"
        if mission.wind_valid else "yok"
    )
    committed_in_s = (
        (mission.committed_plan_monotonic_ns - now_ns) / 1e9
        if mission.arrival_committed else float("nan")
    )  # taahhüt edilen varışa kalan süre
    logger.info(
        f"{state_name} | WP{mission.active_wp_index} | "
        f"rota kalan {mission.remaining_distance_m:.0f} m | ETA {mission.eta_s:.0f} s | "
        f"zamanlama hatası {plan_error_s:+.1f} s | çıpa {anchor_shift_s:+.1f} s | "
        f"plan T+{committed_in_s:.0f} s | "
        f"rüzgâr {wind_text} | "
        f"E {mission.robust_earliest_s:.0f} L {mission.robust_latest_s:.0f} "
        f"komut {mission.commanded_airspeed_mps:.1f} m/s | "
        f"yer hızı {snapshot.groundspeed_mps:.1f} m/s | "
        f"irtifa {snapshot.altitude_msl_m:.0f} m MSL | "
        f"düz mesafe {geodesic_distance_m(snapshot.position, config.target):.0f} m | "
        f"telemetri yaşı {age_s:.2f} s | diğer araçlar: {peer_text}"
    )


def _start_context(domain_id: int) -> rclpy.Context:
    """verilen domain için bağımsız bir ros bağlamı başlatır"""
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=domain_id,
               signal_handler_options=SignalHandlerOptions.NO)  # sinyalleri ana süreç yönetir
    return context


def _spin_in_thread(
    executor: SingleThreadedExecutor, name: str, errors: List[str]
) -> threading.Thread:
    """ros çalıştırıcısını ayrı bir thread içinde başlatır"""
    def run() -> None:
        """çalıştırıcıyı döndürür ve oluşan hatayı kaydeder"""
        try:
            executor.spin()
        except Exception as exc:  # noqa: BLE001
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
    """çalıştırıcıları, düğümleri ve ros bağlamlarını güvenli biçimde kapatır"""
    for executor in executors:
        executor.shutdown()
    for node in nodes:
        node.destroy_node()
    for context in contexts:
        rclpy.shutdown(context=context)
    for thread in threads:
        thread.join(timeout=SHUTDOWN_JOIN_TIMEOUT_S)


def main() -> int:
    """iki ROS domain'ini ve görev yöneticisini başlatır"""
    parser = argparse.ArgumentParser(description="OASY araç düğümü")
    parser.add_argument("--config", required=True, type=Path)
    args, _ = parser.parse_known_args()  # ros2 launch tarafından eklenen argümanları yok sayar

    config = load_vehicle_config(args.config)
    logging.basicConfig(
        level=logging.INFO,
        format=f"%(asctime)s HA-{config.vehicle_id} %(name)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )

    vehicle_context = _start_context(config.vehicle_domain_id)
    coordination_context = _start_context(config.coordination_domain_id)

    vehicle = VehicleSide(vehicle_context, config)
    coordination = CoordinationSide(coordination_context, config)

    mission = MissionManager(
        config=config,
        commander=MavlinkCommander(config.mavlink_address),
        telemetry=vehicle.telemetry,
        peer_commitments=coordination.peers.committed_arrivals,
        peer_feasible_arrivals=coordination.peers.feasible_arrivals,
        guided_commander=vehicle.guided,
        peer_wind=coordination.peers.settled_wind,
    )
    coordination.node.create_timer(
        1.0 / config.status_publish_hz,
        make_status_loop(vehicle, coordination, config, mission),
    )
    coordination.node.get_logger().info(
        f"HA-{config.vehicle_id} başlatıldı | araç domain {config.vehicle_domain_id} | "
        f"koordinasyon domain {config.coordination_domain_id}"
    )

    vehicle_executor = SingleThreadedExecutor(context=vehicle_context)
    vehicle_executor.add_node(vehicle.node)
    coordination_executor = SingleThreadedExecutor(context=coordination_context)
    coordination_executor.add_node(coordination.node)

    errors: List[str] = []
    threads = [
        _spin_in_thread(vehicle_executor, "araç", errors),
        _spin_in_thread(coordination_executor, "koordinasyon", errors),
    ]
    mission.start()

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    stop.wait()

    coordination.node.get_logger().info("kapatılıyor")
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
