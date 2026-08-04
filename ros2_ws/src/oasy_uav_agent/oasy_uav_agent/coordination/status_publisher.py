"""Aracin kendi durumunu koordinasyon domain'ine yayinlamasi."""
from __future__ import annotations

import time

from oasy_interfaces.msg import VehicleStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ..autopilot_adapter.dds_telemetry import TelemetrySnapshot
from ..mission_manager import MissionSnapshot

STATUS_TOPIC = "/oasy/vehicle_status"
QOS_DEPTH = 10


class StatusPublisher:
    """VehicleStatus mesajini uretir ve yayinlar."""

    def __init__(self, node: Node, vehicle_id: int) -> None:
        self._vehicle_id = vehicle_id
        self._node = node
        self._seq = 0
        # Durum yayini yuksek frekansli ve eskiyen veri oldugu icin
        # kaybolan bir ornegin yeniden gonderilmesinin degeri yok.
        qos = QoSProfile(depth=QOS_DEPTH, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._publisher = node.create_publisher(VehicleStatus, STATUS_TOPIC, qos)

    def publish(
        self, snapshot: TelemetrySnapshot, mission: MissionSnapshot
    ) -> VehicleStatus:
        msg = self._build(snapshot, mission)
        self._publisher.publish(msg)
        self._seq += 1
        return msg

    def _build(self, snapshot: TelemetrySnapshot, mission: MissionSnapshot) -> VehicleStatus:
        msg = VehicleStatus()
        # stamp insan okunabilir kayit icindir; zamanlama matematigi
        # monotonic_ns uzerinden yurur.
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.vehicle_id = self._vehicle_id
        msg.seq = self._seq
        msg.mission_state = int(mission.state)
        msg.monotonic_ns = time.monotonic_ns()
        msg.target_reached = mission.target_reached
        msg.actual_arrival_monotonic_ns = mission.arrival_monotonic_ns
        # Peer'lar taahhut edilmis plani referans alir; capa duzeltmesi
        # yerel kalir, aksi halde araclar birbirinin kaymasini besler.
        msg.planned_arrival_monotonic_ns = mission.committed_plan_monotonic_ns
        msg.arrival_committed = mission.arrival_committed
        msg.earliest_feasible_arrival_monotonic_ns = (
            mission.earliest_feasible_arrival_monotonic_ns
        )
        # Bekleme kullanmadan E/L; bekleme yetkisi ayri tasinir.
        msg.wind_valid = mission.wind_valid
        msg.wind_speed = float(mission.wind_speed_mps)
        msg.wind_dir_deg = float(mission.wind_from_direction_deg)
        # Varis teshis verileri (en yakin gecis, interpolasyon) bilerek
        # yayinlanmaz; peer kararlarinda kullanilmiyorlar. Rapor icin
        # MissionEvent uzerinden tasinacaklar.

        if snapshot.valid:
            msg.latitude = snapshot.position.lat
            msg.longitude = snapshot.position.lon
            msg.altitude_msl = float(snapshot.altitude_msl_m)
            msg.groundspeed = float(snapshot.groundspeed_mps)

        msg.remaining_distance = float(mission.remaining_distance_m)
        msg.eta_seconds = float(mission.eta_s)

        return msg
