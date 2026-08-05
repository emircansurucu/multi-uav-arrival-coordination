"""aracın durumunu koordinasyon alanında yayınlar"""
from __future__ import annotations

import time

from oasy_interfaces.msg import VehicleStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ..autopilot_adapter.dds_telemetry import TelemetrySnapshot
from ..mission_manager import MissionSnapshot

STATUS_TOPIC = "/oasy/vehicle_status"  # araç durumlarının yayınlandığı konu
QOS_DEPTH = 10  # qos kuyruk uzunluğu


class StatusPublisher:
    """araç durum mesajını üretir ve yayınlar"""

    def __init__(self, node: Node, vehicle_id: int) -> None:
        """araç kimliğini ve durum yayıncısını hazırlar"""
        self._vehicle_id = vehicle_id
        self._node = node
        self._seq = 0
        # geciken durum mesajlarının yeniden gönderilmesi gerekmez
        qos = QoSProfile(depth=QOS_DEPTH, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._publisher = node.create_publisher(VehicleStatus, STATUS_TOPIC, qos)

    def publish(
        self, snapshot: TelemetrySnapshot, mission: MissionSnapshot
    ) -> VehicleStatus:
        """güncel durum mesajını oluşturup koordinasyon alanında yayınlar"""
        msg = self._build(snapshot, mission)
        self._publisher.publish(msg)
        self._seq += 1
        return msg

    def _build(self, snapshot: TelemetrySnapshot, mission: MissionSnapshot) -> VehicleStatus:
        """telemetri ve görev bilgisinden araç durum mesajı oluşturur"""
        msg = VehicleStatus()
        # damga yalnızca kayıt amacıyla kullanılır
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.vehicle_id = self._vehicle_id
        msg.seq = self._seq
        msg.mission_state = int(mission.state)
        msg.monotonic_ns = time.monotonic_ns()
        msg.target_reached = mission.target_reached
        msg.actual_arrival_monotonic_ns = mission.arrival_monotonic_ns
        # diğer araçlara yalnızca taahhüt edilen plan gönderilir
        msg.planned_arrival_monotonic_ns = mission.committed_plan_monotonic_ns
        msg.arrival_committed = mission.arrival_committed
        msg.earliest_feasible_arrival_monotonic_ns = (
            mission.earliest_feasible_arrival_monotonic_ns
        )
        msg.wind_valid = mission.wind_valid
        msg.wind_speed = float(mission.wind_speed_mps)
        msg.wind_dir_deg = float(mission.wind_from_direction_deg)
        if snapshot.valid:
            msg.latitude = snapshot.position.lat
            msg.longitude = snapshot.position.lon
            msg.altitude_msl = float(snapshot.altitude_msl_m)
            msg.groundspeed = float(snapshot.groundspeed_mps)

        msg.remaining_distance = float(mission.remaining_distance_m)
        msg.eta_seconds = float(mission.eta_s)

        return msg
