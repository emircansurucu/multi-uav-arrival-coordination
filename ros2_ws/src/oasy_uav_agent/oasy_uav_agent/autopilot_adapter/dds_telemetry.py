"""ap dds telemetrisini okuyup son geçerli durumu saklar"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ..estimation.geodesy import LatLon

GEOPOSE_TOPIC = "/ap/geopose/filtered"  # filtrelenmiş küresel konum konusu
TWIST_TOPIC = "/ap/twist/filtered"  # filtrelenmiş yer hızı konusu
AIRSPEED_TOPIC = "/ap/airspeed"  # gövde eksenindeki hava hızı konusu
QOS_DEPTH = 10  # abonelik kuyruğu derinliği
MIN_VALID_COORDINATE_DEG = 1e-6  # ekf kurulmadan gelen sıfır koordinatı eleme sınırı


def _is_valid_coordinate(latitude: float, longitude: float) -> bool:
    """koordinatın ekf kurulmadan gelen sıfır değer olmadığını denetler"""
    return (
        abs(latitude) > MIN_VALID_COORDINATE_DEG
        and abs(longitude) > MIN_VALID_COORDINATE_DEG
    )


@dataclass(frozen=True)
class TelemetrySnapshot:
    """tek bir andaki araç telemetrisini taşır"""

    position: Optional[LatLon]
    altitude_msl_m: float
    velocity_east_mps: float  # doğu yönündeki yer hızı
    velocity_north_mps: float  # kuzey yönündeki yer hızı
    airspeed_forward_mps: float  # gövde ileri eksenindeki hava hızı
    airspeed_left_mps: float  # gövde sol eksenindeki hava hızı
    airspeed_up_mps: float  # gövde yukarı eksenindeki hava hızı
    orientation_xyzw: Tuple[float, float, float, float]  # tam araç yönelimi
    updated_monotonic_ns: int  # son konum örneğinin zamanı
    twist_monotonic_ns: int  # son yer hızı örneğinin zamanı
    airspeed_monotonic_ns: int  # son hava hızı örneğinin zamanı

    @property
    def groundspeed_mps(self) -> float:
        """yatay yer hızının büyüklüğünü döner"""
        return math.hypot(self.velocity_east_mps, self.velocity_north_mps)

    @property
    def airspeed_mps(self) -> float:
        """üç eksendeki hava hızının büyüklüğünü döner"""
        return math.sqrt(
            self.airspeed_forward_mps ** 2
            + self.airspeed_left_mps ** 2
            + self.airspeed_up_mps ** 2
        )

    @property
    def valid(self) -> bool:
        """geçerli bir konum örneği bulunup bulunmadığını döner"""
        return self.position is not None

    def age_s(self, now_monotonic_ns: int) -> float:
        """son konum örneğinin yaşını saniye olarak döner"""
        if not self.valid:
            return math.inf
        return (now_monotonic_ns - self.updated_monotonic_ns) / 1e9

    @property
    def wind_sample_spread_s(self) -> float:
        """rüzgâr hesabındaki örneklerin zaman farkını döner"""
        stamps = (
            self.updated_monotonic_ns,
            self.twist_monotonic_ns,
            self.airspeed_monotonic_ns,
        )
        if min(stamps) <= 0:
            return math.inf
        return (max(stamps) - min(stamps)) / 1e9


class DdsTelemetry:
    """ap dds konularına abone olup son durumu saklar"""

    def __init__(self, node: Node) -> None:
        """telemetri aboneliklerini ve başlangıç değerlerini hazırlar"""
        self._lock = threading.Lock()
        self._position: Optional[LatLon] = None
        self._altitude_msl_m = 0.0
        self._velocity_east_mps = 0.0
        self._velocity_north_mps = 0.0
        self._airspeed_forward_mps = 0.0
        self._airspeed_left_mps = 0.0
        self._airspeed_up_mps = 0.0
        self._orientation_xyzw = (0.0, 0.0, 0.0, 1.0)
        self._updated_monotonic_ns = 0
        self._twist_monotonic_ns = 0
        self._airspeed_monotonic_ns = 0
        self.invalid_position_count = 0

        qos = QoSProfile(depth=QOS_DEPTH, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(GeoPoseStamped, GEOPOSE_TOPIC, self._on_geopose, qos)
        node.create_subscription(TwistStamped, TWIST_TOPIC, self._on_twist, qos)
        node.create_subscription(Vector3Stamped, AIRSPEED_TOPIC, self._on_airspeed, qos)

    def _on_geopose(self, msg: GeoPoseStamped) -> None:
        """geçerli konum ve yönelim bilgisini kaydeder"""
        position = msg.pose.position
        if not _is_valid_coordinate(position.latitude, position.longitude):
            self.invalid_position_count += 1
            return
        orientation = msg.pose.orientation
        with self._lock:
            self._position = LatLon(position.latitude, position.longitude)
            self._altitude_msl_m = position.altitude
            self._orientation_xyzw = (
                orientation.x, orientation.y, orientation.z, orientation.w
            )
            self._updated_monotonic_ns = time.monotonic_ns()

    def _on_twist(self, msg: TwistStamped) -> None:
        """doğu ve kuzey yönündeki yer hızını kaydeder"""
        linear = msg.twist.linear
        with self._lock:
            self._velocity_east_mps = linear.x
            self._velocity_north_mps = linear.y
            self._twist_monotonic_ns = time.monotonic_ns()

    def _on_airspeed(self, msg: Vector3Stamped) -> None:
        """gövde eksenlerindeki hava hızını kaydeder"""
        with self._lock:
            self._airspeed_forward_mps = msg.vector.x
            self._airspeed_left_mps = msg.vector.y
            self._airspeed_up_mps = msg.vector.z
            self._airspeed_monotonic_ns = time.monotonic_ns()

    def snapshot(self) -> TelemetrySnapshot:
        """kilit altında güncel telemetri kopyasını döner"""
        with self._lock:
            return TelemetrySnapshot(
                position=self._position,
                altitude_msl_m=self._altitude_msl_m,
                velocity_east_mps=self._velocity_east_mps,
                velocity_north_mps=self._velocity_north_mps,
                airspeed_forward_mps=self._airspeed_forward_mps,
                airspeed_left_mps=self._airspeed_left_mps,
                airspeed_up_mps=self._airspeed_up_mps,
                orientation_xyzw=self._orientation_xyzw,
                updated_monotonic_ns=self._updated_monotonic_ns,
                twist_monotonic_ns=self._twist_monotonic_ns,
                airspeed_monotonic_ns=self._airspeed_monotonic_ns,
            )
