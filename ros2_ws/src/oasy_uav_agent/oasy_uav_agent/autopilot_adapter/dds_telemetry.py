"""AP_DDS telemetri abonelikleri ve son gecerli durumun tutulmasi.

Abonelik geri cagrilari arac executor'unda, okuyucu ise koordinasyon
executor'unda calistigi icin paylasilan durum kilit altinda tutulur.
"""
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

GEOPOSE_TOPIC = "/ap/geopose/filtered"
TWIST_TOPIC = "/ap/twist/filtered"
AIRSPEED_TOPIC = "/ap/airspeed"
QOS_DEPTH = 10


# EKF origin kurulmadan once geopose sifir koordinat yayinliyor. Gorev
# alani bu noktadan binlerce kilometre uzakta oldugu icin sifira yakin
# konumlar gecersiz sayilir.
MIN_VALID_COORDINATE_DEG = 1e-6


def _is_valid_coordinate(latitude: float, longitude: float) -> bool:
    return (
        abs(latitude) > MIN_VALID_COORDINATE_DEG
        and abs(longitude) > MIN_VALID_COORDINATE_DEG
    )


@dataclass(frozen=True)
class TelemetrySnapshot:
    """Belirli bir andaki arac durumu. position None ise henuz veri gelmedi."""

    position: Optional[LatLon]
    altitude_msl_m: float
    # ENU bileseni; ETA rota dogrultusundaki izdusumu icin vektore ihtiyac duyar.
    velocity_east_mps: float
    velocity_north_mps: float
    # Govde cercevesinde (FLU) gercek hava hizi vektoru ve aracin tam
    # yonelimi; ruzgar kestirimi vektoru yonelimle ENU'ya cevirip yer
    # hizindan cikarir. Yalnizca yaw yetmez: EKF vektoru tam yonelimle
    # govdeye dondurdugu icin tirmanista pitch, donuslerde roll hata birakir.
    airspeed_forward_mps: float
    airspeed_left_mps: float
    airspeed_up_mps: float
    orientation_xyzw: Tuple[float, float, float, float]
    updated_monotonic_ns: int
    # Yer hizi ve hava hizi ayri konularda yayinlandigi icin kendi
    # damgalarini tasirlar; ruzgar kestirimi ucunun es zamanli olmasini ister.
    twist_monotonic_ns: int
    airspeed_monotonic_ns: int

    @property
    def groundspeed_mps(self) -> float:
        return math.hypot(self.velocity_east_mps, self.velocity_north_mps)

    @property
    def airspeed_mps(self) -> float:
        return math.sqrt(
            self.airspeed_forward_mps ** 2
            + self.airspeed_left_mps ** 2
            + self.airspeed_up_mps ** 2
        )

    @property
    def valid(self) -> bool:
        return self.position is not None

    def age_s(self, now_monotonic_ns: int) -> float:
        if not self.valid:
            return math.inf
        return (now_monotonic_ns - self.updated_monotonic_ns) / 1e9

    @property
    def wind_sample_spread_s(self) -> float:
        """Ruzgar kestiriminde kullanilan uc ornegin zaman yayilimi.

        snapshot() her konunun son degerini birlestirir; ornekler farkli
        anlara aitse yer hizi ile hava hizi vektorlerinin farki gercek
        ruzgari vermez. Ucu de 33 ms'de bir yayinlandigi icin normalde
        yayilim bir periyodu asmaz.
        """
        stamps = (
            self.updated_monotonic_ns,
            self.twist_monotonic_ns,
            self.airspeed_monotonic_ns,
        )
        if min(stamps) <= 0:
            return math.inf
        return (max(stamps) - min(stamps)) / 1e9


class DdsTelemetry:
    """AP_DDS konularina abone olur ve son durumu saklar."""

    def __init__(self, node: Node) -> None:
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

        # AP_DDS yayinci QoS'u garanti edilmedigi icin abone tarafi
        # her iki durumla da uyumlu olan BEST_EFFORT secilir.
        qos = QoSProfile(depth=QOS_DEPTH, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(GeoPoseStamped, GEOPOSE_TOPIC, self._on_geopose, qos)
        node.create_subscription(TwistStamped, TWIST_TOPIC, self._on_twist, qos)
        node.create_subscription(Vector3Stamped, AIRSPEED_TOPIC, self._on_airspeed, qos)

    def _on_geopose(self, msg: GeoPoseStamped) -> None:
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
        linear = msg.twist.linear
        with self._lock:
            self._velocity_east_mps = linear.x
            self._velocity_north_mps = linear.y
            self._twist_monotonic_ns = time.monotonic_ns()

    def _on_airspeed(self, msg: Vector3Stamped) -> None:
        with self._lock:
            self._airspeed_forward_mps = msg.vector.x
            self._airspeed_left_mps = msg.vector.y
            self._airspeed_up_mps = msg.vector.z
            self._airspeed_monotonic_ns = time.monotonic_ns()

    def snapshot(self) -> TelemetrySnapshot:
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
