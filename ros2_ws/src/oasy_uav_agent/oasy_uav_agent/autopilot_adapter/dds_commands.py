"""GUIDED modda konum hedefi gonderimi (AP_DDS /ap/cmd_gps_pose).

S-manevrasi ve hedefin disindaki son yasal bekleme kapisinda kullanilir.
ArduPlane GUIDED'da verilen noktaya gidip etrafinda cember atar; bu yuzden
ortak hedefin kendisi buradan komut edilmez. Son yaklasma AUTO gorevine
birakilir, aksi halde arac
hedefin etrafinda donerek 5 m kabul yaricapina hic giremez ve dokumandaki
2 km loiter yasagi ihlal edilir.
"""
from __future__ import annotations

from ardupilot_msgs.msg import GlobalPosition
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ..estimation.geodesy import LatLon

CMD_GPS_POSE_TOPIC = "/ap/cmd_gps_pose"
QOS_DEPTH = 10
# AP_DDS_ExternalControl::handle_global_position_control ilk is olarak
# header.frame_id'yi "map" ile karsilastirir; esitlemezse hedefi hic
# ayarlamadan false doner. Bos birakildiginda arac GUIDED'a geciyor ama
# komut edilen noktayi almiyor ve bulundugu yerde cember atiyor.
MAP_FRAME_ID = "map"

# Yalnizca konum komut ediliyor; hiz, ivme ve yonelim alanlari yok sayilir.
_IGNORE_MASK = (
    GlobalPosition.IGNORE_VX | GlobalPosition.IGNORE_VY | GlobalPosition.IGNORE_VZ
    | GlobalPosition.IGNORE_AFX | GlobalPosition.IGNORE_AFY | GlobalPosition.IGNORE_AFZ
    | GlobalPosition.IGNORE_YAW | GlobalPosition.IGNORE_YAW_RATE
)


class GuidedPositionCommander:
    """Arac domain'inde GUIDED konum hedefi yayinlar."""

    def __init__(self, node: Node) -> None:
        self._node = node
        qos = QoSProfile(depth=QOS_DEPTH, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._publisher = node.create_publisher(GlobalPosition, CMD_GPS_POSE_TOPIC, qos)

    def send(self, position: LatLon, altitude_msl_m: float) -> None:
        msg = GlobalPosition()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = MAP_FRAME_ID
        msg.coordinate_frame = GlobalPosition.FRAME_GLOBAL_INT
        msg.type_mask = _IGNORE_MASK
        msg.latitude = position.lat
        msg.longitude = position.lon
        msg.altitude = float(altitude_msl_m)
        self._publisher.publish(msg)
