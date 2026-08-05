"""guided konum hedefini ap dds üzerinden gönderir"""
from __future__ import annotations

from ardupilot_msgs.msg import GlobalPosition
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ..estimation.geodesy import LatLon

CMD_GPS_POSE_TOPIC = "/ap/cmd_gps_pose"  # guided konum komutu konusu
QOS_DEPTH = 10  # yayın kuyruğu derinliği
MAP_FRAME_ID = "map"  # ap dds konum komutunun beklediği koordinat çerçevesi

_IGNORE_MASK = (  # konum dışındaki guided alanlarını yok sayan bit maskesi
    GlobalPosition.IGNORE_VX | GlobalPosition.IGNORE_VY | GlobalPosition.IGNORE_VZ
    | GlobalPosition.IGNORE_AFX | GlobalPosition.IGNORE_AFY | GlobalPosition.IGNORE_AFZ
    | GlobalPosition.IGNORE_YAW | GlobalPosition.IGNORE_YAW_RATE
)


class GuidedPositionCommander:
    """araç alanında guided konum hedefi yayınlar"""

    def __init__(self, node: Node) -> None:
        """guided konum komutu yayıncısını hazırlar"""
        self._node = node
        qos = QoSProfile(depth=QOS_DEPTH, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._publisher = node.create_publisher(GlobalPosition, CMD_GPS_POSE_TOPIC, qos)

    def send(self, position: LatLon, altitude_msl_m: float) -> None:
        """verilen msl irtifasına küresel konum hedefi gönderir"""
        msg = GlobalPosition()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = MAP_FRAME_ID
        msg.coordinate_frame = GlobalPosition.FRAME_GLOBAL_INT
        msg.type_mask = _IGNORE_MASK
        msg.latitude = position.lat
        msg.longitude = position.lon
        msg.altitude = float(altitude_msl_m)
        self._publisher.publish(msg)
