"""ArduPlane ile MAVLink komut baglantisi.

AP_DDS 4.6.3 gorev yukleme servisi sunmadigi ve airspeed komutunun DDS
karsiligi olmadigi icin komutlar MAVLink uzerinden gider. Telemetri okuma
bu modulun sorumlulugunda degildir; o tamamen AP_DDS uzerinden yurur.

Cagrilar bloklayicidir ve gorev yoneticisinin kendi thread'inde calisir;
rclpy executor'larindan cagrilmamalidir.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Optional, Sequence

from pymavlink import mavutil

logger = logging.getLogger(__name__)

ACK_TIMEOUT_S = 5.0
ARM_RETRY_INTERVAL_S = 2.0
DEFAULT_CONNECT_TIMEOUT_S = 30.0


@dataclass(frozen=True)
class MissionItem:
    """Tek bir MAVLink gorev ogesi. Irtifa daima MSL (MAV_FRAME_GLOBAL)."""

    command: int
    lat: float
    lon: float
    alt_msl: float
    param1: float = 0.0
    param2: float = 0.0


class MavlinkCommander:
    """Otopilota komut gonderir ve uygulandigini telemetriden dogrular."""

    def __init__(self, address: str) -> None:
        self._address = address
        self._conn: Optional[mavutil.mavfile] = None

    @property
    def connected(self) -> bool:
        return self._conn is not None

    @property
    def flight_mode(self) -> str:
        return self._conn.flightmode if self._conn is not None else ""

    def connect(self, timeout_s: float = DEFAULT_CONNECT_TIMEOUT_S) -> bool:
        try:
            conn = mavutil.mavlink_connection(self._address)
        except OSError as exc:
            logger.error("MAVLink baglantisi acilamadi (%s): %s", self._address, exc)
            return False

        if conn.wait_heartbeat(timeout=timeout_s) is None:
            logger.error("Heartbeat alinamadi: %s", self._address)
            return False

        self._conn = conn
        logger.info("MAVLink baglandi: %s (sysid=%d)", self._address, conn.target_system)
        return True

    def drain(self, max_messages: int = 200) -> int:
        """Alim tamponunu bosaltir ve okunan mesaj sayisini doner.

        Bu duzenli olarak cagrilmazsa SITL'in TCP tamponu doluyor, ana dongu
        tikaniyor ve AP_DDS yayini tamamen duruyor. Belirti, telemetri
        yasinin surekli buyumesidir.
        """
        if self._conn is None:
            return 0
        count = 0
        while count < max_messages and self._conn.recv_match(blocking=False) is not None:
            count += 1
        return count

    def wait_gps_ready(self, timeout_s: float = 120.0) -> bool:
        """3B GPS fix'i bekler. Fix olmadan arm denemesi anlamsizdir."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            msg = self._conn.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
            if msg is not None and msg.fix_type >= 3:
                logger.info("GPS hazir (fix_type=%d, uydu=%d)", msg.fix_type, msg.satellites_visible)
                return True
        logger.error("GPS fix zaman asimina ugradi")
        return False

    def set_message_interval(self, message_id: int, hz: float) -> None:
        self._conn.mav.command_long_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, 1e6 / hz, 0, 0, 0, 0, 0,
        )

    def upload_mission(self, items: Sequence[MissionItem]) -> bool:
        """Gorevi siler ve yeni rotayi yukler. Oge 0 home olarak gonderilir."""
        conn = self._conn
        conn.mav.mission_clear_all_send(conn.target_system, conn.target_component)
        conn.recv_match(type="MISSION_ACK", blocking=True, timeout=ACK_TIMEOUT_S)

        conn.mav.mission_count_send(conn.target_system, conn.target_component, len(items))

        remaining = set(range(len(items)))
        deadline = time.monotonic() + ACK_TIMEOUT_S * len(items)
        while remaining and time.monotonic() < deadline:
            request = conn.recv_match(
                type=["MISSION_REQUEST", "MISSION_REQUEST_INT"],
                blocking=True, timeout=ACK_TIMEOUT_S,
            )
            if request is None:
                continue
            self._send_item(request.seq, items[request.seq])
            remaining.discard(request.seq)

        ack = conn.recv_match(type="MISSION_ACK", blocking=True, timeout=ACK_TIMEOUT_S)
        if ack is None or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            logger.error("Gorev yuklenemedi (ack=%s)", ack)
            return False
        logger.info("Gorev yuklendi: %d oge", len(items))
        return True

    def _send_item(self, seq: int, item: MissionItem) -> None:
        conn = self._conn
        conn.mav.mission_item_int_send(
            conn.target_system, conn.target_component,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            item.command,
            1 if seq == 0 else 0,  # current
            1,                     # autocontinue
            item.param1, item.param2, 0.0, 0.0,
            int(round(item.lat * 1e7)), int(round(item.lon * 1e7)), item.alt_msl,
        )

    def set_mode(self, mode_name: str, timeout_s: float = ACK_TIMEOUT_S) -> bool:
        """Ucus modunu degistirir ve HEARTBEAT'ten uygulandigini dogrular."""
        mode_id = self._conn.mode_mapping().get(mode_name)
        if mode_id is None:
            logger.error("Bilinmeyen mod: %s", mode_name)
            return False

        self._conn.mav.set_mode_send(
            self._conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        return self.wait_mode(mode_name, timeout_s)

    def wait_mode(self, mode_name: str, timeout_s: float = ACK_TIMEOUT_S) -> bool:
        """Modun gercekten uygulandigini telemetriden dogrular."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            heartbeat = self._conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if heartbeat is not None and self._conn.flightmode == mode_name:
                logger.info("Mod: %s", mode_name)
                return True
        logger.error("Mod %s dogrulanamadi (mevcut: %s)", mode_name, self._conn.flightmode)
        return False

    def arm(self, timeout_s: float = 60.0) -> bool:
        """Prearm kontrolleri gecene kadar arm denemesini tekrarlar."""
        deadline = time.monotonic() + timeout_s
        last_reason = "sonuc yok"
        while time.monotonic() < deadline:
            self._conn.mav.command_long_send(
                self._conn.target_system, self._conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0,
            )
            ack = self._wait_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
            if ack is not None and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                logger.info("Arm edildi")
                return True
            last_reason = "sonuc yok" if ack is None else f"result={ack.result}"
            time.sleep(ARM_RETRY_INTERVAL_S)

        logger.error("Arm edilemedi (%s)", last_reason)
        return False

    def set_airspeed(self, airspeed_mps: float) -> None:
        """AUTO modunda hedef hava hizini degistirir (DO_CHANGE_SPEED).

        Kalici parametre yazmak yerine komut kullanilir; boylece eeprom
        durumu kosular arasinda degismez.
        """
        self._conn.mav.command_long_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
            0,               # airspeed
            airspeed_mps,
            -1,              # throttle degismesin
            0, 0, 0, 0,
        )

    def _wait_command_ack(self, command: int):
        deadline = time.monotonic() + ACK_TIMEOUT_S
        while time.monotonic() < deadline:
            ack = self._conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
            if ack is not None and ack.command == command:
                return ack
        return None
