"""ArduPlane ile MAVLink baglantisi: gorev yukleme, mod degisimi, arm, mesaj hizi.

AP_DDS 4.6.3 gorev yukleme servisi sunmadigi icin rota yuklemesi MAVLink
uzerinden yapilir. Telemetri okuma sorumlulugu bu modulde degildir.
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


@dataclass(frozen=True)
class MissionItem:
    """Tek bir MAVLink gorev ogesi. Irtifa daima MSL (MAV_FRAME_GLOBAL)."""

    command: int
    lat: float
    lon: float
    alt_msl: float
    param1: float = 0.0
    param2: float = 0.0


def connect(address: str, timeout_s: float = 30.0) -> mavutil.mavfile:
    """SITL'e baglanir ve ilk heartbeat'i bekler."""
    conn = mavutil.mavlink_connection(address)
    if conn.wait_heartbeat(timeout=timeout_s) is None:
        raise TimeoutError(f"Heartbeat alinamadi: {address}")
    logger.info("MAVLink baglandi: %s (sysid=%d)", address, conn.target_system)
    return conn


def set_message_interval(conn: mavutil.mavfile, message_id: int, hz: float) -> None:
    """Belirli bir mesajin yayin hizini ayarlar."""
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1e6 / hz, 0, 0, 0, 0, 0,
    )


def wait_gps_ready(conn: mavutil.mavfile, timeout_s: float = 120.0) -> bool:
    """3B GPS fix'i bekler. Fix olmadan arm denemesi anlamsizdir."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = conn.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
        if msg is not None and msg.fix_type >= 3:
            logger.info("GPS hazir (fix_type=%d, uydu=%d)", msg.fix_type, msg.satellites_visible)
            return True
    logger.error("GPS fix zaman asimina ugradi")
    return False


def upload_mission(conn: mavutil.mavfile, items: Sequence[MissionItem]) -> bool:
    """Gorevi siler ve yeni rotayi yukler. Oge 0 home olarak gonderilir."""
    conn.mav.mission_clear_all_send(conn.target_system, conn.target_component)
    conn.recv_match(type="MISSION_ACK", blocking=True, timeout=ACK_TIMEOUT_S)

    conn.mav.mission_count_send(conn.target_system, conn.target_component, len(items))

    remaining = set(range(len(items)))
    deadline = time.monotonic() + ACK_TIMEOUT_S * len(items)
    while remaining and time.monotonic() < deadline:
        req = conn.recv_match(
            type=["MISSION_REQUEST", "MISSION_REQUEST_INT"], blocking=True, timeout=ACK_TIMEOUT_S
        )
        if req is None:
            continue
        _send_item(conn, req.seq, items[req.seq])
        remaining.discard(req.seq)

    ack = conn.recv_match(type="MISSION_ACK", blocking=True, timeout=ACK_TIMEOUT_S)
    if ack is None or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        logger.error("Gorev yuklenemedi (ack=%s)", ack)
        return False
    logger.info("Gorev yuklendi: %d oge", len(items))
    return True


def _send_item(conn: mavutil.mavfile, seq: int, item: MissionItem) -> None:
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


def set_mode(conn: mavutil.mavfile, mode_name: str, timeout_s: float = ACK_TIMEOUT_S) -> bool:
    """Ucus modunu degistirir ve HEARTBEAT'ten uygulandigini dogrular."""
    mode_id = conn.mode_mapping().get(mode_name)
    if mode_id is None:
        logger.error("Bilinmeyen mod: %s", mode_name)
        return False

    conn.mav.set_mode_send(
        conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id
    )
    return wait_mode(conn, mode_name, timeout_s)


def wait_mode(conn: mavutil.mavfile, mode_name: str, timeout_s: float = ACK_TIMEOUT_S) -> bool:
    """Modun gercekten uygulandigini telemetriden dogrular."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb is not None and conn.flightmode == mode_name:
            logger.info("Mod: %s", mode_name)
            return True
    logger.error("Mod %s dogrulanamadi (mevcut: %s)", mode_name, conn.flightmode)
    return False


def arm(conn: mavutil.mavfile, timeout_s: float = 60.0) -> bool:
    """Prearm kontrolleri gecene kadar arm denemesini tekrarlar."""
    deadline = time.monotonic() + timeout_s
    last_reason = ""
    while time.monotonic() < deadline:
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0,
        )
        ack = _wait_command_ack(conn, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        if ack is not None and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            logger.info("Arm edildi")
            return True
        last_reason = "sonuc yok" if ack is None else f"result={ack.result}"
        time.sleep(ARM_RETRY_INTERVAL_S)

    logger.error("Arm edilemedi (%s)", last_reason)
    return False


def _wait_command_ack(conn: mavutil.mavfile, command: int) -> Optional[object]:
    deadline = time.monotonic() + ACK_TIMEOUT_S
    while time.monotonic() < deadline:
        ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if ack is not None and ack.command == command:
            return ack
    return None
