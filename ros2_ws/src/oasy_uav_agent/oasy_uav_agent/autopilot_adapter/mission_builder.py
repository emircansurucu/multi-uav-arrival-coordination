"""Rota noktalarindan ArduPlane gorev listesi uretimi.

Irtifalar MAV_FRAME_GLOBAL ile MSL olarak yazilir; boylece home irtifasina
bagli relatif donusum gerekmez ve dokumandaki 400 m MSL sarti dogrudan
gorev dosyasinda gorunur.
"""
from __future__ import annotations

from typing import List, Sequence

from pymavlink import mavutil

from ..estimation.geodesy import LatLon, from_local_xy, to_local_xy
from .mavlink_link import MissionItem

TAKEOFF_PITCH_DEG = 15.0
# Hedef ogesinde genis kabul yaricapi kullanilirsa otopilot arac 5 m'ye
# yaklasmadan gorevi tamamlamis sayar. Varis karari bu degere bagli degildir,
# ama aracin hedefin uzerinden gecmesi icin dar tutulur.
TARGET_WP_ACCEPT_RADIUS_M = 5.0

# Son bacaga onceden yerlestirilen bos yuvalar. Baslangicta bacak dogrusu
# uzerinde durduklari icin rotayi degistirmezler; S-manevrasi gerektiginde
# MISSION_WRITE_PARTIAL_LIST ile yanal ofsetli konumlarla uzerlerine yazilir.
# Boylece gorev silinmeden, mod degistirmeden ve oge sayisi degismeden
# yorunge uzatilabilir. GUIDED bu isi yapamiyor: ModeGuided::navigate
# update_loiter cagirir ve set_guided_WP crosstrack'i kapatir, yani yol
# takibi yoktur; iki denemede de arac noktalarda takilmisti.
S_SLOT_COUNT = 6
# Yuvalar S noktasina donustugunde ~200-400 m aralikli olur. Rota geneli icin
# kullanilan genis yaricap burada kosegen kesmeye yol acar; ArduPlane waypoint
# basina yaricapi param2'den okudugu icin (AP_Mission.cpp: acp = packet.param2)
# yalnizca bu yuvalar dar tutulur.
S_SLOT_ACCEPT_RADIUS_M = 20.0


def build_mission(
    home: LatLon,
    route: Sequence[LatLon],
    cruise_alt_msl_m: float,
    takeoff_alt_msl_m: float,
    wp_accept_radius_m: float,
    s_maneuver_enabled: bool = False,
) -> List[MissionItem]:
    """Home + kalkis + rota noktalarindan gorev ogelerini uretir.

    Yedek S yuvalari yalnizca manevra aciksa eklenir. Kapali ozellik icin
    ucus kritik gorev listesinde oge tasimak gereksiz risktir: yuvalarin
    konumu bir kez hatali hesaplanip hedefin uzerine dusmustu ve hedefin
    5 m'lik dar kabul yaricapini golgeleyecekti.
    """
    if len(route) < 1:
        raise ValueError("rota en az bir nokta icermeli")

    items = [MissionItem(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, home.lat, home.lon, 0.0)]
    items.append(
        MissionItem(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0.0, 0.0, takeoff_alt_msl_m,
            param1=TAKEOFF_PITCH_DEG,
        )
    )

    last_index = len(route) - 1
    for index, point in enumerate(route):
        # Son bacagin basinda yuvalar acilir; hedeften onceki bu bolge
        # dokumanin loiter yasakladigi ama S-manevrasina izin verdigi yer.
        if s_maneuver_enabled and index == last_index and last_index > 0:
            items.extend(
                _spare_slots(route[last_index - 1], point, cruise_alt_msl_m)
            )
        radius_m = TARGET_WP_ACCEPT_RADIUS_M if index == last_index else wp_accept_radius_m
        items.append(
            MissionItem(
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                point.lat, point.lon, cruise_alt_msl_m,
                param2=radius_m,
            )
        )
    return items


def spare_slot_range(route: Sequence[LatLon]) -> tuple:
    """Yedek yuvalarin gorev listesindeki (ilk, son) indeksi.

    Home ve kalkis ogeleri rotadan once geldigi icin ofset ikidir. Yalnizca
    s_maneuver_enabled ile uretilmis gorevler icin anlamlidir.
    """
    if len(route) < 2:
        raise ValueError("yedek yuva icin en az iki rota noktasi gerekir")
    first = 2 + (len(route) - 1)
    return first, first + S_SLOT_COUNT - 1


def _spare_slots(
    leg_start: LatLon, leg_end: LatLon, cruise_alt_msl_m: float
) -> List[MissionItem]:
    """Bacak dogrusu uzerinde esit araliklarla duran bos yuvalar."""
    slots = []
    for index in range(S_SLOT_COUNT):
        nokta = point_along_leg(leg_start, leg_end, (index + 1) / (S_SLOT_COUNT + 1))
        slots.append(
            MissionItem(
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                nokta.lat, nokta.lon, cruise_alt_msl_m,
                param2=S_SLOT_ACCEPT_RADIUS_M,
            )
        )
    return slots


def point_along_leg(start: LatLon, end: LatLon, fraction: float) -> LatLon:
    """Bacak uzerinde verilen orandaki nokta."""
    start_xy = to_local_xy(start, start)
    end_xy = to_local_xy(end, start)
    return from_local_xy(
        start_xy[0] + fraction * (end_xy[0] - start_xy[0]),
        start_xy[1] + fraction * (end_xy[1] - start_xy[1]),
        start,
    )
