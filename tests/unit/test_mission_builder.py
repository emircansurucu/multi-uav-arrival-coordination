"""Gorev uretimi testleri."""
import pytest
from pymavlink import mavutil

from oasy_uav_agent.autopilot_adapter.mission_builder import (
    S_SLOT_ACCEPT_RADIUS_M,
    S_SLOT_COUNT,
    spare_slot_range,
    TARGET_WP_ACCEPT_RADIUS_M,
    build_mission,
)
from oasy_uav_agent.estimation.geodesy import LatLon, to_local_xy

HOME = LatLon(47.530002, -122.302457)
ROUTE = [
    LatLon(47.556939, -122.295004),
    LatLon(47.543977, -122.240829),
    LatLon(47.535683, -122.228584),
]
def perpendicular_distance_m(point, start, end):
    """Noktanin bacak dogrusuna dik uzakligi."""
    import math

    sx, sy = to_local_xy(start, start)
    ex, ey = to_local_xy(end, start)
    px, py = to_local_xy(point, start)
    dx, dy = ex - sx, ey - sy
    t = ((px - sx) * dx + (py - sy) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (sx + t * dx), py - (sy + t * dy))


CRUISE_ALT = 400.0
TAKEOFF_ALT = 100.0
WP_RADIUS = 120.0


def make(s_maneuver_enabled: bool = False) -> list:
    return build_mission(
        HOME, ROUTE, CRUISE_ALT, TAKEOFF_ALT, WP_RADIUS, s_maneuver_enabled
    )


def test_oge_sayisi():
    # home + kalkis + rota noktalari
    assert len(make()) == 2 + len(ROUTE)


def test_manevra_kapaliyken_yedek_yuva_eklenmez():
    """Kapali ozellik ucus kritik gorev listesinde oge tasimamali.

    Yuvalarin konumu bir kez hatali hesaplanip hedefin uzerine dusmustu;
    hedefin 5 m'lik dar kabul yaricapini 20 m'lik yuvayla golgeleyecekti.
    Var olmayan yuva bozulamaz.
    """
    kapali = make()
    acik = make(s_maneuver_enabled=True)
    assert len(acik) - len(kapali) == S_SLOT_COUNT
    for item in kapali:
        assert item.param2 != S_SLOT_ACCEPT_RADIUS_M


def test_ilk_oge_home():
    item = make()[0]
    assert item.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    assert item.lat == pytest.approx(HOME.lat)
    assert item.lon == pytest.approx(HOME.lon)


def test_ikinci_oge_kalkis():
    item = make()[1]
    assert item.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    assert item.alt_msl == TAKEOFF_ALT


def test_rota_noktalari_seyir_irtifasinda():
    for item in make()[2:]:
        assert item.alt_msl == CRUISE_ALT


def test_son_oge_hedef():
    item = make()[-1]
    assert item.lat == pytest.approx(ROUTE[-1].lat)
    assert item.lon == pytest.approx(ROUTE[-1].lon)


def test_ara_noktalar_konfigurasyon_yaricapi():
    """Rota noktalari genis yaricapi kullanir; yuvalar bunun disindadir."""
    ilk_yuva, _ = spare_slot_range(ROUTE)
    ara = make()[2:ilk_yuva]
    assert ara, "en az bir ara nokta olmali"
    for item in ara:
        assert item.param2 == WP_RADIUS


def test_yedek_yuvalar_bacak_dogrusunda_ve_dar_yaricapli():
    """Baslangicta yuvalar rotayi degistirmemeli.

    S-manevrasi yazilana kadar bacak dogrusu uzerinde dururlar; yaricaplari
    dar tutulur cunku S noktalarina donustuklerinde 200-400 m arayla dizilir
    ve genis yaricap kosegen kesmeye yol acar.
    """
    items = make(s_maneuver_enabled=True)
    ilk, son = spare_slot_range(ROUTE)
    for item in items[ilk:son + 1]:
        assert item.param2 == S_SLOT_ACCEPT_RADIUS_M
        sapma = perpendicular_distance_m(
            LatLon(item.lat, item.lon), ROUTE[-2], ROUTE[-1]
        )
        assert sapma < 1.0


def test_hedef_dar_yaricap():
    """Genis kabul yaricapi aracin 5 m'ye yaklasmasini engellerdi."""
    assert make()[-1].param2 == TARGET_WP_ACCEPT_RADIUS_M
    assert TARGET_WP_ACCEPT_RADIUS_M < WP_RADIUS


def test_bos_rota_reddedilir():
    with pytest.raises(ValueError):
        build_mission(HOME, [], CRUISE_ALT, TAKEOFF_ALT, WP_RADIUS)
