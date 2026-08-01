"""Gorev uretimi testleri."""
import pytest
from pymavlink import mavutil

from oasy_uav_agent.autopilot_adapter.mission_builder import (
    TARGET_WP_ACCEPT_RADIUS_M,
    build_mission,
)
from oasy_uav_agent.estimation.geodesy import LatLon

HOME = LatLon(47.530002, -122.302457)
ROUTE = [
    LatLon(47.556939, -122.295004),
    LatLon(47.543977, -122.240829),
    LatLon(47.535683, -122.228584),
]
CRUISE_ALT = 400.0
TAKEOFF_ALT = 100.0
WP_RADIUS = 120.0


def make() -> list:
    return build_mission(HOME, ROUTE, CRUISE_ALT, TAKEOFF_ALT, WP_RADIUS)


def test_oge_sayisi():
    # home + kalkis + rota noktalari
    assert len(make()) == 2 + len(ROUTE)


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
    ara = make()[2:-1]
    assert ara, "en az bir ara nokta olmali"
    for item in ara:
        assert item.param2 == WP_RADIUS


def test_hedef_dar_yaricap():
    """Genis kabul yaricapi aracin 5 m'ye yaklasmasini engellerdi."""
    assert make()[-1].param2 == TARGET_WP_ACCEPT_RADIUS_M
    assert TARGET_WP_ACCEPT_RADIUS_M < WP_RADIUS


def test_bos_rota_reddedilir():
    with pytest.raises(ValueError):
        build_mission(HOME, [], CRUISE_ALT, TAKEOFF_ALT, WP_RADIUS)
