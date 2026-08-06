"""arduplane görev öğelerinin üretimini sınar"""
import pytest
from pymavlink import mavutil

from oasy_uav_agent.autopilot_adapter.mission_builder import (
    TARGET_WP_ACCEPT_RADIUS_M,
    build_mission,
)
from oasy_uav_agent.estimation.geodesy import LatLon

HOME = LatLon(47.530002, -122.302457)  # test başlangıç konumu
ROUTE = [
    LatLon(47.556939, -122.295004),
    LatLon(47.543977, -122.240829),
    LatLon(47.535683, -122.228584),
]  # test rotası

CRUISE_ALT = 400.0  # seyir irtifası
TAKEOFF_ALT = 100.0  # kalkış görevi irtifası
WP_RADIUS = 120.0  # ara rota noktası yarıçapı


def make() -> list:
    """sabit test değerleriyle bir arduplane görevi oluşturur"""
    return build_mission(HOME, ROUTE, CRUISE_ALT, TAKEOFF_ALT, WP_RADIUS)


def test_oge_sayisi():
    """görev öğesi sayısının home kalkış ve rota toplamına uymasını sınar"""
    # ev kalkış ve rota noktaları
    assert len(make()) == 2 + len(ROUTE)


def test_ilk_oge_home():
    """ilk görev öğesinin home konumunda waypoint olmasını sınar"""
    item = make()[0]
    assert item.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    assert item.lat == pytest.approx(HOME.lat)
    assert item.lon == pytest.approx(HOME.lon)


def test_ikinci_oge_kalkis():
    """ikinci görev öğesinin kalkış komutu olmasını sınar"""
    item = make()[1]
    assert item.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    assert item.alt_msl == TAKEOFF_ALT


def test_rota_noktalari_seyir_irtifasinda():
    """rota noktalarının seyir irtifasında oluşturulmasını sınar"""
    for item in make()[2:]:
        assert item.alt_msl == CRUISE_ALT


def test_son_oge_hedef():
    """son görev öğesinin ortak hedefte olmasını sınar"""
    item = make()[-1]
    assert item.lat == pytest.approx(ROUTE[-1].lat)
    assert item.lon == pytest.approx(ROUTE[-1].lon)


def test_ara_noktalar_konfigurasyon_yaricapi():
    """ara rota noktalarının yapılandırılan yarıçapı kullanmasını sınar"""
    ara = make()[2:-1]
    assert ara, "en az bir ara nokta olmali"
    for item in ara:
        assert item.param2 == WP_RADIUS


def test_hedef_dar_yaricap():
    """hedef noktasında beş metre yarıçap kullanılmasını sınar"""
    assert make()[-1].param2 == TARGET_WP_ACCEPT_RADIUS_M
    assert TARGET_WP_ACCEPT_RADIUS_M < WP_RADIUS


def test_bos_rota_reddedilir():
    """boş rota ile görev oluşturulamamasını sınar"""
    with pytest.raises(ValueError):
        build_mission(HOME, [], CRUISE_ALT, TAKEOFF_ALT, WP_RADIUS)
