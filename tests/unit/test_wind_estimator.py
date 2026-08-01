"""Ruzgar kestirimi ve ruzgar altinda rota suresi testleri."""
import math

import pytest

from oasy_uav_agent.estimation.geodesy import LatLon
from oasy_uav_agent.estimation.wind_estimator import (
    WindEstimate,
    estimate_wind,
    route_duration_with_wind_s,
)

HA1_HOME = LatLon(47.530002, -122.302457)
HA1_ROUTE = [
    LatLon(47.556939, -122.295004),
    LatLon(47.565332, -122.265617),
    LatLon(47.571248, -122.245860),
    LatLon(47.564550, -122.230073),
    LatLon(47.543977, -122.240829),
    LatLon(47.535683, -122.228584),
]
HA1_ROUTE_LENGTH_M = 12205
CRUISE_MPS = 22.9

RUZGARSIZ = WindEstimate(0.0, 0.0, 1)


KUZEYE_YAW = math.pi / 2  # ENU'da yaw dogudan olculur; kuzey = 90 derece


def test_ruzgarsiz_kestirim_sifir():
    # Kuzeye 20 m/s yer hizi, burun kuzeyde, 20 m/s ileri hava hizi.
    tahmin = estimate_wind((0.0, 20.0), (20.0, 0.0), KUZEYE_YAW)
    assert tahmin.speed_mps == pytest.approx(0.0, abs=1e-6)


def test_karsi_ruzgar_kestirimi():
    """Kuzeye giderken yer hizi hava hizindan dusukse karsi ruzgar var."""
    tahmin = estimate_wind((0.0, 15.0), (23.0, 0.0), KUZEYE_YAW)
    assert tahmin.north_mps == pytest.approx(-8.0, abs=1e-6)
    assert tahmin.speed_mps == pytest.approx(8.0, abs=1e-6)
    # Ruzgar kuzeyden geliyor.
    assert tahmin.from_direction_deg == pytest.approx(0.0, abs=0.5)


def test_arka_ruzgar_kestirimi():
    tahmin = estimate_wind((0.0, 31.0), (23.0, 0.0), KUZEYE_YAW)
    assert tahmin.north_mps == pytest.approx(8.0, abs=1e-6)
    assert tahmin.from_direction_deg == pytest.approx(180.0, abs=0.5)


def test_yan_ruzgar_crab_ile_dogru_kestirilir():
    """Burun ruzgara kirmisken vektor farki ruzgari tam verir.

    Arac kuzeye ilerliyor ama burnu 20 derece doguya kirmis; hava hizi
    vektoru burun dogrultusunda. Fark, dogudan esen yan ruzgari vermeli.
    """
    yaw = math.radians(70.0)  # ENU: doguya 20 derece kirilmis kuzey
    airspeed = 23.0
    tahmin = estimate_wind((0.0, 20.0), (airspeed, 0.0), yaw)
    assert tahmin.east_mps == pytest.approx(-airspeed * math.cos(yaw), abs=1e-6)
    assert tahmin.from_direction_deg == pytest.approx(90.0, abs=15.0)


def test_govde_enu_donusumu():
    from oasy_uav_agent.estimation.wind_estimator import body_to_enu

    # Burun doguda (yaw=0): ileri = dogu, sol = kuzey.
    assert body_to_enu(10.0, 0.0, 0.0)[0] == pytest.approx(10.0)
    assert body_to_enu(0.0, 10.0, 0.0)[1] == pytest.approx(10.0)
    # Burun kuzeyde (yaw=90): ileri = kuzey.
    east, north = body_to_enu(10.0, 0.0, math.pi / 2)
    assert east == pytest.approx(0.0, abs=1e-9)
    assert north == pytest.approx(10.0)


def test_gecersiz_girdi_reddedilir():
    assert estimate_wind((0.0, 20.0), (0.0, 0.0), KUZEYE_YAW) is None


def test_ruzgarsiz_rota_suresi_bilinen_degere_esit():
    sure = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, RUZGARSIZ)
    assert sure == pytest.approx(HA1_ROUTE_LENGTH_M / CRUISE_MPS, rel=0.01)


def test_ruzgar_rota_suresini_uzatir():
    """Kapali rota olmadigindan net etki genelde sureyi uzatir."""
    ruzgarli = WindEstimate(0.0, -8.0, 1)
    ruzgarsiz_s = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, RUZGARSIZ)
    ruzgarli_s = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, ruzgarli)
    assert ruzgarli_s > ruzgarsiz_s


def test_yan_ruzgar_da_sureyi_uzatir():
    """Crab acisi ileri bileseni azalttigi icin yan ruzgar bile yavaslatir."""
    tek_bacak = [LatLon(HA1_HOME.lat + 0.05, HA1_HOME.lon)]  # tam kuzey
    yan = WindEstimate(8.0, 0.0, 1)
    ruzgarsiz_s = route_duration_with_wind_s(HA1_HOME, tek_bacak, CRUISE_MPS, RUZGARSIZ)
    yan_s = route_duration_with_wind_s(HA1_HOME, tek_bacak, CRUISE_MPS, yan)
    assert yan_s > ruzgarsiz_s
    # 8 m/s yan ruzgarda ileri bilesen sqrt(22.9^2 - 8^2) = 21.4 m/s.
    assert yan_s == pytest.approx(ruzgarsiz_s * CRUISE_MPS / 21.45, rel=0.01)


def test_tam_karsi_ruzgarda_tek_bacak_suresi():
    kuzeye = [LatLon(HA1_HOME.lat + 0.05, HA1_HOME.lon)]
    karsi = WindEstimate(0.0, -8.0, 1)
    sure = route_duration_with_wind_s(HA1_HOME, kuzeye, CRUISE_MPS, karsi)
    ruzgarsiz_s = route_duration_with_wind_s(HA1_HOME, kuzeye, CRUISE_MPS, RUZGARSIZ)
    # Yer hizi 22.9 - 8 = 14.9 m/s.
    assert sure == pytest.approx(ruzgarsiz_s * CRUISE_MPS / 14.9, rel=0.01)


def test_asiri_ruzgarda_sure_sonsuza_gitmez():
    firtina = WindEstimate(0.0, -100.0, 1)
    sure = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, firtina)
    assert math.isfinite(sure)


def test_sifir_hava_hizi_reddedilir():
    with pytest.raises(ValueError):
        route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, 0.0, RUZGARSIZ)


def test_ha1_rotasinda_8ms_kuzey_ruzgarinin_etkisi():
    """Olculen senaryo: 8 m/s kuzey ruzgari HA-1 rotasini ne kadar uzatir."""
    karsi = WindEstimate(0.0, -8.0, 1)
    ruzgarsiz_s = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, RUZGARSIZ)
    ruzgarli_s = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, karsi)
    # Rota cok yonlu oldugu icin net etki tek bacaktakinden cok daha kucuk.
    assert 0.0 < ruzgarli_s - ruzgarsiz_s < 120.0
