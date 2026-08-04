"""Jeodezik yardimcilar ve hedef cemberi kesisim tespiti testleri."""
import math

import pytest

from oasy_uav_agent.estimation.geodesy import (
    LatLon,
    circle_entry_fraction,
    cross_track_distance_m,
    geodesic_distance_m,
    initial_bearing_deg,
    last_circle_entry_on_route,
    to_local_xy,
)

TARGET = LatLon(47.535683, -122.228584)
HA1_HOME = LatLon(47.530002, -122.302457)


def test_bilinen_mesafe():
    # Vaka dokumani Tablo 1: HA-1 kalkis noktasindan hedefe duz cizgi.
    assert geodesic_distance_m(HA1_HOME, TARGET) == pytest.approx(5598, abs=5)


def test_ayni_nokta_sifir_mesafe():
    assert geodesic_distance_m(TARGET, TARGET) == pytest.approx(0.0, abs=1e-6)


def test_kerteriz_kuzey():
    kuzey = LatLon(TARGET.lat + 0.01, TARGET.lon)
    assert initial_bearing_deg(TARGET, kuzey) == pytest.approx(0.0, abs=0.5)


def test_yerel_koordinat_dogu_pozitif():
    dogu = LatLon(TARGET.lat, TARGET.lon + 0.001)
    east, north = to_local_xy(dogu, TARGET)
    assert east > 0
    assert north == pytest.approx(0.0, abs=0.5)


def test_yerel_koordinat_mesafeyle_tutarli():
    nokta = LatLon(TARGET.lat + 0.002, TARGET.lon + 0.002)
    yerel = math.hypot(*to_local_xy(nokta, TARGET))
    assert yerel == pytest.approx(geodesic_distance_m(nokta, TARGET), rel=1e-3)


def test_cember_disindan_gecis_yok():
    # Cemberin 20 m yanindan gecen segment kesisim uretmemeli.
    assert circle_entry_fraction((-50.0, 20.0), (50.0, 20.0), 5.0) is None


def test_cemberi_kesen_segment_iki_ornek_disinda():
    # Iki ornek de cemberin disinda ama segment cemberi kesiyor:
    # mesafe interpolasyonunun kaciracagi, asil onemli durum.
    fraction = circle_entry_fraction((-50.0, 0.0), (50.0, 0.0), 5.0)
    assert fraction is not None
    assert fraction == pytest.approx(0.45, abs=1e-6)


def test_baslangic_cember_icinde_ise_sifir():
    assert circle_entry_fraction((1.0, 1.0), (100.0, 100.0), 5.0) == 0.0


def test_hareketsiz_segment():
    assert circle_entry_fraction((50.0, 0.0), (50.0, 0.0), 5.0) is None


def test_bacak_uzerinde_sapma_sifir():
    orta = LatLon((HA1_HOME.lat + TARGET.lat) / 2, (HA1_HOME.lon + TARGET.lon) / 2)
    assert cross_track_distance_m(orta, HA1_HOME, TARGET) == pytest.approx(0.0, abs=1.0)


def test_bacak_ucunda_sapma_sifir():
    assert cross_track_distance_m(HA1_HOME, HA1_HOME, TARGET) == pytest.approx(0.0, abs=0.1)
    assert cross_track_distance_m(TARGET, HA1_HOME, TARGET) == pytest.approx(0.0, abs=0.1)


def test_dik_sapma_olculur():
    # Bacagin ortasindan kuzeye 0.001 derece (~111 m) kaydirilmis nokta.
    orta = LatLon((HA1_HOME.lat + TARGET.lat) / 2, (HA1_HOME.lon + TARGET.lon) / 2)
    kaydirilmis = LatLon(orta.lat + 0.001, orta.lon)
    sapma = cross_track_distance_m(kaydirilmis, HA1_HOME, TARGET)
    assert 40.0 < sapma < 115.0


def test_bacak_disinda_uc_noktaya_olculur():
    """Bacagin gerisinde kalan nokta icin baslangic ucuna mesafe kullanilir."""
    geride = LatLon(HA1_HOME.lat - 0.01, HA1_HOME.lon)
    beklenen = geodesic_distance_m(geride, HA1_HOME)
    assert cross_track_distance_m(geride, HA1_HOME, TARGET) == pytest.approx(beklenen, rel=0.02)


def test_sifir_uzunluklu_bacak():
    nokta = LatLon(TARGET.lat + 0.001, TARGET.lon)
    beklenen = geodesic_distance_m(nokta, TARGET)
    assert cross_track_distance_m(nokta, TARGET, TARGET) == pytest.approx(beklenen, rel=0.02)


def test_cembere_teget_gecis():
    # Tam yaricap mesafesinden tegetsel gecis; sayisal olarak tek kok.
    fraction = circle_entry_fraction((-50.0, 5.0), (50.0, 5.0), 5.0)
    assert fraction == pytest.approx(0.5, abs=1e-6)


def test_rotanin_cembere_son_girisi_bulunur():
    disari = LatLon(TARGET.lat + 0.04, TARGET.lon)
    iceri = LatLon(TARGET.lat + 0.01, TARGET.lon)
    result = last_circle_entry_on_route(disari, (iceri, TARGET), TARGET, 2500.0)

    assert result is not None
    gate, active_index = result
    assert active_index == 0
    assert geodesic_distance_m(gate, TARGET) == pytest.approx(2500.0, abs=2.0)


def test_rota_cikip_yeniden_girerse_son_giris_secilir():
    kuzey_dis = LatLon(TARGET.lat + 0.04, TARGET.lon)
    kuzey_ic = LatLon(TARGET.lat + 0.01, TARGET.lon)
    guney_dis = LatLon(TARGET.lat - 0.04, TARGET.lon)
    guney_ic = LatLon(TARGET.lat - 0.01, TARGET.lon)
    route = (kuzey_ic, guney_dis, guney_ic, TARGET)

    result = last_circle_entry_on_route(kuzey_dis, route, TARGET, 2500.0)

    assert result is not None
    gate, active_index = result
    assert active_index == 2
    assert gate.lat < TARGET.lat
    assert geodesic_distance_m(gate, TARGET) == pytest.approx(2500.0, abs=2.0)


def test_rota_bastan_icerideyse_yeni_giris_yoktur():
    iceri = LatLon(TARGET.lat + 0.005, TARGET.lon)
    assert last_circle_entry_on_route(iceri, (TARGET,), TARGET, 2500.0) is None
