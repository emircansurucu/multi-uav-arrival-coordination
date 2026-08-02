"""Ruzgar kestirimi ve ruzgar altinda rota suresi testleri."""
import math

import pytest

from oasy_uav_agent.estimation.geodesy import LatLon
from oasy_uav_agent.estimation.wind_estimator import (
    WIND_SETTLE_AFTER_S,
    body_to_enu,
    WindEstimate,
    WindFilter,
    estimate_wind,
    route_duration_with_wind_s,
    wind_from_speed_direction,
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

RUZGARSIZ = WindEstimate(0.0, 0.0)


def _quat_zyx(yaw_rad: float, pitch_rad: float = 0.0, roll_rad: float = 0.0):
    """ENU cercevesinde (x, y, z, w) kuaterniyonu; ROS FLU govde sirasi."""
    cy, sy = math.cos(yaw_rad / 2), math.sin(yaw_rad / 2)
    cp, sp = math.cos(pitch_rad / 2), math.sin(pitch_rad / 2)
    cr, sr = math.cos(roll_rad / 2), math.sin(roll_rad / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


# ENU'da yaw dogudan olculur; kuzey = 90 derece.
DUZ_DOGU = _quat_zyx(0.0)
DUZ_KUZEY = _quat_zyx(math.pi / 2)


def tirmanis_kuzey(derece: float):
    # ROS FLU'da pitch sol eksen etrafinda olculur; burun yukari negatiftir.
    return _quat_zyx(math.pi / 2, -math.radians(derece))


def yatis_kuzey(derece: float):
    return _quat_zyx(math.pi / 2, 0.0, math.radians(derece))


def test_ruzgarsiz_kestirim_sifir():
    # Kuzeye 20 m/s yer hizi, burun kuzeyde, 20 m/s ileri hava hizi.
    tahmin = estimate_wind((0.0, 20.0), (20.0, 0.0, 0.0), DUZ_KUZEY)
    assert tahmin.speed_mps == pytest.approx(0.0, abs=1e-6)


def test_karsi_ruzgar_kestirimi():
    """Kuzeye giderken yer hizi hava hizindan dusukse karsi ruzgar var."""
    tahmin = estimate_wind((0.0, 15.0), (23.0, 0.0, 0.0), DUZ_KUZEY)
    assert tahmin.north_mps == pytest.approx(-8.0, abs=1e-6)
    assert tahmin.speed_mps == pytest.approx(8.0, abs=1e-6)
    # Ruzgar kuzeyden geliyor.
    assert tahmin.from_direction_deg == pytest.approx(0.0, abs=0.5)


def test_arka_ruzgar_kestirimi():
    tahmin = estimate_wind((0.0, 31.0), (23.0, 0.0, 0.0), DUZ_KUZEY)
    assert tahmin.north_mps == pytest.approx(8.0, abs=1e-6)
    assert tahmin.from_direction_deg == pytest.approx(180.0, abs=0.5)


def test_yan_ruzgar_crab_ile_dogru_kestirilir():
    """Burun ruzgara kirmisken vektor farki ruzgari tam verir.

    Arac kuzeye ilerliyor ama burnu 20 derece doguya kirmis; hava hizi
    vektoru burun dogrultusunda. Fark, dogudan esen yan ruzgari vermeli.
    """
    yaw = math.radians(70.0)  # ENU: doguya 20 derece kirilmis kuzey
    airspeed = 23.0
    tahmin = estimate_wind((0.0, 20.0), (airspeed, 0.0, 0.0), _quat_zyx(yaw))
    assert tahmin.east_mps == pytest.approx(-airspeed * math.cos(yaw), abs=1e-6)
    assert tahmin.from_direction_deg == pytest.approx(90.0, abs=15.0)


def test_govde_enu_donusumu():
    # Burun doguda, seviye: ileri = dogu, sol = kuzey.
    assert body_to_enu((10.0, 0.0, 0.0), DUZ_DOGU)[0] == pytest.approx(10.0)
    assert body_to_enu((0.0, 10.0, 0.0), DUZ_DOGU)[1] == pytest.approx(10.0)
    # Burun kuzeyde, seviye: ileri = kuzey.
    east, north, _ = body_to_enu((10.0, 0.0, 0.0), DUZ_KUZEY)
    assert east == pytest.approx(0.0, abs=1e-9)
    assert north == pytest.approx(10.0)


def test_tirmanista_ileri_bilesenin_yatay_izdusumu_kisalir():
    """Burun yukaridayken ileri eksen yatay degildir.

    Yalnizca yaw ile dondurulseydi 23 m/s'lik ileri bilesen tamamen yatay
    sayilirdi; 15 derece tirmanista gercek yatay izdusum 22.2 m/s.
    """
    east, north, up = body_to_enu((23.0, 0.0, 0.0), tirmanis_kuzey(15.0))
    assert math.hypot(east, north) == pytest.approx(23.0 * math.cos(math.radians(15.0)), abs=0.01)
    assert up == pytest.approx(23.0 * math.sin(math.radians(15.0)), abs=0.01)


def test_tirmanista_ruzgar_kestirimi_dogru_kalir():
    """Tirmanista olculen ruzgar, seviye ucustakiyle ayni cikmali.

    Olcumde gorulen sapmanin kaynagi buydu: 8 m/s karsi ruzgar tirmanista
    6.3 m/s olarak kestiriliyor ve plan bu degere kilitleniyordu.
    """
    tirmanis_acisi = 15.0
    # Kuzeye 15 m/s yer hizi; hava hizinin yatay izdusumu 23 m/s olmali,
    # yani govde ileri bileseni 23 / cos(15) olur.
    ileri = 23.0 / math.cos(math.radians(tirmanis_acisi))
    tahmin = estimate_wind(
        (0.0, 15.0), (ileri, 0.0, 0.0), tirmanis_kuzey(tirmanis_acisi)
    )
    assert tahmin.speed_mps == pytest.approx(8.0, abs=0.1)
    assert tahmin.from_direction_deg == pytest.approx(0.0, abs=1.0)


def test_donuste_ruzgar_kestirimi_dogru_kalir():
    """Yatiste yan eksen yatay degildir; roll de hesaba katilmali."""
    duz = estimate_wind((0.0, 15.0), (23.0, 0.0, 0.0), DUZ_KUZEY)
    yatik = estimate_wind((0.0, 15.0), (23.0, 0.0, 0.0), yatis_kuzey(30.0))
    assert yatik.speed_mps == pytest.approx(duz.speed_mps, abs=0.01)
    assert yatik.from_direction_deg == pytest.approx(duz.from_direction_deg, abs=0.5)


def test_gecersiz_girdi_reddedilir():
    assert estimate_wind((0.0, 20.0), (0.0, 0.0, 0.0), DUZ_KUZEY) is None


def test_ruzgarsiz_rota_suresi_bilinen_degere_esit():
    sure = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, RUZGARSIZ)
    assert sure == pytest.approx(HA1_ROUTE_LENGTH_M / CRUISE_MPS, rel=0.01)


def test_ruzgar_rota_suresini_uzatir():
    """Kapali rota olmadigindan net etki genelde sureyi uzatir."""
    ruzgarli = WindEstimate(0.0, -8.0)
    ruzgarsiz_s = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, RUZGARSIZ)
    ruzgarli_s = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, ruzgarli)
    assert ruzgarli_s > ruzgarsiz_s


def test_yan_ruzgar_da_sureyi_uzatir():
    """Crab acisi ileri bileseni azalttigi icin yan ruzgar bile yavaslatir."""
    tek_bacak = [LatLon(HA1_HOME.lat + 0.05, HA1_HOME.lon)]  # tam kuzey
    yan = WindEstimate(8.0, 0.0)
    ruzgarsiz_s = route_duration_with_wind_s(HA1_HOME, tek_bacak, CRUISE_MPS, RUZGARSIZ)
    yan_s = route_duration_with_wind_s(HA1_HOME, tek_bacak, CRUISE_MPS, yan)
    assert yan_s > ruzgarsiz_s
    # 8 m/s yan ruzgarda ileri bilesen sqrt(22.9^2 - 8^2) = 21.4 m/s.
    assert yan_s == pytest.approx(ruzgarsiz_s * CRUISE_MPS / 21.45, rel=0.01)


def test_tam_karsi_ruzgarda_tek_bacak_suresi():
    kuzeye = [LatLon(HA1_HOME.lat + 0.05, HA1_HOME.lon)]
    karsi = WindEstimate(0.0, -8.0)
    sure = route_duration_with_wind_s(HA1_HOME, kuzeye, CRUISE_MPS, karsi)
    ruzgarsiz_s = route_duration_with_wind_s(HA1_HOME, kuzeye, CRUISE_MPS, RUZGARSIZ)
    # Yer hizi 22.9 - 8 = 14.9 m/s.
    assert sure == pytest.approx(ruzgarsiz_s * CRUISE_MPS / 14.9, rel=0.01)


def test_asiri_ruzgarda_sure_sonsuza_gitmez():
    firtina = WindEstimate(0.0, -100.0)
    sure = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, firtina)
    assert math.isfinite(sure)


def test_sifir_hava_hizi_reddedilir():
    with pytest.raises(ValueError):
        route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, 0.0, RUZGARSIZ)


def test_ha1_rotasinda_8ms_kuzey_ruzgarinin_etkisi():
    """Olculen senaryo: 8 m/s kuzey ruzgari HA-1 rotasini ne kadar uzatir."""
    karsi = WindEstimate(0.0, -8.0)
    ruzgarsiz_s = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, RUZGARSIZ)
    ruzgarli_s = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, karsi)
    # Rota cok yonlu oldugu icin net etki tek bacaktakinden cok daha kucuk.
    assert 0.0 < ruzgarli_s - ruzgarsiz_s < 120.0


ORNEK_DT_S = 1.0 / 30.0  # AP_DDS ruzgar konularini 33 ms'de bir yayinlar


def besle(filtre, ornek, sure_s, dt_s=ORNEK_DT_S):
    for _ in range(int(sure_s / dt_s)):
        filtre.update(ornek, dt_s)


def test_filtre_oturmadan_kestirim_gecerli_sayilmaz():
    """Tek ornek plani kaydirabilir; oturmadan kullanilmamali."""
    filtre = WindFilter()
    assert filtre.estimate is None
    assert filtre.settled is False

    filtre.update(WindEstimate(0.0, -8.0), ORNEK_DT_S)
    assert filtre.estimate is not None
    assert filtre.settled is False

    besle(filtre, WindEstimate(0.0, -8.0), WIND_SETTLE_AFTER_S)
    assert filtre.settled is True


def test_filtre_sabit_ruzgara_yakinsar():
    filtre = WindFilter()
    besle(filtre, WindEstimate(0.0, -8.0), WIND_SETTLE_AFTER_S)

    assert filtre.estimate.speed_mps == pytest.approx(8.0, abs=0.5)
    assert filtre.estimate.from_direction_deg == pytest.approx(0.0, abs=1.0)


def test_filtre_gurultuyu_bastirir():
    """Sirayla sapan ornekler ortalamada gercek degeri vermeli."""
    filtre = WindFilter()
    sapmali = (WindEstimate(3.0, -8.0), WindEstimate(-3.0, -8.0))
    for index in range(int(WIND_SETTLE_AFTER_S / ORNEK_DT_S)):
        filtre.update(sapmali[index % 2], ORNEK_DT_S)

    # Dogu bileseni +-3 arasinda salindi; filtre sifira yakin kalmali.
    assert filtre.estimate.east_mps == pytest.approx(0.0, abs=0.5)
    assert filtre.estimate.north_mps == pytest.approx(-8.0, abs=0.5)


def test_kuzey_ruzgarinda_yon_sarmasi_bozulmaz():
    """359 ve 1 derecelik olcumlerin ortalamasi 180 degil 0 olmali.

    Filtre (hiz, yon) cifti uzerinde calissaydi bu iki olcum ortalanip
    tam ters yonu gosterirdi; vektor ortalamasi bu tuzagi tasimaz.
    """
    filtre = WindFilter()
    ornekler = (
        wind_from_speed_direction(8.0, 359.0),
        wind_from_speed_direction(8.0, 1.0),
    )
    for index in range(int(WIND_SETTLE_AFTER_S / ORNEK_DT_S)):
        filtre.update(ornekler[index % 2], ORNEK_DT_S)

    assert filtre.estimate.speed_mps == pytest.approx(8.0, abs=0.1)
    sapma = abs((filtre.estimate.from_direction_deg + 180.0) % 360.0 - 180.0)
    assert sapma < 1.0


def test_ornek_araligi_degisince_zaman_sabiti_korunur():
    """Ayni surede beslenen filtre, ornek sikligindan bagimsiz yakinsamali."""
    ruzgar = WindEstimate(0.0, -8.0)
    sik = WindFilter()
    besle(sik, ruzgar, WIND_SETTLE_AFTER_S, dt_s=ORNEK_DT_S)
    seyrek = WindFilter()
    besle(seyrek, ruzgar, WIND_SETTLE_AFTER_S, dt_s=4 * ORNEK_DT_S)

    assert sik.estimate.north_mps == pytest.approx(seyrek.estimate.north_mps, abs=0.2)


def test_duran_telemetri_filtreyi_ilerletmez():
    """dt sifir gelirse ayni ornek filtreyi oturtmamali."""
    filtre = WindFilter()
    filtre.update(WindEstimate(0.0, -8.0), ORNEK_DT_S)
    for _ in range(1000):
        filtre.update(WindEstimate(0.0, -8.0), 0.0)

    assert filtre.settled is False
