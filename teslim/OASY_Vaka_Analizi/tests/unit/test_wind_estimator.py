"""rüzgâr kestirimini ve rüzgâr altındaki rota süresini sınar"""
import math

import pytest

from oasy_uav_agent.estimation.geodesy import LatLon
from oasy_uav_agent.estimation.wind_estimator import (
    WIND_SETTLE_AFTER_S,
    body_to_enu,
    WindEstimate,
    WindFilter,
    estimate_wind,
    route_duration_with_airspeed_ramp_s,
    route_duration_with_wind_s,
    wind_from_speed_direction,
)

HA1_HOME = LatLon(47.530002, -122.302457)  # ha1 başlangıç konumu
HA1_ROUTE = [
    LatLon(47.556939, -122.295004),
    LatLon(47.565332, -122.265617),
    LatLon(47.571248, -122.245860),
    LatLon(47.564550, -122.230073),
    LatLon(47.543977, -122.240829),
    LatLon(47.535683, -122.228584),
]  # ha1 test rotası
HA1_ROUTE_LENGTH_M = 12205  # ha1 rotasının yaklaşık uzunluğu
CRUISE_MPS = 22.9  # testlerde kullanılan seyir hızı

RUZGARSIZ = WindEstimate(0.0, 0.0)  # rüzgârsız ortam vektörü


def _quat_zyx(yaw_rad: float, pitch_rad: float = 0.0, roll_rad: float = 0.0):
    """enu çerçevesinde yönelim kuaterniyonu üretir"""
    cy, sy = math.cos(yaw_rad / 2), math.sin(yaw_rad / 2)
    cp, sp = math.cos(pitch_rad / 2), math.sin(pitch_rad / 2)
    cr, sr = math.cos(roll_rad / 2), math.sin(roll_rad / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


# enu sisteminde kuzey yönü 90 derecedir
DUZ_DOGU = _quat_zyx(0.0)  # doğuya düz uçuş yönelimi
DUZ_KUZEY = _quat_zyx(math.pi / 2)  # kuzeye düz uçuş yönelimi


def tirmanis_kuzey(derece: float):
    """kuzeye tırmanış için test yönelimi oluşturur"""
    # flu sisteminde burun yukarı yunuslama negatiftir
    return _quat_zyx(math.pi / 2, -math.radians(derece))


def yatis_kuzey(derece: float):
    """kuzeye yatışlı uçuş için test yönelimi oluşturur"""
    return _quat_zyx(math.pi / 2, 0.0, math.radians(derece))


def test_ruzgarsiz_kestirim_sifir():
    """eşit yer ve hava hızında sıfır rüzgâr kestirilmesini sınar"""
    # kuzeye eşit yer ve hava hızı rüzgârsız olmalı
    tahmin = estimate_wind((0.0, 20.0), (20.0, 0.0, 0.0), DUZ_KUZEY)
    assert tahmin.speed_mps == pytest.approx(0.0, abs=1e-6)


def test_karsi_ruzgar_kestirimi():
    """yer hızı düşükken karşı rüzgâr kestirilmesini sınar"""
    tahmin = estimate_wind((0.0, 15.0), (23.0, 0.0, 0.0), DUZ_KUZEY)
    assert tahmin.north_mps == pytest.approx(-8.0, abs=1e-6)
    assert tahmin.speed_mps == pytest.approx(8.0, abs=1e-6)
    # rüzgâr kuzeyden gelmeli
    assert tahmin.from_direction_deg == pytest.approx(0.0, abs=0.5)


def test_arka_ruzgar_kestirimi():
    """yer hızı yüksekken arka rüzgâr kestirilmesini sınar"""
    tahmin = estimate_wind((0.0, 31.0), (23.0, 0.0, 0.0), DUZ_KUZEY)
    assert tahmin.north_mps == pytest.approx(8.0, abs=1e-6)
    assert tahmin.from_direction_deg == pytest.approx(180.0, abs=0.5)


def test_yan_ruzgar_crab_ile_dogru_kestirilir():
    """rüzgâra kırılmış uçuşta vektör farkını sınar"""
    yaw = math.radians(70.0)  # kuzeyden doğuya 20 derece kırılmış yön
    airspeed = 23.0
    tahmin = estimate_wind((0.0, 20.0), (airspeed, 0.0, 0.0), _quat_zyx(yaw))
    assert tahmin.east_mps == pytest.approx(-airspeed * math.cos(yaw), abs=1e-6)
    assert tahmin.from_direction_deg == pytest.approx(90.0, abs=15.0)


def test_govde_enu_donusumu():
    """gövde hızının enu eksenlerine doğru çevrilmesini sınar"""
    # doğuya bakan araçta ileri doğu ve sol kuzey olmalı
    assert body_to_enu((10.0, 0.0, 0.0), DUZ_DOGU)[0] == pytest.approx(10.0)
    assert body_to_enu((0.0, 10.0, 0.0), DUZ_DOGU)[1] == pytest.approx(10.0)
    # kuzeye bakan araçta ileri kuzey olmalı
    east, north, _ = body_to_enu((10.0, 0.0, 0.0), DUZ_KUZEY)
    assert east == pytest.approx(0.0, abs=1e-9)
    assert north == pytest.approx(10.0)


def test_tirmanista_ileri_bilesenin_yatay_izdusumu_kisalir():
    """tırmanışta ileri eksenin yatay bileşenini sınar"""
    east, north, up = body_to_enu((23.0, 0.0, 0.0), tirmanis_kuzey(15.0))
    assert math.hypot(east, north) == pytest.approx(23.0 * math.cos(math.radians(15.0)), abs=0.01)
    assert up == pytest.approx(23.0 * math.sin(math.radians(15.0)), abs=0.01)


def test_tirmanista_ruzgar_kestirimi_dogru_kalir():
    """tırmanışta rüzgâr kestiriminin değişmemesini sınar"""
    tirmanis_acisi = 15.0
    # gövde hızını yatay bileşene göre kurar
    ileri = 23.0 / math.cos(math.radians(tirmanis_acisi))
    tahmin = estimate_wind(
        (0.0, 15.0), (ileri, 0.0, 0.0), tirmanis_kuzey(tirmanis_acisi)
    )
    assert tahmin.speed_mps == pytest.approx(8.0, abs=0.1)
    assert tahmin.from_direction_deg == pytest.approx(0.0, abs=1.0)


def test_donuste_ruzgar_kestirimi_dogru_kalir():
    """yatış açısının gövde dönüşümüne katılmasını sınar"""
    duz = estimate_wind((0.0, 15.0), (23.0, 0.0, 0.0), DUZ_KUZEY)
    yatik = estimate_wind((0.0, 15.0), (23.0, 0.0, 0.0), yatis_kuzey(30.0))
    assert yatik.speed_mps == pytest.approx(duz.speed_mps, abs=0.01)
    assert yatik.from_direction_deg == pytest.approx(duz.from_direction_deg, abs=0.5)


def test_gecersiz_girdi_reddedilir():
    """sıfır hava hızı örneğinin rüzgâr kestiriminde reddedilmesini sınar"""
    assert estimate_wind((0.0, 20.0), (0.0, 0.0, 0.0), DUZ_KUZEY) is None


def test_ruzgarsiz_rota_suresi_bilinen_degere_esit():
    """rüzgârsız rota süresinin mesafe bölü hız değerine uymasını sınar"""
    sure = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, RUZGARSIZ)
    assert sure == pytest.approx(HA1_ROUTE_LENGTH_M / CRUISE_MPS, rel=0.01)


def test_ruzgar_rota_suresini_uzatir():
    """rüzgârın çok bacaklı rota süresine etkisini sınar"""
    ruzgarli = WindEstimate(0.0, -8.0)
    ruzgarsiz_s = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, RUZGARSIZ)
    ruzgarli_s = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, ruzgarli)
    assert ruzgarli_s > ruzgarsiz_s


def test_yan_ruzgar_da_sureyi_uzatir():
    """yan rüzgârın ileri hız bileşenini azaltmasını sınar"""
    tek_bacak = [LatLon(HA1_HOME.lat + 0.05, HA1_HOME.lon)]  # kuzeye tek bacak
    yan = WindEstimate(8.0, 0.0)
    ruzgarsiz_s = route_duration_with_wind_s(HA1_HOME, tek_bacak, CRUISE_MPS, RUZGARSIZ)
    yan_s = route_duration_with_wind_s(HA1_HOME, tek_bacak, CRUISE_MPS, yan)
    assert yan_s > ruzgarsiz_s
    # yan rüzgârda ileri bileşen azalmalı
    assert yan_s == pytest.approx(ruzgarsiz_s * CRUISE_MPS / 21.45, rel=0.01)


def test_tam_karsi_ruzgarda_tek_bacak_suresi():
    """tek bacakta karşı rüzgârın rota süresine etkisini sınar"""
    kuzeye = [LatLon(HA1_HOME.lat + 0.05, HA1_HOME.lon)]
    karsi = WindEstimate(0.0, -8.0)
    sure = route_duration_with_wind_s(HA1_HOME, kuzeye, CRUISE_MPS, karsi)
    ruzgarsiz_s = route_duration_with_wind_s(HA1_HOME, kuzeye, CRUISE_MPS, RUZGARSIZ)
    # karşı rüzgâr yer hızından düşmeli
    assert sure == pytest.approx(ruzgarsiz_s * CRUISE_MPS / 14.9, rel=0.01)


def test_asiri_ruzgarda_sure_sonsuza_gitmez():
    """aşırı rüzgârda rota süresinin sonlu kalmasını sınar"""
    firtina = WindEstimate(0.0, -100.0)
    sure = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, firtina)
    assert math.isfinite(sure)


def test_sifir_hava_hizi_reddedilir():
    """sıfır hava hızıyla rota süresi hesaplanamamasını sınar"""
    with pytest.raises(ValueError):
        route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, 0.0, RUZGARSIZ)


def test_hizlanma_rampasi_anlik_azami_hizdan_daha_uzun_surer():
    """hızlanma rampasının anlık azami hızdan daha uzun sürmesini sınar"""
    anlik_azami_s = route_duration_with_wind_s(
        HA1_HOME, HA1_ROUTE, 28.0, RUZGARSIZ
    )
    rampali_s = route_duration_with_airspeed_ramp_s(
        HA1_HOME, HA1_ROUTE, 20.0, 28.0, 0.5, RUZGARSIZ
    )
    assert rampali_s > anlik_azami_s


def test_yavaslama_rampasi_anlik_asgari_hizdan_daha_kisa_surer():
    """yavaşlama rampasının anlık asgari hızdan daha kısa sürmesini sınar"""
    anlik_asgari_s = route_duration_with_wind_s(
        HA1_HOME, HA1_ROUTE, 13.0, RUZGARSIZ
    )
    rampali_s = route_duration_with_airspeed_ramp_s(
        HA1_HOME, HA1_ROUTE, 23.0, 13.0, 0.5, RUZGARSIZ
    )
    assert rampali_s < anlik_asgari_s


def test_ha1_rotasinda_8ms_kuzey_ruzgarinin_etkisi():
    """kuzey rüzgârının ha1 rota süresini uzatmasını sınar"""
    karsi = WindEstimate(0.0, -8.0)
    ruzgarsiz_s = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, RUZGARSIZ)
    ruzgarli_s = route_duration_with_wind_s(HA1_HOME, HA1_ROUTE, CRUISE_MPS, karsi)
    # çok yönlü rotadaki net etki tek bacaktan küçük olmalı
    assert 0.0 < ruzgarli_s - ruzgarsiz_s < 120.0


ORNEK_DT_S = 1.0 / 30.0  # ap dds rüzgâr örnek aralığı


def besle(filtre, ornek, sure_s, dt_s=ORNEK_DT_S):
    """rüzgâr filtresini seçilen örnekle verilen süre boyunca besler"""
    for _ in range(int(sure_s / dt_s)):
        filtre.update(ornek, dt_s)


def test_filtre_oturmadan_kestirim_gecerli_sayilmaz():
    """tek örneğin oturmuş kabul edilmemesini sınar"""
    filtre = WindFilter()
    assert filtre.estimate is None
    assert filtre.settled is False

    filtre.update(WindEstimate(0.0, -8.0), ORNEK_DT_S)
    assert filtre.estimate is not None
    assert filtre.settled is False

    besle(filtre, WindEstimate(0.0, -8.0), WIND_SETTLE_AFTER_S)
    assert filtre.settled is True


def test_filtre_sabit_ruzgara_yakinsar():
    """rüzgâr filtresinin sabit örnek değerine yakınsamasını sınar"""
    filtre = WindFilter()
    besle(filtre, WindEstimate(0.0, -8.0), WIND_SETTLE_AFTER_S)

    assert filtre.estimate.speed_mps == pytest.approx(8.0, abs=0.5)
    assert filtre.estimate.from_direction_deg == pytest.approx(0.0, abs=1.0)


def test_filtre_gurultuyu_bastirir():
    """gürültülü örneklerin gerçek değere yakınsamasını sınar"""
    filtre = WindFilter()
    sapmali = (WindEstimate(3.0, -8.0), WindEstimate(-3.0, -8.0))
    for index in range(int(WIND_SETTLE_AFTER_S / ORNEK_DT_S)):
        filtre.update(sapmali[index % 2], ORNEK_DT_S)

    # doğu bileşeni sıfıra yakın kalmalı
    assert filtre.estimate.east_mps == pytest.approx(0.0, abs=0.5)
    assert filtre.estimate.north_mps == pytest.approx(-8.0, abs=0.5)


def test_kuzey_ruzgarinda_yon_sarmasi_bozulmaz():
    """yön sarmasında vektör ortalamasını sınar"""
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
    """filtrenin örnek sıklığından bağımsız yakınsamasını sınar"""
    ruzgar = WindEstimate(0.0, -8.0)
    sik = WindFilter()
    besle(sik, ruzgar, WIND_SETTLE_AFTER_S, dt_s=ORNEK_DT_S)
    seyrek = WindFilter()
    besle(seyrek, ruzgar, WIND_SETTLE_AFTER_S, dt_s=4 * ORNEK_DT_S)

    assert sik.estimate.north_mps == pytest.approx(seyrek.estimate.north_mps, abs=0.2)


def test_duran_telemetri_filtreyi_ilerletmez():
    """sıfır süreli örneklerin filtreyi oturtmamasını sınar"""
    filtre = WindFilter()
    filtre.update(WindEstimate(0.0, -8.0), ORNEK_DT_S)
    for _ in range(1000):
        filtre.update(WindEstimate(0.0, -8.0), 0.0)

    assert filtre.settled is False
