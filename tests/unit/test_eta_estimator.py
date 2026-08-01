"""ETA tahmin edicisi testleri.

Senaryolar gercek HA-1 rotasi uzerinde kurulur; ruzgar etkisi, hiz
vektorunu rota dogrultusundan saptirarak temsil edilir.
"""
import math

import pytest

from oasy_uav_agent.estimation.eta_estimator import EtaEstimator
from oasy_uav_agent.estimation.geodesy import (
    LatLon,
    geodesic_distance_m,
    initial_bearing_deg,
    to_local_xy,
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
SECOND_NS = 1_000_000_000


def make_estimator() -> EtaEstimator:
    return EtaEstimator(HA1_ROUTE, HA1_HOME)


def velocity_towards(origin: LatLon, destination: LatLon, speed_mps: float):
    """origin'den destination'a dogru, verilen buyuklukte ENU hiz vektoru."""
    east, north = to_local_xy(destination, origin)
    norm = math.hypot(east, north)
    return (speed_mps * east / norm, speed_mps * north / norm)


def test_baslangicta_kalan_mesafe_rota_uzunlugu():
    estimator = make_estimator()
    result = estimator.update(HA1_HOME, velocity_towards(HA1_HOME, HA1_ROUTE[0], 20.0), 0)
    assert result.remaining_distance_m == pytest.approx(HA1_ROUTE_LENGTH_M, abs=20)


def test_kalan_mesafe_duz_cizgiden_buyuk():
    # Duz cizgi 5598 m; rota takibi zorunlulugu ETA'yi iki katina cikariyor.
    estimator = make_estimator()
    result = estimator.update(HA1_HOME, (0.0, 20.0), 0)
    duz_cizgi = geodesic_distance_m(HA1_HOME, HA1_ROUTE[-1])
    assert result.remaining_distance_m > 2 * duz_cizgi


def test_sabit_hizda_eta_rota_suresine_yakin():
    estimator = make_estimator()
    hiz = velocity_towards(HA1_HOME, HA1_ROUTE[0], 22.9)
    result = estimator.update(HA1_HOME, hiz, 0)
    assert result.eta_s == pytest.approx(HA1_ROUTE_LENGTH_M / 22.9, rel=0.02)


def test_waypoint_yaricapa_girince_ilerler():
    estimator = make_estimator()
    estimator.update(HA1_HOME, (0.0, 20.0), 0)
    assert estimator.active_index == 0
    estimator.update(HA1_ROUTE[0], (0.0, 20.0), SECOND_NS)
    assert estimator.active_index == 1


def test_kose_kesilse_bile_ilerler():
    # Waypoint'in 400 m yanindan ama duzlemini asarak gecis: yaricap kosulu
    # tetiklenmez, iz-boyu kosulu tetiklenmeli.
    estimator = make_estimator()
    estimator.update(HA1_HOME, (0.0, 20.0), 0)
    yan_gecis = LatLon(HA1_ROUTE[0].lat + 0.004, HA1_ROUTE[0].lon + 0.004)
    assert geodesic_distance_m(yan_gecis, HA1_ROUTE[0]) > 400
    estimator.update(yan_gecis, (0.0, 20.0), SECOND_NS)
    assert estimator.active_index >= 1


def test_aktif_indeks_geri_gitmez():
    estimator = make_estimator()
    estimator.update(HA1_ROUTE[2], (0.0, 20.0), 0)
    ileri = estimator.active_index
    estimator.update(HA1_HOME, (0.0, 20.0), SECOND_NS)
    assert estimator.active_index == ileri


def test_hedefte_indeks_durur():
    """Rota bitince indeks son waypoint'te doyar, tasmaz."""
    estimator = make_estimator()
    for index, waypoint in enumerate(HA1_ROUTE):
        estimator.update(waypoint, (0.0, 20.0), index * SECOND_NS)
    assert estimator.active_index == len(HA1_ROUTE) - 1

    for step in range(5):
        estimator.update(HA1_ROUTE[-1], (0.0, 20.0), (10 + step) * SECOND_NS)
    assert estimator.active_index == len(HA1_ROUTE) - 1


def test_telemetri_boslugunda_atlanan_waypointler_toplu_ilerler():
    """Kesinti sirasinda birden fazla waypoint gecilirse indeks toplu ilerlemeli.

    Iz-boyu kosulu kalicidir: arac ileri gittigi surece oran 1.0'in ustunde
    kalir, dolayisiyla kacirilan gecis bir sonraki ornekte yakalanir.
    """
    estimator = make_estimator()
    estimator.update(HA1_HOME, (0.0, 20.0), 0)
    assert estimator.active_index == 0

    # 10 saniyelik kesinti sonrasi arac ucuncu bacaga gecmis durumda.
    estimator.update(HA1_ROUTE[2], (0.0, 20.0), 10 * SECOND_NS)
    assert estimator.active_index == 3


def test_karsi_ruzgar_etayi_uzatir():
    """Ayni konumda dusuk yer hizi daha uzun ETA vermeli."""
    hedefe = lambda hiz: velocity_towards(HA1_HOME, HA1_ROUTE[0], hiz)  # noqa: E731
    normal = make_estimator().update(HA1_HOME, hedefe(22.9), 0)
    karsi_ruzgar = make_estimator().update(HA1_HOME, hedefe(14.0), 0)
    assert karsi_ruzgar.eta_s > normal.eta_s * 1.5


def test_arka_ruzgar_etayi_kisaltir():
    hedefe = lambda hiz: velocity_towards(HA1_HOME, HA1_ROUTE[0], hiz)  # noqa: E731
    normal = make_estimator().update(HA1_HOME, hedefe(22.9), 0)
    arka_ruzgar = make_estimator().update(HA1_HOME, hedefe(30.0), 0)
    assert arka_ruzgar.eta_s < normal.eta_s


def test_yan_ruzgar_ilerleme_hizini_dusurur():
    """Rotaya dik hiz bileseni hedefe yaklastirmaz."""
    ileri = velocity_towards(HA1_HOME, HA1_ROUTE[0], 22.9)
    dik = (-ileri[1], ileri[0])
    result = make_estimator().update(HA1_HOME, dik, 0)
    assert result.progress_speed_mps == pytest.approx(0.0, abs=0.1)


def test_hedeften_uzaklasirken_eta_patlamaz():
    """Negatif ilerleme hizinda alt sinir devreye girer."""
    geri = velocity_towards(HA1_ROUTE[0], HA1_HOME, 22.9)
    result = make_estimator().update(HA1_HOME, geri, 0)
    assert math.isfinite(result.eta_s)
    assert result.eta_s == pytest.approx(HA1_ROUTE_LENGTH_M / 3.0, rel=0.05)


def test_hiz_filtresi_ani_sicramayi_yumusatir():
    estimator = make_estimator()
    hiz = velocity_towards(HA1_HOME, HA1_ROUTE[0], 22.9)
    estimator.update(HA1_HOME, hiz, 0)
    # 0.2 s sonra hiz yariya duserse filtrelenmis deger araya dusmeli.
    dusuk = velocity_towards(HA1_HOME, HA1_ROUTE[0], 11.0)
    result = estimator.update(HA1_HOME, dusuk, SECOND_NS // 5)
    assert 11.0 < result.progress_speed_mps < 22.9


def test_telemetri_kesintisi_sonrasi_filtre_devam_eder():
    """Uzun bosluktan sonra filtre yeni degere hizla yakinsamali."""
    estimator = make_estimator()
    hizli = velocity_towards(HA1_HOME, HA1_ROUTE[0], 22.9)
    estimator.update(HA1_HOME, hizli, 0)
    yavas = velocity_towards(HA1_HOME, HA1_ROUTE[0], 10.0)
    result = estimator.update(HA1_HOME, yavas, 15 * SECOND_NS)
    assert result.progress_speed_mps == pytest.approx(10.0, abs=0.5)


def test_rota_ilerledikce_kalan_mesafe_azalir():
    estimator = make_estimator()
    hiz = (0.0, 20.0)
    onceki = estimator.update(HA1_HOME, hiz, 0).remaining_distance_m
    for index, waypoint in enumerate(HA1_ROUTE, start=1):
        simdiki = estimator.update(waypoint, hiz, index * SECOND_NS).remaining_distance_m
        assert simdiki < onceki
        onceki = simdiki
    assert onceki == pytest.approx(0.0, abs=1.0)


def test_bos_rota_reddedilir():
    with pytest.raises(ValueError):
        EtaEstimator([], HA1_HOME)
