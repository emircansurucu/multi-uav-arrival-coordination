"""Varis tespiti testleri.

Konumlar hedef etrafinda yerel metre ofsetiyle uretilir; boylece testler
gercek jeodezik mesafelerle calisir.
"""
import math

import pytest

from oasy_uav_agent.estimation.arrival_detector import ArrivalDetector
from oasy_uav_agent.estimation.geodesy import LatLon, geodesic_distance_m

TARGET = LatLon(47.535683, -122.228584)
EARTH_RADIUS_M = 6378137.0
SECOND_NS = 1_000_000_000


def offset(east_m: float, north_m: float) -> LatLon:
    """Hedeften verilen metre ofsetinde konum uretir."""
    lat = TARGET.lat + math.degrees(north_m / EARTH_RADIUS_M)
    lon = TARGET.lon + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(math.radians(TARGET.lat))))
    return LatLon(lat, lon)


def test_ofset_yardimcisi_tutarli():
    assert geodesic_distance_m(offset(100.0, 0.0), TARGET) == pytest.approx(100.0, abs=0.5)


def test_cember_disinda_varis_yok():
    detector = ArrivalDetector(TARGET)
    for step, east in enumerate([-100.0, -60.0, -20.0]):
        assert detector.update(offset(east, 20.0), step * SECOND_NS) is False
    assert not detector.arrived


def test_dogrudan_ornekle_varis():
    detector = ArrivalDetector(TARGET)
    detector.update(offset(-50.0, 0.0), 0)
    assert detector.update(offset(-2.0, 0.0), SECOND_NS) is True
    assert detector.arrived
    assert detector.interpolated is False


def test_ornekler_arasinda_kalan_gecis_interpolasyonla_yakalanir():
    """Iki ornek de cemberin disinda ama segment cemberi kesiyor."""
    detector = ArrivalDetector(TARGET)
    detector.update(offset(-50.0, 0.0), 0)
    assert detector.update(offset(50.0, 0.0), SECOND_NS) is True
    assert detector.interpolated is True
    # Giris orani 0.45 -> 1 saniyelik araligin 0.45'i.
    assert detector.arrival_monotonic_ns == pytest.approx(0.45 * SECOND_NS, rel=1e-3)


def test_varis_bir_kez_tetiklenir():
    detector = ArrivalDetector(TARGET)
    detector.update(offset(-50.0, 0.0), 0)
    detector.update(offset(0.0, 0.0), SECOND_NS)
    ilk_varis = detector.arrival_monotonic_ns
    assert detector.update(offset(1.0, 0.0), 2 * SECOND_NS) is False
    assert detector.arrival_monotonic_ns == ilk_varis


def test_en_yakin_gecis_kaydedilir():
    detector = ArrivalDetector(TARGET)
    for step, east in enumerate([-100.0, -40.0, -12.0]):
        detector.update(offset(east, 8.0), step * SECOND_NS)
    beklenen = math.hypot(12.0, 8.0)
    assert detector.min_distance_m == pytest.approx(beklenen, abs=0.5)


def test_en_yakin_gecis_basari_olcutu_degil():
    """8 m'den gecen arac varis uretmemeli."""
    detector = ArrivalDetector(TARGET)
    for step, east in enumerate([-60.0, 0.0, 60.0]):
        detector.update(offset(east, 8.0), step * SECOND_NS)
    assert not detector.arrived
    assert detector.min_distance_m == pytest.approx(8.0, abs=0.5)


def test_ozel_yaricap():
    detector = ArrivalDetector(TARGET, radius_m=20.0)
    detector.update(offset(-60.0, 0.0), 0)
    assert detector.update(offset(-15.0, 0.0), SECOND_NS) is True
