"""Zaman yedirme manevrasi geometrisi testleri."""
import math

import pytest

from oasy_uav_agent.control.maneuver_planner import (
    LOITER_FORBIDDEN_RADIUS_M,
    MAX_LATERAL_OFFSET_M,
    lateral_offset_for_extra,
    loiter_allowed,
    plan_s_maneuver,
    turn_radius_m,
)
from oasy_uav_agent.estimation.geodesy import (
    LatLon,
    cross_track_distance_m,
    geodesic_distance_m,
    to_local_xy,
)

TARGET = LatLon(47.535683, -122.228584)
# HA-1 ve HA-3'un ortak son bacagi: 1304 m.
LEG_START = LatLon(47.543977, -122.240829)
TURN_RADIUS_M = 71.0


def path_length_m(points) -> float:
    """Manevra noktalari mevcut konumu icermez; bacak basi one eklenir."""
    full = [LEG_START, *points]
    return sum(geodesic_distance_m(a, b) for a, b in zip(full, full[1:]))


def side_of_leg(point: LatLon) -> float:
    """Noktanin bacagin hangi tarafinda kaldigini isaretle doner."""
    leg = to_local_xy(TARGET, LEG_START)
    relative = to_local_xy(point, LEG_START)
    return leg[0] * relative[1] - leg[1] * relative[0]


def test_dokuman_kisitlari():
    assert LOITER_FORBIDDEN_RADIUS_M == 2000.0
    assert MAX_LATERAL_OFFSET_M == 500.0


def test_loiter_kritik_bolgede_yasak():
    yakin = LatLon(TARGET.lat + 0.005, TARGET.lon)
    assert geodesic_distance_m(yakin, TARGET) < LOITER_FORBIDDEN_RADIUS_M
    assert loiter_allowed(yakin, TARGET) is False


def test_loiter_bolge_disinda_serbest():
    uzak = LatLon(TARGET.lat + 0.05, TARGET.lon)
    assert geodesic_distance_m(uzak, TARGET) > LOITER_FORBIDDEN_RADIUS_M
    assert loiter_allowed(uzak, TARGET) is True


def test_yanal_ofset_bagintisi():
    """d = sqrt(e*(2L+e))/2 bagintisi dogru uygulanmali."""
    d = lateral_offset_for_extra(1304.0, 280.0, 1)
    beklenen = math.sqrt(280.0 * (2 * 1304.0 + 280.0)) / 2.0
    assert d == pytest.approx(beklenen)
    assert d == pytest.approx(450.0, abs=2.0)


def test_cok_dongu_yanal_ofseti_kucultur():
    tek = lateral_offset_for_extra(1304.0, 280.0, 1)
    iki = lateral_offset_for_extra(1304.0, 280.0, 2)
    uc = lateral_offset_for_extra(1304.0, 280.0, 3)
    assert tek > iki > uc
    assert uc == pytest.approx(150.0, abs=5.0)


def test_donus_yaricapi_hesabi():
    assert turn_radius_m(20.0, 30.0) == pytest.approx(70.6, abs=1.0)
    # Daha dik banka daha kucuk yaricap verir.
    assert turn_radius_m(20.0, 35.0) < turn_radius_m(20.0, 25.0)


def test_manevra_istenen_mesafeyi_ekler():
    manevra = plan_s_maneuver(LEG_START, TARGET, 280.0, TURN_RADIUS_M)
    assert manevra is not None
    duz = geodesic_distance_m(LEG_START, TARGET)
    assert path_length_m(manevra.waypoints) - duz == pytest.approx(280.0, rel=0.05)


def test_manevra_en_az_yanal_sapmayi_secer():
    manevra = plan_s_maneuver(LEG_START, TARGET, 280.0, TURN_RADIUS_M)
    tek_dongu = lateral_offset_for_extra(
        geodesic_distance_m(LEG_START, TARGET), 280.0, 1
    )
    assert manevra.cycles > 1
    assert manevra.lateral_offset_m < tek_dongu


def test_sapma_sinir_icinde_kalir():
    manevra = plan_s_maneuver(LEG_START, TARGET, 280.0, TURN_RADIUS_M)
    for nokta in manevra.waypoints:
        sapma = cross_track_distance_m(nokta, LEG_START, TARGET)
        assert sapma <= MAX_LATERAL_OFFSET_M


def test_asiri_talep_reddedilir():
    """Sinirlar altinda uretilemeyen manevra None donmeli."""
    assert plan_s_maneuver(LEG_START, TARGET, 5000.0, TURN_RADIUS_M) is None


def test_donus_yaricapi_cok_siki_zikzagi_engeller():
    """Buyuk donus yaricapinda kisa bacakta manevra uretilemez."""
    assert plan_s_maneuver(LEG_START, TARGET, 280.0, min_turn_radius_m=400.0) is None


def test_son_nokta_hedef():
    manevra = plan_s_maneuver(LEG_START, TARGET, 280.0, TURN_RADIUS_M)
    assert manevra.waypoints[-1] == TARGET


def test_ofsetler_donusumlu_taraflarda():
    """Zikzak olusmasi icin ara noktalar bacagin iki yanina dagilmali."""
    manevra = plan_s_maneuver(LEG_START, TARGET, 280.0, TURN_RADIUS_M)
    ara = manevra.waypoints[:-1]
    assert len(ara) >= 2
    isaretler = [math.copysign(1.0, side_of_leg(nokta)) for nokta in ara]
    for onceki, sonraki in zip(isaretler, isaretler[1:]):
        assert onceki != sonraki


def test_sifir_talep_manevra_uretmez():
    assert plan_s_maneuver(LEG_START, TARGET, 0.0, TURN_RADIUS_M) is None


def test_takip_noktasi_arac_onunde_kalir():
    """Takip noktasi ulasilabilir olmamali; loiter yaricapindan uzak durmali."""
    from oasy_uav_agent.control.maneuver_planner import follow_path

    manevra = plan_s_maneuver(LEG_START, TARGET, 280.0, TURN_RADIUS_M)
    yol = (LEG_START, *manevra.waypoints)
    durum = follow_path(yol, LEG_START, lookahead_m=250.0)
    assert durum is not None
    # WP_LOITER_RAD 80 m; takip noktasi bunun cok uzerinde olmali.
    assert geodesic_distance_m(LEG_START, durum.carrot) > 200.0


def test_takip_noktasi_yorunge_boyunca_ilerler():
    from oasy_uav_agent.control.maneuver_planner import follow_path

    manevra = plan_s_maneuver(LEG_START, TARGET, 280.0, TURN_RADIUS_M)
    yol = (LEG_START, *manevra.waypoints)

    onceki_kalan = None
    for nokta in yol:
        durum = follow_path(yol, nokta, lookahead_m=250.0)
        if onceki_kalan is not None:
            assert durum.remaining_to_end_m < onceki_kalan
        onceki_kalan = durum.remaining_to_end_m


def test_takip_segmenti_geri_gitmez():
    """min_segment_index gecilen segmentlere donusu engellemeli."""
    from oasy_uav_agent.control.maneuver_planner import follow_path

    manevra = plan_s_maneuver(LEG_START, TARGET, 280.0, TURN_RADIUS_M)
    yol = (LEG_START, *manevra.waypoints)
    ileri = follow_path(yol, yol[2], lookahead_m=250.0)
    geri = follow_path(yol, LEG_START, lookahead_m=250.0, min_segment_index=ileri.segment_index)
    assert geri.segment_index >= ileri.segment_index


def test_yorunge_sonunda_kalan_mesafe_sifira_yaklasir():
    from oasy_uav_agent.control.maneuver_planner import follow_path

    manevra = plan_s_maneuver(LEG_START, TARGET, 280.0, TURN_RADIUS_M)
    yol = (LEG_START, *manevra.waypoints)
    durum = follow_path(yol, TARGET, lookahead_m=250.0)
    assert durum.remaining_to_end_m < 1.0
    assert durum.carrot == TARGET


def test_gecersiz_yorunge_reddedilir():
    from oasy_uav_agent.control.maneuver_planner import follow_path

    assert follow_path((TARGET,), TARGET, 250.0) is None
    assert follow_path((LEG_START, TARGET), LEG_START, 0.0) is None
