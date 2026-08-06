"""merkeziyetsiz varış zamanlamasını sınar"""
import pytest

from oasy_uav_agent.coordination.arrival_schedule import (
    ARRIVAL_SEPARATION_S,
    NANOSECONDS_PER_SECOND,
    compute_reference_arrival,
    compute_takeoff_time,
)

BASE_NS = 1_000_000_000_000  # testlerde kullanılan taban an
SEPARATION_NS = int(ARRIVAL_SEPARATION_S * NANOSECONDS_PER_SECOND)  # varış aralığı


def test_dokuman_ayrimi_yirmi_saniye():
    """zorunlu varış ayrımının yirmi saniye olmasını sınar"""
    assert ARRIVAL_SEPARATION_S == 20.0


def test_oncu_aracin_referansi_yok():
    """ha1 aracının kendi nominal planını kullanmasını sınar"""
    result = compute_reference_arrival(1, {2: BASE_NS, 3: BASE_NS})
    assert not result.resolved
    assert result.source_vehicle_ids == ()


def test_ha2_ha1den_yirmi_saniye_sonra():
    """ha2 planının ha1 varışından yirmi saniye sonra kurulmasını sınar"""
    result = compute_reference_arrival(2, {1: BASE_NS})
    assert result.monotonic_ns == BASE_NS + SEPARATION_NS
    assert result.source_vehicle_ids == (1,)


def test_ha3_iki_kisitin_gec_olanini_secer():
    """ha2 zamanındaysa kendi planının bağlayıcı olmasını sınar"""
    committed = {1: BASE_NS, 2: BASE_NS + SEPARATION_NS}
    result = compute_reference_arrival(3, committed)
    assert result.monotonic_ns == BASE_NS + 2 * SEPARATION_NS
    assert result.source_vehicle_ids == (1, 2)


def test_ha3_ha2_erken_planlarsa_ha1_kisiti_baglayici():
    """erken ha2 planında ha1 kısıtının uygulanmasını sınar"""
    committed = {1: BASE_NS, 2: BASE_NS + 5 * NANOSECONDS_PER_SECOND}
    result = compute_reference_arrival(3, committed)
    assert result.monotonic_ns == BASE_NS + 2 * SEPARATION_NS


def test_ha3_ha2_kayipsa_ha1e_duser():
    """araç kaybında kalan kısıtın korunmasını sınar"""
    result = compute_reference_arrival(3, {1: BASE_NS})
    assert result.monotonic_ns == BASE_NS + 2 * SEPARATION_NS
    assert result.source_vehicle_ids == (1,)


def test_ha3_ha1_kayipsa_ha2ye_duser():
    """ha1 kaybında ha3 aracının ha2 planını kullanmasını sınar"""
    result = compute_reference_arrival(3, {2: BASE_NS})
    assert result.monotonic_ns == BASE_NS + SEPARATION_NS
    assert result.source_vehicle_ids == (2,)


def test_taahhut_yoksa_referans_cozulmez():
    """önceki araç taahhüdü yokken referans oluşmamasını sınar"""
    result = compute_reference_arrival(2, {})
    assert not result.resolved


def test_sonraki_araclar_referansi_etkilemez():
    """ha2 aracının ha3 planından etkilenmemesini sınar"""
    sadece_onceki = compute_reference_arrival(2, {1: BASE_NS})
    sonraki_de_var = compute_reference_arrival(2, {1: BASE_NS, 3: BASE_NS})
    assert sadece_onceki.monotonic_ns == sonraki_de_var.monotonic_ns


def test_kalkis_zamani_ucus_suresi_kadar_once():
    """kalkış anının varıştan uçuş süresi kadar önce olmasını sınar"""
    arrival_ns = BASE_NS
    takeoff_ns = compute_takeoff_time(arrival_ns, 533.0)
    assert (arrival_ns - takeoff_ns) / NANOSECONDS_PER_SECOND == pytest.approx(533.0)


def test_ucus_sureleri_farkliysa_yer_beklemesi_farkli():
    """kısa rotalı ha3 aracının yerde daha uzun beklemesini sınar"""
    ha1_arrival = BASE_NS
    ha3_arrival = compute_reference_arrival(3, {1: ha1_arrival}).monotonic_ns

    ha1_takeoff = compute_takeoff_time(ha1_arrival, 533.0)
    ha3_takeoff = compute_takeoff_time(ha3_arrival, 405.0)

    gecikme_s = (ha3_takeoff - ha1_takeoff) / NANOSECONDS_PER_SECOND
    assert gecikme_s == pytest.approx(40.0 + 533.0 - 405.0, abs=0.1)


def test_capa_en_yavas_araca_gore_belirlenir():
    """ulaşılamayan planda çıpanın yavaş araca göre kurulmasını sınar"""
    from oasy_uav_agent.coordination.arrival_schedule import (
        compute_feasible_anchor,
        target_arrival,
    )

    # ha2 için ulaşılabilen varış anı taban artı 80 saniye
    feasible = {
        1: BASE_NS,
        2: BASE_NS + 80 * NANOSECONDS_PER_SECOND,
        3: BASE_NS + 30 * NANOSECONDS_PER_SECOND,
    }
    anchor = compute_feasible_anchor(feasible)
    # ha2 çıpa adayı taban artı 60 saniye
    assert anchor == BASE_NS + 60 * NANOSECONDS_PER_SECOND

    hedefler = {vid: target_arrival(anchor, vid) for vid in (1, 2, 3)}
    assert hedefler[2] - hedefler[1] == SEPARATION_NS
    assert hedefler[3] - hedefler[2] == SEPARATION_NS
    # ha2 hedefi ulaşılabilen varış anına eşit olmalı
    assert hedefler[2] == feasible[2]


def test_capa_tum_araclarda_ayni_sonucu_verir():
    """aynı veriyi gören araçların aynı çıpayı bulmasını sınar"""
    from oasy_uav_agent.coordination.arrival_schedule import compute_feasible_anchor

    feasible = {1: BASE_NS, 2: BASE_NS + 45 * NANOSECONDS_PER_SECOND, 3: BASE_NS}
    assert compute_feasible_anchor(feasible) == compute_feasible_anchor(dict(reversed(list(feasible.items()))))


def test_capa_tek_arac_verisiyle_de_hesaplanir():
    """ortak çıpanın tek araç verisiyle de hesaplanmasını sınar"""
    from oasy_uav_agent.coordination.arrival_schedule import compute_feasible_anchor

    assert compute_feasible_anchor({3: BASE_NS}) == BASE_NS - 2 * SEPARATION_NS
    assert compute_feasible_anchor({}) is None





def test_kapi_gecis_penceresi_hedef_suresinden_turetilir():
    """kapı geçiş penceresinin hedef varıştan doğru hesaplanmasını sınar"""
    from oasy_uav_agent.coordination.arrival_schedule import compute_gate_release_window

    S = NANOSECONDS_PER_SECOND
    target = 1_000 * S
    lower, upper = compute_gate_release_window(
        target, terminal_earliest_s=200.0, terminal_latest_s=300.0,
        early_margin_s=5.0, late_margin_s=8.0,
    )
    assert lower == 705 * S
    assert upper == 792 * S


def test_kapi_penceresi_kontrol_yetkisi_yoksa_bostur():
    """terminal hız yetkisi yokken kapı penceresinin boş olmasını sınar"""
    from oasy_uav_agent.coordination.arrival_schedule import compute_gate_release_window

    lower, upper = compute_gate_release_window(
        BASE_NS, terminal_earliest_s=300.0, terminal_latest_s=200.0,
    )
    assert lower > upper


def test_kapi_penceresi_negatif_marji_reddeder():
    """negatif kapı güvenlik marjının reddedilmesini sınar"""
    from oasy_uav_agent.coordination.arrival_schedule import compute_gate_release_window

    with pytest.raises(ValueError):
        compute_gate_release_window(BASE_NS, 100.0, 200.0, early_margin_s=-1.0)
