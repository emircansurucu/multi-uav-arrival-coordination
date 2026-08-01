"""Merkeziyetsiz varis zamanlamasi sozlesmesi testleri."""
import pytest

from oasy_uav_agent.coordination.arrival_schedule import (
    ARRIVAL_SEPARATION_S,
    NANOSECONDS_PER_SECOND,
    compute_reference_arrival,
    compute_takeoff_time,
)

BASE_NS = 1_000_000_000_000
SEPARATION_NS = int(ARRIVAL_SEPARATION_S * NANOSECONDS_PER_SECOND)


def test_dokuman_ayrimi_yirmi_saniye():
    assert ARRIVAL_SEPARATION_S == 20.0


def test_oncu_aracin_referansi_yok():
    """HA-1 kimseyi takip etmez; kendi nominal planini uygular."""
    result = compute_reference_arrival(1, {2: BASE_NS, 3: BASE_NS})
    assert not result.resolved
    assert result.source_vehicle_ids == ()


def test_ha2_ha1den_yirmi_saniye_sonra():
    result = compute_reference_arrival(2, {1: BASE_NS})
    assert result.monotonic_ns == BASE_NS + SEPARATION_NS
    assert result.source_vehicle_ids == (1,)


def test_ha3_iki_kisitin_gec_olanini_secer():
    """HA-2 zamaninda ise baglayici kisit HA-2 + 20'dir."""
    committed = {1: BASE_NS, 2: BASE_NS + SEPARATION_NS}
    result = compute_reference_arrival(3, committed)
    assert result.monotonic_ns == BASE_NS + 2 * SEPARATION_NS
    assert result.source_vehicle_ids == (1, 2)


def test_ha3_ha2_erken_planlarsa_ha1_kisiti_baglayici():
    """HA-2 kendi planini cok erkene koyarsa HA-1 + 40 devreye girer."""
    committed = {1: BASE_NS, 2: BASE_NS + 5 * NANOSECONDS_PER_SECOND}
    result = compute_reference_arrival(3, committed)
    assert result.monotonic_ns == BASE_NS + 2 * SEPARATION_NS


def test_ha3_ha2_kayipsa_ha1e_duser():
    """Peer kaybinda kalan kisit gecerli kalir."""
    result = compute_reference_arrival(3, {1: BASE_NS})
    assert result.monotonic_ns == BASE_NS + 2 * SEPARATION_NS
    assert result.source_vehicle_ids == (1,)


def test_ha3_ha1_kayipsa_ha2ye_duser():
    result = compute_reference_arrival(3, {2: BASE_NS})
    assert result.monotonic_ns == BASE_NS + SEPARATION_NS
    assert result.source_vehicle_ids == (2,)


def test_taahhut_yoksa_referans_cozulmez():
    result = compute_reference_arrival(2, {})
    assert not result.resolved


def test_sonraki_araclar_referansi_etkilemez():
    """HA-2, HA-3'u dinlemez; sira asla tersine donmez."""
    sadece_onceki = compute_reference_arrival(2, {1: BASE_NS})
    sonraki_de_var = compute_reference_arrival(2, {1: BASE_NS, 3: BASE_NS})
    assert sadece_onceki.monotonic_ns == sonraki_de_var.monotonic_ns


def test_kalkis_zamani_ucus_suresi_kadar_once():
    arrival_ns = BASE_NS
    takeoff_ns = compute_takeoff_time(arrival_ns, 533.0)
    assert (arrival_ns - takeoff_ns) / NANOSECONDS_PER_SECOND == pytest.approx(533.0)


def test_ucus_sureleri_farkliysa_yer_beklemesi_farkli():
    """HA-3'un rotasi kisa oldugu icin yerde daha uzun beklemeli."""
    ha1_arrival = BASE_NS
    ha3_arrival = compute_reference_arrival(3, {1: ha1_arrival}).monotonic_ns

    ha1_takeoff = compute_takeoff_time(ha1_arrival, 533.0)
    ha3_takeoff = compute_takeoff_time(ha3_arrival, 405.0)

    gecikme_s = (ha3_takeoff - ha1_takeoff) / NANOSECONDS_PER_SECOND
    assert gecikme_s == pytest.approx(40.0 + 533.0 - 405.0, abs=0.1)


def test_capa_en_yavas_araca_gore_belirlenir():
    """Ulasilamayan bir plan varsa capa o araca gore geriye kayar."""
    from oasy_uav_agent.coordination.arrival_schedule import (
        compute_feasible_anchor,
        target_arrival,
    )

    # HA-2 ancak BASE+80'de varabiliyor (ruzgar); HA-1 ve HA-3 daha erken.
    feasible = {
        1: BASE_NS,
        2: BASE_NS + 80 * NANOSECONDS_PER_SECOND,
        3: BASE_NS + 30 * NANOSECONDS_PER_SECOND,
    }
    anchor = compute_feasible_anchor(feasible)
    # HA-2 adayi: BASE+80-20 = BASE+60, en gec olan bu.
    assert anchor == BASE_NS + 60 * NANOSECONDS_PER_SECOND

    hedefler = {vid: target_arrival(anchor, vid) for vid in (1, 2, 3)}
    assert hedefler[2] - hedefler[1] == SEPARATION_NS
    assert hedefler[3] - hedefler[2] == SEPARATION_NS
    # HA-2'nin hedefi kendi ulasabilecegi ana esit olmali.
    assert hedefler[2] == feasible[2]


def test_capa_tum_araclarda_ayni_sonucu_verir():
    """Merkeziyetsizlik: ayni veriyi goren her arac ayni capayi bulur."""
    from oasy_uav_agent.coordination.arrival_schedule import compute_feasible_anchor

    feasible = {1: BASE_NS, 2: BASE_NS + 45 * NANOSECONDS_PER_SECOND, 3: BASE_NS}
    assert compute_feasible_anchor(feasible) == compute_feasible_anchor(dict(reversed(list(feasible.items()))))


def test_capa_tek_arac_verisiyle_de_hesaplanir():
    from oasy_uav_agent.coordination.arrival_schedule import compute_feasible_anchor

    assert compute_feasible_anchor({3: BASE_NS}) == BASE_NS - 2 * SEPARATION_NS
    assert compute_feasible_anchor({}) is None
