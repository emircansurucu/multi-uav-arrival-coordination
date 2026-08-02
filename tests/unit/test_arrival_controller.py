"""Varis hiz kontrolcusu testleri."""
import pytest

from oasy_uav_agent.control.arrival_controller import (
    MIN_REMAINING_TIME_S,
    ArrivalController,
    ControlAction,
)

NANOSECONDS_PER_SECOND = 1_000_000_000
NOW_NS = 1_000_000_000_000


def make_controller(**overrides) -> ArrivalController:
    params = {
        "nominal_airspeed_mps": 22.9,
        "min_airspeed_mps": 15.0,
        "max_airspeed_mps": 28.0,
        "rate_limit_mps_per_s": 0.5,
        "deadband_s": 0.5,
    }
    params.update(overrides)
    return ArrivalController(**params)


def planned_in(seconds: float) -> int:
    return NOW_NS + int(seconds * NANOSECONDS_PER_SECOND)


def test_gecersiz_sinirlar_reddedilir():
    with pytest.raises(ValueError):
        make_controller(min_airspeed_mps=28.0, max_airspeed_mps=15.0)


def test_taahhut_yoksa_komut_uretilmez():
    controller = make_controller()
    assert controller.update(300.0, 6000.0, 0, NOW_NS, 1.0) is None


def test_varisa_az_kalinca_komut_degismez():
    """Kalan sure kucuklurken gerekli hiz sonsuza gider; kontrol durur."""
    controller = make_controller()
    result = controller.update(
        5.0, 100.0, planned_in(MIN_REMAINING_TIME_S - 1), NOW_NS, 1.0
    )
    assert result is None


def test_deadband_icinde_mudahale_yok():
    controller = make_controller()
    onceki = controller.commanded_airspeed_mps
    command = controller.update(300.3, 6870.0, planned_in(300.0), NOW_NS, 1.0)
    assert command.action is ControlAction.HOLD
    assert command.deadband_active is True
    assert command.changed is False
    assert controller.commanded_airspeed_mps == onceki


def test_gec_kalinca_hizlanir():
    controller = make_controller()
    onceki = controller.commanded_airspeed_mps
    # ETA 310 s ama 300 s kaldi: 10 saniye gec.
    command = controller.update(310.0, 6870.0, planned_in(300.0), NOW_NS, 1.0)
    assert command.action is ControlAction.SPEED_UP
    assert command.timing_error_s == pytest.approx(10.0)
    assert controller.commanded_airspeed_mps > onceki


def test_erken_kalinca_yavaslar():
    controller = make_controller()
    onceki = controller.commanded_airspeed_mps
    command = controller.update(290.0, 6870.0, planned_in(300.0), NOW_NS, 1.0)
    assert command.action is ControlAction.SLOW_DOWN
    assert controller.commanded_airspeed_mps < onceki


def test_rate_limit_ani_sicramayi_engeller():
    controller = make_controller(rate_limit_mps_per_s=0.5)
    onceki = controller.commanded_airspeed_mps
    # Cok buyuk hata olsa bile 1 saniyede en fazla 0.5 m/s degisebilir.
    command = controller.update(600.0, 12000.0, planned_in(300.0), NOW_NS, 1.0)
    assert command.rate_limited is True
    assert abs(controller.commanded_airspeed_mps - onceki) == pytest.approx(0.5, abs=1e-6)


def test_saturation_ust_sinirda_durur():
    controller = make_controller(rate_limit_mps_per_s=100.0)
    # 22.9 * (600/300) = 45.8 m/s; ust sinir 28.
    command = controller.update(600.0, 12000.0, planned_in(300.0), NOW_NS, 1.0)
    assert command.saturated is True
    assert controller.commanded_airspeed_mps == pytest.approx(28.0)


def test_saturation_alt_sinirda_durur():
    controller = make_controller(rate_limit_mps_per_s=100.0)
    # 22.9 * (100/300) = 7.6 m/s; alt sinir 15.
    command = controller.update(100.0, 1500.0, planned_in(300.0), NOW_NS, 1.0)
    assert command.saturated is True
    assert controller.commanded_airspeed_mps == pytest.approx(15.0)


def test_komut_hicbir_zaman_sinir_disina_cikmaz():
    controller = make_controller(rate_limit_mps_per_s=100.0)
    for eta, distance in [(600.0, 12000.0), (100.0, 1500.0), (300.0, 6870.0)]:
        controller.update(eta, distance, planned_in(300.0), NOW_NS, 1.0)
        assert 15.0 <= controller.commanded_airspeed_mps <= 28.0


def test_hata_kapaninca_kontrolcu_yerlesir():
    """Kapali dongu: hata sifira giderken komut salinmamali.

    Arac, komut edilen hava hizinin 0.95 kati kadar yer ilerlemesi yapiyor
    (donus ve ruzgar kaybi). Kontrolcu bunu telafi edip yerlesmeli.
    """
    controller = make_controller(rate_limit_mps_per_s=100.0)
    distance_m = 6870.0
    remaining_s = 300.0
    ilerleme_orani = 0.95
    komutlar = []

    for _ in range(20):
        eta_s = distance_m / (controller.commanded_airspeed_mps * ilerleme_orani)
        command = controller.update(eta_s, distance_m, planned_in(remaining_s), NOW_NS, 1.0)
        komutlar.append(controller.commanded_airspeed_mps)
        if command is not None and command.deadband_active:
            break

    # Yerlesen komut, kaybi telafi eden hiza yakinsamali.
    beklenen = distance_m / remaining_s / ilerleme_orani
    assert komutlar[-1] == pytest.approx(beklenen, rel=0.02)
    assert max(komutlar[-3:]) - min(komutlar[-3:]) < 0.2


def test_kucuk_degisiklikte_yeni_komut_gonderilmez():
    controller = make_controller(rate_limit_mps_per_s=0.01)
    command = controller.update(310.0, 6870.0, planned_in(300.0), NOW_NS, 1.0)
    # 1 saniyede en fazla 0.01 m/s degisim, esik 0.1 m/s.
    assert command.changed is False


def test_kucuk_adimlar_birikince_komut_gonderilir():
    """Rate limit adimi esikten kucuk olsa da birikim komutu tetiklemeli.

    Karsilastirma bir onceki tick'e gore yapilsaydi (0.5 m/s^2 * 0.05 s =
    0.025 m/s) hicbir komut gonderilmezdi.
    """
    controller = make_controller(rate_limit_mps_per_s=0.5)
    gonderilen = 0
    for _ in range(20):
        command = controller.update(400.0, 9000.0, planned_in(300.0), NOW_NS, 0.05)
        if command is not None and command.changed:
            gonderilen += 1
    assert gonderilen > 0

