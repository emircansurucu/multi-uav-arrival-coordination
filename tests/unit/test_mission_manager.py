"""Gorev durum makinesi gecis testleri.

MAVLink ve DDS yerine sahte nesneler kullanilir; durum makinesi ROS'a
bagimli olmadigi icin testler calisma alani kurulmadan calisir.
"""
import math

import pytest

from oasy_uav_agent.config_model import VehicleConfig
from oasy_uav_agent.estimation.geodesy import LatLon, geodesic_distance_m
from oasy_uav_agent.mission_manager import (
    TERMINAL_RADIUS_M,
    MissionManager,
    MissionState,
)

HOME = LatLon(47.530002, -122.302457)
ROUTE = (
    LatLon(47.556939, -122.295004),
    LatLon(47.543977, -122.240829),
    LatLon(47.535683, -122.228584),
)
TARGET = ROUTE[-1]
SECOND_NS = 1_000_000_000


def make_config(vehicle_id: int = 1) -> VehicleConfig:
    return VehicleConfig(
        vehicle_id=vehicle_id,
        vehicle_domain_id=vehicle_id,
        coordination_domain_id=10,
        mavlink_address="tcp:127.0.0.1:5760",
        home=HOME,
        route=ROUTE,
        cruise_alt_msl_m=400.0,
        takeoff_alt_msl_m=100.0,
        wp_accept_radius_m=120.0,
        nominal_cruise_speed_mps=22.9,
        min_airspeed_mps=15.0,
        max_airspeed_mps=28.0,
        airspeed_rate_limit_mps2=0.5,
        timing_deadband_s=0.5,
        status_publish_hz=5.0,
        peer_stale_after_s=2.0,
        peer_lost_after_s=5.0,
    )


class FakeSnapshot:
    def __init__(self, position, altitude_msl_m, monotonic_ns, age_s=0.0, velocity=(0.0, 20.0)):
        self.position = position
        self.altitude_msl_m = altitude_msl_m
        self.updated_monotonic_ns = monotonic_ns
        self.velocity_east_mps, self.velocity_north_mps = velocity
        self._age_s = age_s

    @property
    def valid(self):
        return self.position is not None

    def age_s(self, _now_ns):
        return self._age_s if self.valid else math.inf


class FakeTelemetry:
    def __init__(self):
        self.current = FakeSnapshot(None, 0.0, 0)

    def set(self, position, altitude_msl_m, monotonic_ns, age_s=0.0, velocity=(0.0, 20.0)):
        self.current = FakeSnapshot(position, altitude_msl_m, monotonic_ns, age_s, velocity)

    def snapshot(self):
        return self.current


class FakeCommander:
    def __init__(self):
        self.connected = False
        self.gps_ready = True
        self.upload_ok = True
        self.mode_ok = True
        self.arm_ok = True
        self.flight_mode = ""
        self.modes = []
        self.uploaded = None
        self.armed = False
        self.airspeed_commands = []

    def connect(self):
        self.connected = True
        return True

    def wait_gps_ready(self):
        return self.gps_ready

    def upload_mission(self, items):
        self.uploaded = items
        return self.upload_ok

    def set_mode(self, name, timeout_s=None):
        if not self.mode_ok:
            return False
        self.modes.append(name)
        self.flight_mode = name
        return True

    def arm(self):
        self.armed = self.arm_ok
        return self.arm_ok

    def set_airspeed(self, airspeed_mps):
        self.airspeed_commands.append(airspeed_mps)


def make_manager(vehicle_id: int = 1, peer_commitments=None):
    telemetry = FakeTelemetry()
    commander = FakeCommander()
    manager = MissionManager(
        make_config(vehicle_id), commander, telemetry, peer_commitments
    )
    return manager, commander, telemetry


def advance_to(manager, telemetry, state, max_steps=40):
    for _ in range(max_steps):
        if manager.state == state:
            return
        manager.step()
    raise AssertionError(f"{state.name} durumuna ulasilamadi, kalinan: {manager.state.name}")


def test_durum_degerleri_mesaj_sabitleriyle_ayni():
    """VehicleStatus sabitleri ile enum ayni sayilari kullanmali."""
    vehicle_status = pytest.importorskip(
        "oasy_interfaces.msg", reason="ROS calisma alani kurulmamis"
    ).VehicleStatus
    for state in MissionState:
        assert getattr(vehicle_status, f"STATE_{state.name}") == state.value


def test_init_baglantiya_gecer():
    manager, _, _ = make_manager()
    assert manager.state == MissionState.INIT
    manager.step()
    assert manager.state == MissionState.CONNECTING


def test_telemetri_gelmeden_gorev_yuklenmez():
    manager, commander, telemetry = make_manager()
    for _ in range(5):
        manager.step()
    assert manager.state == MissionState.CONNECTING
    assert commander.connected is True
    assert commander.uploaded is None


def test_gps_hazir_degilse_beklenir():
    manager, commander, telemetry = make_manager()
    commander.gps_ready = False
    telemetry.set(HOME, 0.0, SECOND_NS)
    for _ in range(5):
        manager.step()
    assert manager.state == MissionState.CONNECTING


def test_gorev_yukleme_basarisiz_olursa_tekrar_denenir():
    manager, commander, telemetry = make_manager()
    commander.upload_ok = False
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.MISSION_UPLOAD)
    for _ in range(3):
        manager.step()
    assert manager.state == MissionState.MISSION_UPLOAD


def test_oncu_arac_peer_beklemeden_kalkisa_gecer():
    manager, _, telemetry = make_manager(vehicle_id=1)
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.WAIT_TAKEOFF_SLOT)
    # Oncu, arm oncesi taahhut vermez.
    assert manager.snapshot().arrival_committed is False


def test_oncu_taahhudu_kalkis_anina_baglanir():
    """Arm suresi plana girmemeli; taahhut arm tamamlaninca verilir."""
    import time as _time

    manager, _, telemetry = make_manager(vehicle_id=1)
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)

    snapshot = manager.snapshot()
    assert snapshot.arrival_committed is True
    planlanan_ucus_s = (
        snapshot.planned_arrival_monotonic_ns - _time.monotonic_ns()
    ) / SECOND_NS
    assert planlanan_ucus_s == pytest.approx(manager.nominal_flight_s, abs=1.0)


def test_takipci_taahhut_gelmeden_kalkmaz():
    manager, commander, telemetry = make_manager(vehicle_id=2, peer_commitments=lambda _ns: {})
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.WAIT_PEERS)

    for _ in range(10):
        manager.step()
    assert manager.state == MissionState.WAIT_PEERS
    assert manager.snapshot().arrival_committed is False
    assert commander.armed is False


def test_takipci_gelecekteki_slota_kadar_yerde_bekler():
    """Kalkis ani gelecekteyse arm edilmez."""
    import time as _time

    uzak_varis = _time.monotonic_ns() + 3600 * SECOND_NS
    manager, commander, telemetry = make_manager(
        vehicle_id=2, peer_commitments=lambda _ns: {1: uzak_varis}
    )
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.WAIT_TAKEOFF_SLOT)
    assert manager.snapshot().arrival_committed is True

    for _ in range(10):
        manager.step()
    assert manager.state == MissionState.WAIT_TAKEOFF_SLOT
    assert commander.armed is False


def test_gecmisteki_slot_hemen_kalkisa_gecer():
    """Kalkis ani gecmisse beklemeden arm edilir."""
    import time as _time

    gecmis_varis = _time.monotonic_ns() - 100 * SECOND_NS
    manager, commander, telemetry = make_manager(
        vehicle_id=2, peer_commitments=lambda _ns: {1: gecmis_varis}
    )
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)
    assert commander.armed is True


def test_nominal_ucus_suresi_rota_uzunlugundan_turer():
    """Duz cizgi degil rota boyunca mesafe kullanilmali."""
    manager, _, _ = make_manager()
    # Test rotasinin jeodezik bacaklari: 3047 + 4325 + 1304 = 8676 m.
    assert manager.nominal_flight_s == pytest.approx(8676 / 22.9, rel=0.01)

    duz_cizgi_suresi = geodesic_distance_m(HOME, TARGET) / 22.9
    assert manager.nominal_flight_s > duz_cizgi_suresi * 1.5


def test_arm_sirasinda_once_auto_moduna_gecilir():
    manager, commander, telemetry = make_manager()
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)
    assert commander.modes[0] == "AUTO"
    assert commander.armed is True


def test_irtifa_esiklerinde_ilerler():
    manager, commander, telemetry = make_manager()
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)

    telemetry.set(HOME, 50.0, 2 * SECOND_NS)
    manager.step()
    assert manager.state == MissionState.TAKEOFF

    telemetry.set(HOME, 95.0, 3 * SECOND_NS)
    manager.step()
    assert manager.state == MissionState.CLIMB

    telemetry.set(HOME, 200.0, 4 * SECOND_NS)
    manager.step()
    assert manager.state == MissionState.CLIMB

    telemetry.set(HOME, 395.0, 5 * SECOND_NS)
    manager.step()
    assert manager.state == MissionState.CRUISE


def reach_cruise(manager, telemetry):
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)
    telemetry.set(HOME, 400.0, 2 * SECOND_NS)
    advance_to(manager, telemetry, MissionState.CRUISE)


def test_kritik_bolgede_terminal_faza_gecer():
    manager, _, telemetry = make_manager()
    reach_cruise(manager, telemetry)

    uzak = LatLon(TARGET.lat + 0.05, TARGET.lon)
    telemetry.set(uzak, 400.0, 3 * SECOND_NS)
    manager.step()
    assert manager.state == MissionState.CRUISE

    yakin = LatLon(TARGET.lat + 0.01, TARGET.lon)
    telemetry.set(yakin, 400.0, 4 * SECOND_NS)
    manager.step()
    assert manager.state == MissionState.TERMINAL


def test_varis_rtl_ve_done_zinciri():
    manager, commander, telemetry = make_manager()
    reach_cruise(manager, telemetry)

    telemetry.set(LatLon(TARGET.lat + 0.001, TARGET.lon), 400.0, 3 * SECOND_NS)
    manager.step()
    telemetry.set(TARGET, 400.0, 4 * SECOND_NS)
    manager.step()
    assert manager.state == MissionState.ARRIVED

    snapshot = manager.snapshot()
    assert snapshot.target_reached is True
    assert snapshot.arrival_monotonic_ns > 0

    manager.step()
    assert manager.state == MissionState.RTL
    assert "RTL" in commander.modes

    manager.step()
    assert manager.state == MissionState.DONE


def test_eskimis_telemetri_varis_uretmez():
    manager, _, telemetry = make_manager()
    reach_cruise(manager, telemetry)
    telemetry.set(TARGET, 400.0, 3 * SECOND_NS, age_s=10.0)
    manager.step()
    assert manager.state == MissionState.CRUISE
    assert manager.snapshot().target_reached is False


def test_rtl_dogrulanmazsa_done_olmaz():
    manager, commander, telemetry = make_manager()
    reach_cruise(manager, telemetry)
    telemetry.set(LatLon(TARGET.lat + 0.001, TARGET.lon), 400.0, 3 * SECOND_NS)
    manager.step()
    telemetry.set(TARGET, 400.0, 4 * SECOND_NS)
    manager.step()

    commander.mode_ok = False
    for _ in range(3):
        manager.step()
    assert manager.state == MissionState.ARRIVED


def test_terminal_yaricapi_dokuman_kritik_bolgesiyle_ayni():
    assert TERMINAL_RADIUS_M == 2000.0


def test_seyirde_gec_kalinca_hiz_komutu_gonderilir():
    """Plan cok kisa tutulursa arac gec kalir ve hizlanma komutu gitmeli."""
    import time as _time

    manager, commander, telemetry = make_manager(vehicle_id=1)
    reach_cruise(manager, telemetry)

    # Taahhut edilen varisa yalnizca 60 s kaldi ama rota cok daha uzun.
    manager._planned_arrival_ns = _time.monotonic_ns() + 60 * SECOND_NS

    baslangic = manager.snapshot().commanded_airspeed_mps
    # Rate limit gercek gecen sureye bagli oldugu icin adimlar arasinda
    # kisa bekleme gerekiyor.
    for step in range(30):
        telemetry.set(HOME, 400.0, (10 + step) * SECOND_NS)
        manager.step()
        _time.sleep(0.02)

    assert commander.airspeed_commands, "hiz komutu gonderilmedi"
    assert manager.snapshot().commanded_airspeed_mps > baslangic


def test_hiz_komutu_konfigurasyon_sinirlarini_asmaz():
    import time as _time

    manager, commander, telemetry = make_manager(vehicle_id=1)
    reach_cruise(manager, telemetry)
    manager._planned_arrival_ns = _time.monotonic_ns() + 60 * SECOND_NS

    for step in range(40):
        telemetry.set(HOME, 400.0, (10 + step) * SECOND_NS)
        manager.step()
        _time.sleep(0.02)

    assert commander.airspeed_commands
    for komut in commander.airspeed_commands:
        assert 15.0 <= komut <= 28.0


def test_capa_nominal_plandan_ileri_kaymadan_birikmez():
    """Ulasilabilirlik nominalden erkense plan hic degismemeli.

    Kayma bir onceki kaymis degere gore olculurse, azami hizin altinda
    ucarken ulasilabilirligin dogal ileri suruklenmesi birikip plani
    sonsuza kadar oteliyordu.
    """
    import time as _time

    manager, _, telemetry = make_manager(vehicle_id=1)
    reach_cruise(manager, telemetry)
    nominal = manager.snapshot().planned_arrival_monotonic_ns
    assert nominal > 0

    for step in range(30):
        telemetry.set(HOME, 400.0, (10 + step) * SECOND_NS)
        manager.step()
        _time.sleep(0.01)

    # Ulasilabilirlik nominalden erken oldugu surece plan sabit kalmali.
    assert manager.snapshot().planned_arrival_monotonic_ns == nominal


def test_ulasilamayan_peer_plani_ileri_kaydirir():
    """Yavas bir peer'in ulasilabilirligi ortak capayi geriye ceker."""
    import time as _time

    now = _time.monotonic_ns()
    # HA-2 ancak 900 s sonra varabiliyor; HA-1 hedefi bunun 20 s oncesi olmali.
    yavas_peer = {2: now + 900 * SECOND_NS}
    telemetry = FakeTelemetry()
    commander = FakeCommander()
    manager = MissionManager(
        make_config(1), commander, telemetry,
        peer_commitments=lambda _ns: {},
        peer_feasible_arrivals=lambda _ns: yavas_peer,
    )
    reach_cruise(manager, telemetry)
    nominal = manager.snapshot().planned_arrival_monotonic_ns

    telemetry.set(HOME, 400.0, 20 * SECOND_NS)
    manager.step()

    plan = manager.snapshot().planned_arrival_monotonic_ns
    assert plan > nominal
    beklenen = yavas_peer[2] - 20 * SECOND_NS
    assert plan == pytest.approx(beklenen, abs=SECOND_NS)
