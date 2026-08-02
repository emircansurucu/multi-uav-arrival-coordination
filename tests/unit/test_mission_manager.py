"""Gorev durum makinesi gecis testleri.

MAVLink ve DDS yerine sahte nesneler kullanilir; durum makinesi ROS'a
bagimli olmadigi icin testler calisma alani kurulmadan calisir.
"""
import math

import pytest

from oasy_uav_agent.config_model import VehicleConfig
from oasy_uav_agent.estimation.geodesy import LatLon, geodesic_distance_m
from oasy_uav_agent.estimation.wind_estimator import WIND_SETTLE_AFTER_S
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
        s_maneuver_enabled=True,
        loiter_enabled=True,
        status_publish_hz=5.0,
        peer_stale_after_s=2.0,
        peer_lost_after_s=5.0,
    )


class FakeSnapshot:
    def __init__(
        self,
        position,
        altitude_msl_m,
        monotonic_ns,
        age_s=0.0,
        velocity=(0.0, 20.0),
        wind_sample_spread_s=0.0,
    ):
        self.position = position
        self.altitude_msl_m = altitude_msl_m
        self.updated_monotonic_ns = monotonic_ns
        self.velocity_east_mps, self.velocity_north_mps = velocity
        # Burun kuzeyde, hava hizi yer hizina esit -> ruzgarsiz.
        self.airspeed_forward_mps = math.hypot(*velocity)
        self.airspeed_left_mps = 0.0
        self.airspeed_up_mps = 0.0
        # Burun kuzeyde, seviye ucus: ENU'da yaw 90 derece.
        self.orientation_xyzw = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        self.wind_sample_spread_s = wind_sample_spread_s
        self._age_s = age_s

    @property
    def valid(self):
        return self.position is not None

    @property
    def airspeed_mps(self):
        return math.sqrt(
            self.airspeed_forward_mps ** 2
            + self.airspeed_left_mps ** 2
            + self.airspeed_up_mps ** 2
        )

    def age_s(self, _now_ns):
        return self._age_s if self.valid else math.inf


class FakeTelemetry:
    def __init__(self):
        self.current = FakeSnapshot(None, 0.0, 0)

    def set(
        self,
        position,
        altitude_msl_m,
        monotonic_ns,
        age_s=0.0,
        velocity=(0.0, 20.0),
        wind_sample_spread_s=0.0,
    ):
        self.current = FakeSnapshot(
            position, altitude_msl_m, monotonic_ns, age_s, velocity, wind_sample_spread_s
        )

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


# Ruzgar filtresi oturana kadar kestirim yayinlanmaz. Testler bu sureyi
# gercek zamanda beklemek yerine ornek damgalarini ilerleterek gecer.
WIND_SAMPLE_DT_NS = SECOND_NS // 10


def feed_wind(
    manager,
    telemetry,
    altitude_msl_m,
    start_ns,
    velocity=(0.0, 15.0),
    airspeed_forward_mps=23.0,
    duration_s=WIND_SETTLE_AFTER_S + 1.0,
    wind_sample_spread_s=0.0,
):
    """Filtre oturacak kadar ayni ruzgar ornegini besler; son damgayi doner."""
    sample_ns = start_ns
    for _ in range(int(duration_s * SECOND_NS / WIND_SAMPLE_DT_NS)):
        telemetry.set(
            HOME,
            altitude_msl_m,
            sample_ns,
            velocity=velocity,
            wind_sample_spread_s=wind_sample_spread_s,
        )
        telemetry.current.airspeed_forward_mps = airspeed_forward_mps
        manager.step()
        sample_ns += WIND_SAMPLE_DT_NS
    return sample_ns


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


def test_varan_arac_gercek_varis_anini_yayinlar():
    """Varan arac capaya olgu ile katilmali: vardigi an.

    Sifir yayinlansaydi capa kumesinden duser ve capa cokerdi; en son
    varacak arac digerlerinin ucmus oldugu kaymayi kaybederdi. Model dali
    kullanilsaydi da "gorev bastan basliyor" gibi tum rota suresi eklenirdi.
    """
    manager, _, telemetry = make_manager(vehicle_id=1)
    reach_cruise(manager, telemetry)
    manager.step()
    assert manager.snapshot().earliest_feasible_arrival_monotonic_ns > 0

    telemetry.set(LatLon(TARGET.lat + 0.001, TARGET.lon), 400.0, 30 * SECOND_NS)
    manager.step()
    telemetry.set(TARGET, 400.0, 31 * SECOND_NS)
    manager.step()
    assert manager.state == MissionState.ARRIVED

    manager.step()
    snapshot = manager.snapshot()
    assert snapshot.arrival_monotonic_ns > 0
    assert (
        snapshot.earliest_feasible_arrival_monotonic_ns
        == snapshot.arrival_monotonic_ns
    )


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
    # Nominal plan da ayarlanmali, aksi halde capa plani geri yazar; ayrica
    # oncunun tek seferlik ruzgar revizyonu devre disi birakilmali, yoksa
    # plani kalkis anindan yeniden kurup senaryoyu bozuyor.
    manager._plan_revised = True
    plan_ns = _time.monotonic_ns() + 60 * SECOND_NS
    manager._planned_arrival_ns = plan_ns
    manager._nominal_plan_ns = plan_ns

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
    # Capa ve oncu revizyonu plani geri yazmasin; test yalnizca hiz
    # komutunun sinirlar icinde kalmasini olcuyor.
    manager._plan_revised = True
    plan_ns = _time.monotonic_ns() + 60 * SECOND_NS
    manager._planned_arrival_ns = plan_ns
    manager._nominal_plan_ns = plan_ns

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
    # Capa artik yerde ve tirmanista da isledigi icin taban olarak
    # taahhut edilen plan alinir; capa onu hicbir zaman oynatmaz.
    nominal = manager.snapshot().committed_plan_monotonic_ns

    telemetry.set(HOME, 400.0, 20 * SECOND_NS)
    manager.step()

    plan = manager.snapshot().planned_arrival_monotonic_ns
    assert plan > nominal
    beklenen = yavas_peer[2] - 20 * SECOND_NS
    assert plan == pytest.approx(beklenen, abs=SECOND_NS)


class FakeGuided:
    def __init__(self):
        self.sent = []

    def send(self, position, altitude_msl_m):
        self.sent.append((position, altitude_msl_m))


def terminal_manager(early_s: float):
    """Terminal fazda, verilen kadar erken ve minimum hizda bir arac kurar."""
    import time as _time

    telemetry = FakeTelemetry()
    commander = FakeCommander()
    guided = FakeGuided()
    manager = MissionManager(
        make_config(1), commander, telemetry,
        peer_commitments=lambda _ns: {}, peer_feasible_arrivals=lambda _ns: {},
        guided_commander=guided,
    )
    reach_cruise(manager, telemetry)
    # Aktif waypoint'in son bacaga gelmesi icin rotayi sirayla gec; aksi
    # halde rota sapmasi yanlis bacaga gore olculur.
    for index, waypoint in enumerate(ROUTE[:-1]):
        telemetry.set(waypoint, 400.0, (10 + index) * SECOND_NS)
        manager.step()

    # Son bacak uzerinde, hedefe ~890 m kala bir nokta (rota sapmasi sifir).
    onceki = ROUTE[-2]
    oran = 0.68
    yakin = LatLon(
        TARGET.lat + oran * (onceki.lat - TARGET.lat),
        TARGET.lon + oran * (onceki.lon - TARGET.lon),
    )
    telemetry.set(yakin, 400.0, 20 * SECOND_NS)
    manager.step()
    assert manager.state == MissionState.TERMINAL

    # Hiz yetkisi tukenmis: minimum hava hizinda ve erken.
    # Nominal plan da guncellenmeli, aksi halde capa mantigi plani geri yazar.
    manager._controller._commanded_mps = 15.0
    # ETA komut edilen hava hizindan turetildigi icin plan, hiz
    # ayarlandiktan sonra ve ayni modelden kurulmali.
    eta_s = manager._model_eta_s(yakin)
    plan_ns = _time.monotonic_ns() + int((eta_s + early_s) * SECOND_NS)
    manager._planned_arrival_ns = plan_ns
    manager._nominal_plan_ns = plan_ns
    return manager, commander, telemetry, guided


def test_hiz_yetkisi_varken_manevra_planlanmaz():
    manager, commander, _, guided = terminal_manager(early_s=30.0)
    # Kontrolcu minimumda degil: hala yavaslayabilir.
    manager._controller._commanded_mps = 22.0
    manager.step()
    assert "GUIDED" not in commander.modes
    assert guided.sent == []


def drive_trigger(manager, ticks=45):
    """Tetikleyici gurultuye basmasin diye ardisik dogrulama istiyor."""
    for _ in range(ticks):
        manager.step()


def test_yetki_tukendiginde_s_manevrasi_baslar():
    manager, commander, telemetry, guided = terminal_manager(early_s=30.0)
    drive_trigger(manager)
    assert "GUIDED" in commander.modes
    # Manevra bir adimda planlanir, sonraki adimda komut edilmeye baslar.
    manager.step()
    assert guided.sent, "GUIDED konum hedefi gonderilmedi"


def test_manevra_hedefi_guided_ile_komut_etmez():
    """ArduPlane GUIDED hedefte cember atar; 5 m'ye girilemez."""
    manager, _, telemetry, guided = terminal_manager(early_s=30.0)
    drive_trigger(manager)
    for position, _alt in guided.sent:
        assert geodesic_distance_m(position, TARGET) > 1.0


def test_manevra_bir_kez_denenir():
    manager, commander, telemetry, _ = terminal_manager(early_s=30.0)
    drive_trigger(manager)
    guided_sayisi = commander.modes.count("GUIDED")
    for _ in range(5):
        manager.step()
    assert commander.modes.count("GUIDED") == guided_sayisi


def test_manevra_bitince_auto_ya_donulur():
    manager, commander, telemetry, guided = terminal_manager(early_s=30.0)
    drive_trigger(manager)
    assert manager._maneuver_path

    # Her ara noktaya ulasilmis gibi ilerlet.
    for step, waypoint in enumerate(list(manager._maneuver_path)):
        telemetry.set(waypoint, 400.0, (30 + step) * SECOND_NS)
        manager.step()

    assert manager._maneuver_path == ()
    assert commander.modes[-1] == "AUTO"


def test_s_manevrasi_kapaliyken_tetiklenmez():
    """Konfigurasyon kapaliysa yetki tukense bile GUIDED'a gecilmez."""
    import time as _time
    from dataclasses import replace

    telemetry = FakeTelemetry()
    commander = FakeCommander()
    guided = FakeGuided()
    kapali = replace(make_config(1), s_maneuver_enabled=False)
    manager = MissionManager(
        kapali, commander, telemetry,
        peer_commitments=lambda _ns: {}, peer_feasible_arrivals=lambda _ns: {},
        guided_commander=guided,
    )
    reach_cruise(manager, telemetry)
    for index, waypoint in enumerate(ROUTE[:-1]):
        telemetry.set(waypoint, 400.0, (10 + index) * SECOND_NS)
        manager.step()

    onceki = ROUTE[-2]
    yakin = LatLon(
        TARGET.lat + 0.68 * (onceki.lat - TARGET.lat),
        TARGET.lon + 0.68 * (onceki.lon - TARGET.lon),
    )
    telemetry.set(yakin, 400.0, 20 * SECOND_NS)
    manager.step()

    manager._controller._commanded_mps = 15.0
    plan_ns = _time.monotonic_ns() + int((manager._eta_s + 30.0) * SECOND_NS)
    manager._planned_arrival_ns = plan_ns
    manager._nominal_plan_ns = plan_ns

    for _ in range(45):
        manager.step()
    assert "GUIDED" not in commander.modes
    assert guided.sent == []


def test_yerdeki_arac_peer_ruzgarina_gore_sureyi_duzeltir():
    """Ilk kalkan arac ruzgar sondasi; yerdekiler slotunu buna gore kurar."""
    telemetry = FakeTelemetry()
    commander = FakeCommander()
    manager = MissionManager(
        make_config(2), commander, telemetry,
        peer_commitments=lambda _ns: {},
        peer_wind=lambda _ns: (8.0, 0.0),  # 8 m/s, kuzeyden
    )
    ruzgarsiz_s = manager.nominal_flight_s

    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.WAIT_PEERS)
    manager.step()

    assert manager.nominal_flight_s > ruzgarsiz_s


def test_ruzgar_yoksa_nominal_sure_degismez():
    manager, _, telemetry = make_manager(vehicle_id=2)
    ruzgarsiz_s = manager.nominal_flight_s
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.WAIT_PEERS)
    manager.step()
    assert manager.nominal_flight_s == ruzgarsiz_s


def test_havadaki_arac_ruzgari_yayinlar():
    """Yer hizi ile hava hizi farki ruzgar olarak raporlanmali."""
    manager, _, telemetry = make_manager(vehicle_id=1)
    reach_cruise(manager, telemetry)
    assert manager.snapshot().wind_valid is False or manager.snapshot().wind_speed_mps < 0.1

    # Kuzeye 15 m/s ilerliyor ama burnu kuzeyde 23 m/s hava hizinda:
    # 8 m/s karsi ruzgar var.
    feed_wind(manager, telemetry, 400.0, 30 * SECOND_NS)

    durum = manager.snapshot()
    assert durum.wind_valid is True
    assert durum.wind_speed_mps == pytest.approx(8.0, abs=0.5)
    assert durum.wind_from_direction_deg == pytest.approx(0.0, abs=5.0)


def test_yerde_beklerken_ruzgar_gelince_slot_guncellenir():
    """Taahhut ani ruzgar olculmeden once oldugu icin slot sonradan guncellenmeli."""
    import time as _time

    ruzgar = {"deger": None}
    telemetry = FakeTelemetry()
    commander = FakeCommander()
    uzak_varis = _time.monotonic_ns() + 3600 * SECOND_NS
    manager = MissionManager(
        make_config(2), commander, telemetry,
        peer_commitments=lambda _ns: {1: uzak_varis},
        peer_wind=lambda _ns: ruzgar["deger"],
    )
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.WAIT_TAKEOFF_SLOT)

    ruzgarsiz_s = manager.nominal_flight_s
    manager.step()
    assert manager.nominal_flight_s == ruzgarsiz_s

    # Oncu havalanip ruzgari olctu.
    ruzgar["deger"] = (8.0, 0.0)
    manager.step()
    assert manager.nominal_flight_s > ruzgarsiz_s
    assert manager.state == MissionState.WAIT_TAKEOFF_SLOT


def test_ruzgar_tirmanista_da_olculur():
    """Tirmanis 200 s surebiliyor; ruzgar seyri beklemeden yayinlanmali."""
    manager, _, telemetry = make_manager(vehicle_id=1)
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)

    feed_wind(manager, telemetry, 150.0, 5 * SECOND_NS)

    assert manager.state in (MissionState.TAKEOFF, MissionState.CLIMB)
    assert manager.snapshot().wind_valid is True
    assert manager.snapshot().wind_speed_mps == pytest.approx(8.0, abs=0.5)


def test_zaman_hizasiz_orneklerden_ruzgar_kestirilmez():
    """Konum, yer hizi ve hava hizi farkli anlara aitse fark ruzgar degildir.

    Uc konu da 33 ms'de bir yayinlanir; yayilimin buyumesi konulardan
    birinin durdugu anlamina gelir.
    """
    manager, _, telemetry = make_manager(vehicle_id=1)
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)

    son_ns = feed_wind(manager, telemetry, 400.0, 5 * SECOND_NS, wind_sample_spread_s=1.5)
    assert manager.snapshot().wind_valid is False

    # Konular tekrar es zamanli yayinlamaya baslayinca kestirim olusmali.
    feed_wind(manager, telemetry, 400.0, son_ns)
    assert manager.snapshot().wind_valid is True


def test_yerde_ruzgar_kestirimi_yapilmaz():
    """Kalkis kosusundaki dusuk hava hizi anlamli ruzgar vermez."""
    manager, _, telemetry = make_manager(vehicle_id=1)
    telemetry.set(HOME, 0.0, SECOND_NS, velocity=(0.0, 2.0))
    telemetry.current.airspeed_forward_mps = 3.0
    for _ in range(6):
        manager.step()
    assert manager.snapshot().wind_valid is False


def test_alcak_irtifada_ruzgar_kestirimi_yapilmaz():
    """Ruzgar yere dogru azaldigi icin alcak ornekler seyri temsil etmez."""
    manager, _, telemetry = make_manager(vehicle_id=1)
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)

    # 30 m: kalkis irtifasinin (100 m) altinda. Filtre oturacak kadar ornek
    # verilse bile hicbiri kabul edilmemeli.
    son_ns = feed_wind(manager, telemetry, 30.0, 5 * SECOND_NS)
    assert manager.snapshot().wind_valid is False

    # 120 m: esigin uzerinde.
    feed_wind(manager, telemetry, 120.0, son_ns)
    assert manager.snapshot().wind_valid is True


def test_oncu_ruzgar_ogrenince_plani_ileri_ceker():
    """Kalkista ruzgarsiz taahhut veren oncu, ruzgari ogrenince plani uzatmali."""
    manager, _, telemetry = make_manager(vehicle_id=1)
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)
    ruzgarsiz_plan = manager.snapshot().committed_plan_monotonic_ns

    # 8 m/s karsi ruzgarda, esigin uzerinde irtifada seyre gec. Revizyon
    # ruzgar kestirimi oturmadan yapilmaz, bu yuzden filtre beslenir.
    feed_wind(manager, telemetry, 400.0, 10 * SECOND_NS)

    assert manager.snapshot().committed_plan_monotonic_ns > ruzgarsiz_plan


def test_plan_asla_one_alinmaz():
    """Ratchet: ruzgar zayiflasa bile plan geriye cekilmemeli."""
    ruzgar = {"deger": (8.0, 0.0)}
    telemetry = FakeTelemetry()
    commander = FakeCommander()
    manager = MissionManager(
        make_config(1), commander, telemetry,
        peer_commitments=lambda _ns: {},
        peer_wind=lambda _ns: ruzgar["deger"],
    )
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)
    telemetry.set(HOME, 400.0, 10 * SECOND_NS, velocity=(0.0, 20.0))
    telemetry.current.airspeed_forward_mps = 20.0
    advance_to(manager, telemetry, MissionState.CRUISE)
    for _ in range(3):
        manager.step()
    ruzgarli_plan = manager.snapshot().planned_arrival_monotonic_ns

    ruzgar["deger"] = (0.0, 0.0)
    for _ in range(5):
        manager.step()
    assert manager.snapshot().planned_arrival_monotonic_ns >= ruzgarli_plan


def test_takipci_oncunun_revizyonunu_yerde_izler():
    """Oncu tirmanista plani ileri cekince, yerdeki takipci de izlemeli."""
    import time as _time

    ha1_plan = {"deger": _time.monotonic_ns() + 600 * SECOND_NS}
    telemetry = FakeTelemetry()
    commander = FakeCommander()
    manager = MissionManager(
        make_config(2), commander, telemetry,
        peer_commitments=lambda _ns: {1: ha1_plan["deger"]},
    )
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.WAIT_TAKEOFF_SLOT)
    ilk_plan = manager.snapshot().planned_arrival_monotonic_ns

    # Oncu ruzgari ogrenip planini 60 s ileri cekti.
    ha1_plan["deger"] += 60 * SECOND_NS
    for _ in range(3):
        manager.step()

    yeni_plan = manager.snapshot().planned_arrival_monotonic_ns
    assert yeni_plan > ilk_plan
    assert yeni_plan - ilk_plan == pytest.approx(60 * SECOND_NS, rel=0.05)
    assert manager.state == MissionState.WAIT_TAKEOFF_SLOT


def test_oncu_plani_yalnizca_bir_kez_revize_eder():
    """Gurultulu ruzgar olcumu plani tekrar tekrar ileri itmemeli."""
    ruzgar = {"deger": (6.0, 0.0)}
    telemetry = FakeTelemetry()
    commander = FakeCommander()
    manager = MissionManager(
        make_config(1), commander, telemetry,
        peer_commitments=lambda _ns: {},
        peer_wind=lambda _ns: ruzgar["deger"],
    )
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)
    feed_wind(manager, telemetry, 400.0, 10 * SECOND_NS)
    # Capa planned_arrival'i ayrica oynatabilir; revizyonun dokundugu deger
    # nominal plandir.
    revize_sonrasi = manager._nominal_plan_ns
    assert manager._plan_revised is True

    # Olcum artsa bile nominal plan bir daha degismemeli.
    for hiz in (7.0, 8.0, 9.0, 10.0):
        ruzgar["deger"] = (hiz, 0.0)
        for _ in range(3):
            manager.step()
    assert manager._nominal_plan_ns == revize_sonrasi


def test_yayinlanan_plan_capa_kaymasini_tasimaz():
    """Peer'lara taahhut edilmis plan gider; capa duzeltmesi yerel kalir.

    Capa kaymasi taahhut uzerinden tasinirsa araclar birbirinin kaymasini
    besleyip plani sonsuza kadar ileri itiyor.
    """
    import time as _time

    telemetry = FakeTelemetry()
    commander = FakeCommander()
    yavas_peer = _time.monotonic_ns() + 2000 * SECOND_NS
    manager = MissionManager(
        make_config(1), commander, telemetry,
        peer_commitments=lambda _ns: {},
        peer_feasible_arrivals=lambda _ns: {2: yavas_peer},
    )
    reach_cruise(manager, telemetry)
    telemetry.set(HOME, 400.0, 30 * SECOND_NS)
    manager.step()

    durum = manager.snapshot()
    # Capa calisma planini ileri itmis olmali...
    assert durum.planned_arrival_monotonic_ns > durum.committed_plan_monotonic_ns
    # ...ama yayinlanan taahhut degismemis olmali.
    assert durum.committed_plan_monotonic_ns == manager._nominal_plan_ns


def test_manevra_sirasinda_ulasilabilirlik_dondurulur():
    """Manevra ilerlemeyi dusurur; ulasilabilirlik bundan etkilenmemeli.

    Aksi halde capa geriye kayar, daha fazla yedirme gerekir ve manevra
    kendini besler.
    """
    manager, _, telemetry, _ = terminal_manager(early_s=30.0)
    drive_trigger(manager)
    assert manager._maneuver_path, "manevra baslamadi"

    dondurulan = manager.snapshot().earliest_feasible_arrival_monotonic_ns
    # Ilerleme cokse bile ulasilabilirlik degismemeli.
    manager._progress_speed_mps = 1.0
    for _ in range(5):
        manager.step()
    assert manager.snapshot().earliest_feasible_arrival_monotonic_ns == dondurulan


def test_anlik_eta_sicramasi_manevra_tetiklemez():
    """Donuslerde -274 s gibi gecici degerler goruldu; tek ornek yetmemeli.

    Kosul dogrulama suresi dolmadan ortadan kalkarsa sayac sifirlanmali.
    """
    manager, commander, telemetry, guided = terminal_manager(early_s=300.0)

    # Dogrulama esiginin altinda kalacak kadar adim: henuz tetiklenmemeli.
    for _ in range(20):
        manager.step()
    assert "GUIDED" not in commander.modes
    assert manager._trigger_streak > 0

    # Sicrama gecti: arac artik minimum hizda degil, kosul bozuldu.
    manager._controller._commanded_mps = 22.0
    manager.step()
    assert manager._trigger_streak == 0, "sayac sifirlanmadi"

    for _ in range(20):
        manager.step()
    assert "GUIDED" not in commander.modes, "gecici sicrama manevra tetikledi"


def test_manevra_plan_sinirlari_ucus_marji_birakir():
    """Ucusta pursuit tasmasi oluyor; plan siniri 500 m'den dar olmali."""
    from oasy_uav_agent.control.maneuver_planner import MAX_LATERAL_OFFSET_M
    from oasy_uav_agent.mission_manager import MANEUVER_PLAN_LATERAL_LIMIT_M

    assert MANEUVER_PLAN_LATERAL_LIMIT_M < MAX_LATERAL_OFFSET_M
    # Olculen tasma orani ~1.6; plan siniri bunu 500 m altinda tutmali.
    assert MANEUVER_PLAN_LATERAL_LIMIT_M * 1.6 <= MAX_LATERAL_OFFSET_M


def nokta_hedefe_uzaklikta(baslangic, bitis, hedef_mesafe_m):
    """baslangic->bitis bacagi uzerinde, hedefe verilen uzaklikta nokta."""
    dusuk, yuksek = 0.0, 1.0
    nokta = baslangic
    for _ in range(40):
        orta = (dusuk + yuksek) / 2
        nokta = LatLon(
            baslangic.lat + orta * (bitis.lat - baslangic.lat),
            baslangic.lon + orta * (bitis.lon - baslangic.lon),
        )
        if geodesic_distance_m(nokta, TARGET) > hedef_mesafe_m:
            dusuk = orta
        else:
            yuksek = orta
    return nokta


def cruise_manager(early_s: float, hedefe_mesafe_m: float = 4000.0):
    """Seyirde, verilen kadar erken, asgari hizda ve hedefe belirli uzaklikta."""
    import time as _time

    telemetry = FakeTelemetry()
    commander = FakeCommander()
    guided = FakeGuided()
    manager = MissionManager(
        make_config(1), commander, telemetry,
        peer_commitments=lambda _ns: {}, peer_feasible_arrivals=lambda _ns: {},
        guided_commander=guided,
    )
    reach_cruise(manager, telemetry)
    # Once ilk waypoint'e gel ki aktif bacak dogru olsun; sapma o bacaga
    # gore olculuyor.
    telemetry.set(ROUTE[0], 400.0, 10 * SECOND_NS)
    manager.step()

    konum = nokta_hedefe_uzaklikta(ROUTE[0], ROUTE[1], hedefe_mesafe_m)
    telemetry.set(konum, 400.0, 20 * SECOND_NS)
    manager.step()

    manager._controller._commanded_mps = 15.0
    eta_s = manager._model_eta_s(konum)
    plan_ns = _time.monotonic_ns() + int((eta_s + early_s) * SECOND_NS)
    manager._planned_arrival_ns = plan_ns
    manager._nominal_plan_ns = plan_ns
    return manager, commander, telemetry, guided


def test_yetki_tukendiginde_loiter_baslar():
    """Asgari hizda hala cok erkense daire cizilerek zaman kaybedilmeli."""
    manager, commander, _, guided = cruise_manager(early_s=30.0)
    assert manager.state == MissionState.CRUISE

    drive_trigger(manager)

    assert "GUIDED" in commander.modes
    assert len(guided.sent) == 1


def test_hiz_yetkisi_varken_loiter_yapilmaz():
    manager, commander, _, guided = cruise_manager(early_s=30.0)
    manager._controller._commanded_mps = 22.0

    drive_trigger(manager)

    assert "GUIDED" not in commander.modes
    assert guided.sent == []


def test_hedefe_yakinken_loiter_yapilmaz():
    """Madde 6: hedefin 2 km cevresinde loiter yasak, paylisiyla birlikte."""
    manager, commander, telemetry, guided = cruise_manager(
        early_s=30.0, hedefe_mesafe_m=2200.0
    )
    mesafe_m = geodesic_distance_m(telemetry.current.position, TARGET)
    assert 2000.0 < mesafe_m < 2500.0
    assert manager.state == MissionState.CRUISE

    drive_trigger(manager)

    assert "GUIDED" not in commander.modes
    assert guided.sent == []


def test_loiter_kapaliyken_tetiklenmez():
    import dataclasses

    manager, commander, _, guided = cruise_manager(early_s=30.0)
    manager._config = dataclasses.replace(manager._config, loiter_enabled=False)

    drive_trigger(manager)

    assert "GUIDED" not in commander.modes


def test_zamanlama_kapaninca_loiterdan_cikilir():
    """Cikis kapali cevrim: erkenlik kapaninca AUTO'ya donulmeli."""
    import time as _time

    manager, commander, _, _ = cruise_manager(early_s=30.0)
    drive_trigger(manager)
    assert manager._loitering is True

    # Daire cizerken sure gecti ve erkenlik kapandi. Nominal plan da
    # guncellenmeli, aksi halde capa plani geri yazar.
    plan_ns = _time.monotonic_ns() + int(manager._loiter_entry_eta_s * SECOND_NS)
    manager._planned_arrival_ns = plan_ns
    manager._nominal_plan_ns = plan_ns
    manager.step()

    assert manager._loitering is False
    assert commander.modes[-1] == "AUTO"


def test_loiter_cikisi_sisen_etaya_aldanmaz():
    """Daire cizerken ETA sisiyor; cikis buna gore erken verilmemeli.

    Olculen hata: 16.5 s erken girilip 4 saniye sonra cikildi. Cikis canli
    ETA'ya baksaydi bu tekrarlanirdi.
    """
    manager, commander, _, _ = cruise_manager(early_s=30.0)
    drive_trigger(manager)
    assert manager._loitering is True

    # Ilerleme coktugu icin canli ETA iki katina ciksin.
    manager._eta_s *= 2.0
    for _ in range(20):
        manager.step()

    assert manager._loitering is True
    assert commander.modes[-1] == "GUIDED"


def test_loiter_sirasinda_ulasilabilirlik_dondurulur():
    """Daire cizerken rota ilerlemesi durur ama aracin yapabilecegi degismez."""
    manager, _, _, _ = cruise_manager(early_s=30.0)
    drive_trigger(manager)
    assert manager._loitering is True

    dondurulan = manager.snapshot().earliest_feasible_arrival_monotonic_ns
    for _ in range(10):
        manager.step()

    assert manager.snapshot().earliest_feasible_arrival_monotonic_ns == dondurulan
