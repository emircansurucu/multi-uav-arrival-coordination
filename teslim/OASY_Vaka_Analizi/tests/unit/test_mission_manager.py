"""sahte bağlantılarla görev durum makinesini sınar"""
import dataclasses
import math

import pytest

from oasy_uav_agent.config_model import VehicleConfig
from oasy_uav_agent.estimation.geodesy import LatLon, geodesic_distance_m
from oasy_uav_agent.estimation.wind_estimator import WIND_SETTLE_AFTER_S
from oasy_uav_agent.mission_manager import (
    GATE_LATE_MARGIN_S,
    TERMINAL_RADIUS_M,
    MissionManager,
    MissionState,
)

HOME = LatLon(47.530002, -122.302457)  # test başlangıç konumu
ROUTE = (
    LatLon(47.556939, -122.295004),
    LatLon(47.543977, -122.240829),
    LatLon(47.535683, -122.228584),
)  # görev yöneticisi test rotası
TARGET = ROUTE[-1]  # ortak hedef
SECOND_NS = 1_000_000_000  # saniyedeki nanosaniye sayısı


def make_config(vehicle_id: int = 1) -> VehicleConfig:
    """seçilen araç kimliğiyle test yapılandırması oluşturur"""
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
        """istenen telemetri değerleriyle sahte bir anlık durum oluşturur"""
        self.position = position
        self.altitude_msl_m = altitude_msl_m
        self.updated_monotonic_ns = monotonic_ns
        self.velocity_east_mps, self.velocity_north_mps = velocity
        # kuzeye eşit yer ve hava hızı rüzgârsız olmalı
        self.airspeed_forward_mps = math.hypot(*velocity)
        self.airspeed_left_mps = 0.0
        self.airspeed_up_mps = 0.0
        # kuzeye düz uçuş yönelimi
        self.orientation_xyzw = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        self.wind_sample_spread_s = wind_sample_spread_s
        self._age_s = age_s

    @property
    def valid(self):
        """sahte telemetride konum bulunup bulunmadığını döner"""
        return self.position is not None

    @property
    def airspeed_mps(self):
        """sahte gövde hava hızının büyüklüğünü döner"""
        return math.sqrt(
            self.airspeed_forward_mps ** 2
            + self.airspeed_left_mps ** 2
            + self.airspeed_up_mps ** 2
        )

    def age_s(self, _now_ns):
        """test için ayarlanan telemetri yaşını döner"""
        return self._age_s if self.valid else math.inf


class FakeTelemetry:
    def __init__(self):
        """başlangıçta geçersiz bir sahte telemetri oluşturur"""
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
        """sahte telemetriyi verilen konum hız ve zamanla günceller"""
        self.current = FakeSnapshot(
            position, altitude_msl_m, monotonic_ns, age_s, velocity, wind_sample_spread_s
        )

    def snapshot(self):
        """güncel sahte telemetri örneğini döner"""
        return self.current


class FakeCommander:
    def __init__(self):
        """başarılı sonuç veren sahte mavlink komutanını hazırlar"""
        self.connected = False
        self.gps_ready = True
        self.upload_ok = True
        self.mode_ok = True
        self.arm_ok = True
        self.flight_mode = ""
        self.modes = []
        self.mode_attempts = []
        self.uploaded = None
        self.armed = False
        self.airspeed_commands = []

    def connect(self):
        """sahte bağlantıyı kurulmuş olarak işaretler"""
        self.connected = True
        return True

    def wait_gps_ready(self):
        """testte ayarlanan gps hazır olma sonucunu döner"""
        return self.gps_ready

    def upload_mission(self, items):
        """yüklenen görevi saklayıp ayarlanan sonucu döner"""
        self.uploaded = items
        return self.upload_ok

    def set_mode(self, name, timeout_s=None):
        """uçuş modu denemesini kaydedip ayarlanan sonucu uygular"""
        self.mode_attempts.append(name)
        if not self.mode_ok:
            return False
        self.modes.append(name)
        self.flight_mode = name
        return True

    def arm(self):
        """arm denemesini kaydedip ayarlanan sonucu döner"""
        self.armed = self.arm_ok
        return self.arm_ok

    def set_airspeed(self, airspeed_mps):
        """gönderilen sahte hava hızı komutunu kaydeder"""
        self.airspeed_commands.append(airspeed_mps)


def make_manager(vehicle_id: int = 1, peer_commitments=None):
    """sahte bağımlılıklarla bir görev yöneticisi oluşturur"""
    telemetry = FakeTelemetry()
    commander = FakeCommander()
    manager = MissionManager(
        make_config(vehicle_id), commander, telemetry, peer_commitments,
        guided_commander=FakeGuided(),
    )
    return manager, commander, telemetry


def advance_to(manager, telemetry, state, max_steps=40):
    """görev yöneticisini istenen duruma ulaşana kadar ilerletir"""
    for _ in range(max_steps):
        if manager.state == state:
            return
        manager.step()
    raise AssertionError(f"{state.name} durumuna ulasilamadi, kalinan: {manager.state.name}")


def test_durum_degerleri_mesaj_sabitleriyle_ayni():
    """mesaj ve görev durumlarının aynı sayıları kullanmasını sınar"""
    vehicle_status = pytest.importorskip(
        "oasy_interfaces.msg", reason="ROS calisma alani kurulmamis"
    ).VehicleStatus
    for state in MissionState:
        assert getattr(vehicle_status, f"STATE_{state.name}") == state.value


def test_init_baglantiya_gecer():
    """ilk adımda bağlantı durumuna geçilmesini sınar"""
    manager, _, _ = make_manager()
    assert manager.state == MissionState.INIT
    manager.step()
    assert manager.state == MissionState.CONNECTING


def test_telemetri_gelmeden_gorev_yuklenmez():
    """telemetri gelmeden görevin yüklenmemesini sınar"""
    manager, commander, telemetry = make_manager()
    for _ in range(5):
        manager.step()
    assert manager.state == MissionState.CONNECTING
    assert commander.connected is True
    assert commander.uploaded is None


def test_gps_hazir_degilse_beklenir():
    """gps hazır değilken bağlantı durumunda beklenmesini sınar"""
    manager, commander, telemetry = make_manager()
    commander.gps_ready = False
    telemetry.set(HOME, 0.0, SECOND_NS)
    for _ in range(5):
        manager.step()
    assert manager.state == MissionState.CONNECTING


def test_gorev_yukleme_basarisiz_olursa_tekrar_denenir():
    """başarısız görev yüklemesinin aynı durumda yeniden denenmesini sınar"""
    manager, commander, telemetry = make_manager()
    commander.upload_ok = False
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.MISSION_UPLOAD)
    for _ in range(3):
        manager.step()
    assert manager.state == MissionState.MISSION_UPLOAD


def test_oncu_arac_peer_beklemeden_kalkisa_gecer():
    """öncü aracın diğer araç taahhüdünü beklememesini sınar"""
    manager, _, telemetry = make_manager(vehicle_id=1)
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.WAIT_TAKEOFF_SLOT)
    # öncü arm olmadan varış taahhüdü vermemeli
    assert manager.snapshot().arrival_committed is False


def test_oncu_taahhudu_kalkis_anina_baglanir():
    """arm süresinin varış planına katılmamasını sınar"""
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
    """takipçi aracın öncü taahhüdü gelmeden kalkmamasını sınar"""
    manager, commander, telemetry = make_manager(vehicle_id=2, peer_commitments=lambda _ns: {})
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.WAIT_PEERS)

    for _ in range(10):
        manager.step()
    assert manager.state == MissionState.WAIT_PEERS
    assert manager.snapshot().arrival_committed is False
    assert commander.armed is False


def test_takipci_gelecekteki_slota_kadar_yerde_bekler():
    """gelecekteki kalkış anından önce arm olunmamasını sınar"""
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
    """geçmiş kalkış anında beklemeden arm olunmasını sınar"""
    import time as _time

    gecmis_varis = _time.monotonic_ns() - 100 * SECOND_NS
    manager, commander, telemetry = make_manager(
        vehicle_id=2, peer_commitments=lambda _ns: {1: gecmis_varis}
    )
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)
    assert commander.armed is True


def test_nominal_ucus_suresi_rota_uzunlugundan_turer():
    """uçuş süresinde rota mesafesinin kullanılmasını sınar"""
    manager, _, _ = make_manager()
    # test rotasının toplam uzunluğu yaklaşık 8676 metre
    assert manager.nominal_flight_s == pytest.approx(8676 / 22.9, rel=0.01)

    duz_cizgi_suresi = geodesic_distance_m(HOME, TARGET) / 22.9
    assert manager.nominal_flight_s > duz_cizgi_suresi * 1.5


def test_arm_sirasinda_once_auto_moduna_gecilir():
    """arm işleminden önce auto moduna geçilmesini sınar"""
    manager, commander, telemetry = make_manager()
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)
    assert commander.modes[0] == "AUTO"
    assert commander.armed is True


WIND_SAMPLE_DT_NS = SECOND_NS // 10  # rüzgâr testi örnek aralığı


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
    """filtreyi oturtacak kadar rüzgâr örneği besler"""
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
    """görev durumunun kalkış ve seyir irtifalarında ilerlemesini sınar"""
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
    """test görevini sahte telemetriyle seyir durumuna getirir"""
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)
    telemetry.set(HOME, 400.0, 2 * SECOND_NS)
    advance_to(manager, telemetry, MissionState.CRUISE)

def terminal_manager(early_s: float, oran: float = 0.10):
    """terminal öncesinde erken ve düşük hızlı araç durumu kurar"""
    import time as _time

    telemetry = FakeTelemetry()
    commander = FakeCommander()
    guided = FakeGuided()
    manager = MissionManager(
        make_config(1), commander, telemetry,
        peer_commitments=lambda _ns: {}, peer_feasible_arrivals=lambda _ns: {},
        guided_commander=guided,
    )
    original_config = manager._config
    manager._config = dataclasses.replace(original_config, loiter_enabled=False)
    reach_cruise(manager, telemetry)
    # aktif rota bacağını sırayla ilerletir
    for index, waypoint in enumerate(ROUTE[:-2]):
        telemetry.set(waypoint, 400.0, (10 + index) * SECOND_NS)
        manager.step()

    onceki = ROUTE[-3] if len(ROUTE) >= 3 else HOME
    yakin = LatLon(
        ROUTE[-2].lat + oran * (onceki.lat - ROUTE[-2].lat),
        ROUTE[-2].lon + oran * (onceki.lon - ROUTE[-2].lon),
    )
    telemetry.set(yakin, 400.0, 20 * SECOND_NS)
    manager.step()

    # erken aracı en düşük hava hızına getirir
    manager._controller._commanded_mps = 15.0
    # planı komut hızından hesaplanan eta ile kurar
    eta_s = manager._model_eta_s(yakin)
    plan_ns = _time.monotonic_ns() + int((eta_s + early_s) * SECOND_NS)
    manager._planned_arrival_ns = plan_ns
    manager._nominal_plan_ns = plan_ns
    return manager, commander, telemetry, guided


def drive_trigger(manager, ticks=45):
    """tetikleyiciyi ardışık görev adımlarıyla çalıştırır"""
    for _ in range(ticks):
        manager.step()


def test_varan_arac_gercek_varis_anini_yayinlar():
    """varan aracın çıpaya gerçek varış anıyla katılmasını sınar"""
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
    """araç kritik hedef bölgesine girince terminal durumuna geçmesini sınar"""
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
    """varıştan sonra rtl ve tamam durumlarının sırayla oluşmasını sınar"""
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
    """eskimiş telemetrinin varış olayı üretmemesini sınar"""
    manager, _, telemetry = make_manager()
    reach_cruise(manager, telemetry)
    telemetry.set(TARGET, 400.0, 3 * SECOND_NS, age_s=10.0)
    manager.step()
    assert manager.state == MissionState.CRUISE
    assert manager.snapshot().target_reached is False


def test_rtl_dogrulanmazsa_done_olmaz():
    """rtl modu doğrulanmadan görevin tamamlanmamasını sınar"""
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
    """terminal yarıçapının dokümandaki iki kilometreye uymasını sınar"""
    assert TERMINAL_RADIUS_M == 2000.0


def test_seyirde_gec_kalinca_hiz_komutu_gonderilir():
    """kısa planda hızlanma komutu gönderilmesini sınar"""
    import time as _time

    manager, commander, telemetry = make_manager(vehicle_id=1)
    reach_cruise(manager, telemetry)

    # kalan rotaya göre kısa bir varış planı kurar
    manager._plan_revised = True
    plan_ns = _time.monotonic_ns() + 60 * SECOND_NS
    manager._planned_arrival_ns = plan_ns
    manager._nominal_plan_ns = plan_ns

    baslangic = manager.snapshot().commanded_airspeed_mps
    # hız sınırı için adımlar arasında kısa süre bırakır
    for step in range(30):
        telemetry.set(HOME, 400.0, (10 + step) * SECOND_NS)
        manager.step()
        _time.sleep(0.02)

    assert commander.airspeed_commands, "hiz komutu gonderilmedi"
    assert manager.snapshot().commanded_airspeed_mps > baslangic


def test_hiz_komutu_konfigurasyon_sinirlarini_asmaz():
    """görev yöneticisinin hız komutlarının yapılandırma sınırlarında kalmasını sınar"""
    import time as _time

    manager, commander, telemetry = make_manager(vehicle_id=1)
    reach_cruise(manager, telemetry)
    # testi yalnızca hız komutu sınırlarına ayırır
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
    """erken ulaşılabilirlikte nominal planın değişmemesini sınar"""
    import time as _time

    manager, _, telemetry = make_manager(vehicle_id=1)
    reach_cruise(manager, telemetry)
    nominal = manager.snapshot().planned_arrival_monotonic_ns
    assert nominal > 0

    for step in range(30):
        telemetry.set(HOME, 400.0, (10 + step) * SECOND_NS)
        manager.step()
        _time.sleep(0.01)

    # plan nominal varış anında sabit kalmalı
    assert manager.snapshot().planned_arrival_monotonic_ns == nominal


def test_ulasilamayan_peer_plani_ileri_kaydirir():
    """yavaş aracın ortak çıpayı ileri taşımasını sınar"""
    import time as _time

    now = _time.monotonic_ns()
    # ha2 için ulaşılabilen varış anı 900 saniye sonra
    yavas_peer = {2: now + 900 * SECOND_NS}
    telemetry = FakeTelemetry()
    commander = FakeCommander()
    manager = MissionManager(
        make_config(1), commander, telemetry,
        peer_commitments=lambda _ns: {},
        peer_feasible_arrivals=lambda _ns: yavas_peer,
        guided_commander=FakeGuided(),
    )
    reach_cruise(manager, telemetry)
    # taahhüt edilen planı çıpa için taban alır
    nominal = manager.snapshot().committed_plan_monotonic_ns

    telemetry.set(HOME, 400.0, 20 * SECOND_NS)
    manager.step()

    plan = manager.snapshot().planned_arrival_monotonic_ns
    assert plan > nominal
    beklenen = yavas_peer[2] - 20 * SECOND_NS
    assert plan == pytest.approx(beklenen, abs=SECOND_NS)


def test_yerdeki_arac_peer_ruzgarina_gore_sureyi_duzeltir():
    """yerdeki aracın diğer araç rüzgârıyla süreyi düzeltmesini sınar"""
    telemetry = FakeTelemetry()
    commander = FakeCommander()
    manager = MissionManager(
        make_config(2), commander, telemetry,
        peer_commitments=lambda _ns: {},
        peer_wind=lambda _ns: (8.0, 0.0),  # kuzeyden sekiz metre bölü saniye
    )
    ruzgarsiz_s = manager.nominal_flight_s

    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.WAIT_PEERS)
    manager.step()

    assert manager.nominal_flight_s > ruzgarsiz_s


def test_ruzgar_yoksa_nominal_sure_degismez():
    """rüzgâr bilgisi yokken nominal uçuş süresinin korunmasını sınar"""
    manager, _, telemetry = make_manager(vehicle_id=2)
    ruzgarsiz_s = manager.nominal_flight_s
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.WAIT_PEERS)
    manager.step()
    assert manager.nominal_flight_s == ruzgarsiz_s


def test_havadaki_arac_ruzgari_yayinlar():
    """yer ve hava hızı farkının rüzgâr olarak yayınlanmasını sınar"""
    manager, _, telemetry = make_manager(vehicle_id=1)
    reach_cruise(manager, telemetry)
    assert manager.snapshot().wind_valid is False or manager.snapshot().wind_speed_mps < 0.1

    # kuzey yönünde sekiz metre bölü saniye karşı rüzgâr kurar
    feed_wind(manager, telemetry, 400.0, 30 * SECOND_NS)

    durum = manager.snapshot()
    assert durum.wind_valid is True
    assert durum.wind_speed_mps == pytest.approx(8.0, abs=0.5)
    assert durum.wind_from_direction_deg == pytest.approx(0.0, abs=5.0)


def test_yerde_beklerken_ruzgar_gelince_slot_guncellenir():
    """rüzgâr gelince yerdeki kalkış anının yenilenmesini sınar"""
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

    # öncü rüzgâr ölçümünü yayınlar
    ruzgar["deger"] = (8.0, 0.0)
    manager.step()
    assert manager.nominal_flight_s > ruzgarsiz_s
    assert manager.state == MissionState.WAIT_TAKEOFF_SLOT


def test_ruzgar_tirmanista_da_olculur():
    """rüzgârın tırmanışta da ölçülmesini sınar"""
    manager, _, telemetry = make_manager(vehicle_id=1)
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)

    feed_wind(manager, telemetry, 150.0, 5 * SECOND_NS)

    assert manager.state in (MissionState.TAKEOFF, MissionState.CLIMB)
    assert manager.snapshot().wind_valid is True
    assert manager.snapshot().wind_speed_mps == pytest.approx(8.0, abs=0.5)


def test_zaman_hizasiz_orneklerden_ruzgar_kestirilmez():
    """zamanı uyuşmayan hız örneklerinin kullanılmamasını sınar"""
    manager, _, telemetry = make_manager(vehicle_id=1)
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)

    son_ns = feed_wind(manager, telemetry, 400.0, 5 * SECOND_NS, wind_sample_spread_s=1.5)
    assert manager.snapshot().wind_valid is False

    # eş zamanlı örneklerle kestirim oluşmalı
    feed_wind(manager, telemetry, 400.0, son_ns)
    assert manager.snapshot().wind_valid is True


def test_yerde_ruzgar_kestirimi_yapilmaz():
    """kalkış koşusundaki düşük hızın rüzgâr üretmemesini sınar"""
    manager, _, telemetry = make_manager(vehicle_id=1)
    telemetry.set(HOME, 0.0, SECOND_NS, velocity=(0.0, 2.0))
    telemetry.current.airspeed_forward_mps = 3.0
    for _ in range(6):
        manager.step()
    assert manager.snapshot().wind_valid is False


def test_alcak_irtifada_ruzgar_kestirimi_yapilmaz():
    """alçak irtifa örneklerinin rüzgâr hesabına katılmamasını sınar"""
    manager, _, telemetry = make_manager(vehicle_id=1)
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)

    # 30 metre irtifadaki örnekler kabul edilmemeli
    son_ns = feed_wind(manager, telemetry, 30.0, 5 * SECOND_NS)
    assert manager.snapshot().wind_valid is False

    # 120 metre irtifadaki örnekler kabul edilmeli
    feed_wind(manager, telemetry, 120.0, son_ns)
    assert manager.snapshot().wind_valid is True


def test_oncu_ruzgar_ogrenince_plani_ileri_ceker():
    """öncünün rüzgâr öğrenince planı ileri çekmesini sınar"""
    manager, _, telemetry = make_manager(vehicle_id=1)
    telemetry.set(HOME, 0.0, SECOND_NS)
    advance_to(manager, telemetry, MissionState.TAKEOFF)
    ruzgarsiz_plan = manager.snapshot().committed_plan_monotonic_ns

    # filtreyi karşı rüzgârla oturtur
    feed_wind(manager, telemetry, 400.0, 10 * SECOND_NS)

    assert manager.snapshot().committed_plan_monotonic_ns > ruzgarsiz_plan


def test_plan_asla_one_alinmaz():
    """rüzgâr zayıflayınca planın geri çekilmemesini sınar"""
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
    """takipçinin öncü planını yerdeyken izlemesini sınar"""
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

    # öncü planını 60 saniye ileri çeker
    ha1_plan["deger"] += 60 * SECOND_NS
    for _ in range(3):
        manager.step()

    yeni_plan = manager.snapshot().planned_arrival_monotonic_ns
    assert yeni_plan > ilk_plan
    assert yeni_plan - ilk_plan == pytest.approx(60 * SECOND_NS, rel=0.05)
    assert manager.state == MissionState.WAIT_TAKEOFF_SLOT


def test_oncu_plani_yalnizca_bir_kez_revize_eder():
    """öncü planının yalnızca bir kez yenilenmesini sınar"""
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
    # yenilemenin dokunduğu nominal planı saklar
    revize_sonrasi = manager._nominal_plan_ns
    assert manager._plan_revised is True

    # ölçüm artsa da nominal plan değişmemeli
    for hiz in (7.0, 8.0, 9.0, 10.0):
        ruzgar["deger"] = (hiz, 0.0)
        for _ in range(3):
            manager.step()
    assert manager._nominal_plan_ns == revize_sonrasi


def test_yayinlanan_plan_capa_kaymasini_tasimaz():
    """çıpa kaymasının yayınlanan taahhüde taşınmamasını sınar"""
    import time as _time

    telemetry = FakeTelemetry()
    commander = FakeCommander()
    yavas_peer = _time.monotonic_ns() + 2000 * SECOND_NS
    manager = MissionManager(
        make_config(1), commander, telemetry,
        peer_commitments=lambda _ns: {},
        peer_feasible_arrivals=lambda _ns: {2: yavas_peer},
        guided_commander=FakeGuided(),
    )
    reach_cruise(manager, telemetry)
    telemetry.set(HOME, 400.0, 30 * SECOND_NS)
    manager.step()

    durum = manager.snapshot()
    # çıpa çalışma planını ileri taşımalı
    assert durum.planned_arrival_monotonic_ns > durum.committed_plan_monotonic_ns
    # yayınlanan taahhüt değişmemeli
    assert durum.committed_plan_monotonic_ns == manager._nominal_plan_ns


def nokta_hedefe_uzaklikta(baslangic, bitis, hedef_mesafe_m):
    """rota bacağı üzerinde hedefe verilen uzaklıkta nokta üretir"""
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


def gate_manager(early_at_gate_s: float = 30.0, hedefe_mesafe_m: float = 2800.0):
    """son yasal kapıya belirtilen kadar erken ulaşan araç kurar"""
    import time as _time

    telemetry = FakeTelemetry()
    commander = FakeCommander()
    guided = FakeGuided()
    manager = MissionManager(
        dataclasses.replace(
            make_config(1), min_airspeed_mps=13.0
        ),
        commander,
        telemetry,
        peer_commitments=lambda _ns: {}, peer_feasible_arrivals=lambda _ns: {},
        guided_commander=guided,
    )
    original_config = manager._config
    manager._config = dataclasses.replace(original_config, loiter_enabled=False)
    reach_cruise(manager, telemetry)
    # kapının bulunduğu rota bacağını etkinleştirir
    telemetry.set(ROUTE[0], 400.0, 10 * SECOND_NS)
    manager.step()

    konum = nokta_hedefe_uzaklikta(ROUTE[0], ROUTE[1], hedefe_mesafe_m)
    telemetry.set(konum, 400.0, 20 * SECOND_NS)
    manager.step()
    # varış taahhüdünü bekleme kapalıyken oturtur
    manager.step()
    manager._config = original_config

    assert manager._hold_gate is not None
    assert manager._active_wp_index == manager._hold_gate.active_wp_index
    eta_to_gate_s = manager._eta_to_gate_s(konum)
    assert eta_to_gate_s is not None

    # testi kapı davranışına ayırmak için çıpayı dondurur
    manager._nominal_plan_ns = manager._planned_arrival_ns

    # kapı penceresini terminal sınırlarından kurar
    now_ns = _time.monotonic_ns()
    terminal_earliest_s = (
        (manager._planned_arrival_ns - now_ns) / SECOND_NS
        - eta_to_gate_s
        - GATE_LATE_MARGIN_S
        - early_at_gate_s
    )
    # 100 saniyelik hız yetkisi pencereyi açık tutmalı
    manager._hold_gate = dataclasses.replace(
        manager._hold_gate,
        terminal_earliest_s=terminal_earliest_s,
        terminal_latest_s=terminal_earliest_s + 100.0,
    )
    return manager, commander, telemetry, guided


def test_kapiya_erken_gelince_tek_yasal_noktada_loiter_baslar():
    """kapıya erken gelen aracın son yasal noktada beklemeye başlamasını sınar"""
    manager, commander, _, guided = gate_manager(early_at_gate_s=30.0)
    assert manager.state == MissionState.CRUISE

    manager.step()

    assert "GUIDED" in commander.modes
    assert len(guided.sent) == 1
    merkez, altitude = guided.sent[0]
    assert merkez == manager._hold_gate.position
    assert geodesic_distance_m(merkez, TARGET) == pytest.approx(2500.0, abs=3.0)
    assert altitude == 400.0


def test_kapi_guided_hedefi_beklerken_yeniden_yayinlanir():
    """tek paket kaybında guided hedefinin yenilenmesini sınar"""
    manager, _, _, guided = gate_manager(early_at_gate_s=30.0)
    manager.step()
    ilk_sayi = len(guided.sent)

    manager.step()

    assert ilk_sayi == 1
    assert len(guided.sent) == 2


def test_kapi_ust_sinira_iki_saniyeden_yakinsa_loiter_yapilmaz():
    """kapı çıkışına çok az kalmışken bekleme başlatılmamasını sınar"""
    manager, commander, _, guided = gate_manager(early_at_gate_s=1.0)
    manager.step()

    assert "GUIDED" not in commander.modes
    assert guided.sent == []


def test_kapi_gec_rezerv_yirmi_saniyenin_altindaysa_kisa_loiter_yapar():
    """kapı geç rezervi azken kısa bekleme başlatılmasını sınar"""
    manager, commander, _, guided = gate_manager(early_at_gate_s=10.0)

    manager.step()

    assert "GUIDED" in commander.modes
    assert len(guided.sent) == 1


def test_hedefe_yakinken_loiter_yapilmaz():
    """hedefe yakınken bekleme yapılmamasını sınar"""
    manager, commander, telemetry, guided = gate_manager(
        early_at_gate_s=30.0, hedefe_mesafe_m=2200.0
    )
    mesafe_m = geodesic_distance_m(telemetry.current.position, TARGET)
    assert 2000.0 < mesafe_m < 2500.0
    assert manager.state == MissionState.CRUISE

    manager.step()

    assert "GUIDED" not in commander.modes
    assert guided.sent == []


def test_loiter_kapaliyken_tetiklenmez():
    """bekleme kapalıyken guided loiter başlatılmamasını sınar"""
    import dataclasses

    manager, commander, _, guided = gate_manager(early_at_gate_s=30.0)
    manager._config = dataclasses.replace(manager._config, loiter_enabled=False)

    manager.step()

    assert "GUIDED" not in commander.modes


def test_kapi_cikis_ani_gelince_auto_ya_donulur():
    """kapı çıkış anı gelince auto moduna dönülmesini sınar"""
    import time as _time

    manager, commander, _, _ = gate_manager(early_at_gate_s=30.0)
    manager.step()
    assert manager._loitering is True

    # çıkış sınırını şimdinin önüne alır
    plan_ns = _time.monotonic_ns() + int(
        (
            manager._hold_gate.terminal_earliest_s
            + GATE_LATE_MARGIN_S
        ) * SECOND_NS
    )
    manager._planned_arrival_ns = plan_ns
    manager._nominal_plan_ns = plan_ns
    manager.step()

    assert manager._loitering is False
    assert commander.modes[-1] == "AUTO"


def test_kapi_auto_gecisi_basarisizsa_normal_akisi_surdurmez():
    """auto geçişi başarısızken normal görevin sürmemesini sınar"""
    import time as _time

    manager, commander, _, _ = gate_manager(early_at_gate_s=30.0)
    manager.step()
    manager._planned_arrival_ns = _time.monotonic_ns() + int(
        (
            manager._hold_gate.terminal_earliest_s
            + GATE_LATE_MARGIN_S
        ) * SECOND_NS
    )
    manager._nominal_plan_ns = manager._planned_arrival_ns
    commander.mode_ok = False

    manager.step()

    assert manager._loitering is True
    assert commander.flight_mode == "GUIDED"
    assert "AUTO" in commander.mode_attempts


def test_kapi_guvenlik_payi_azalirsa_auto_ya_doner():
    """kapı güvenlik payı azalınca guided beklemenin bitmesini sınar"""
    manager, commander, telemetry, _ = gate_manager(early_at_gate_s=30.0)
    manager.step()
    assert manager._loitering is True
    guvensiz = nokta_hedefe_uzaklikta(ROUTE[0], ROUTE[1], 2150.0)
    telemetry.set(guvensiz, 400.0, 30 * SECOND_NS)

    manager.step()

    assert manager._loitering is False
    assert commander.modes[-1] == "AUTO"


def test_kapi_loiteri_canli_eta_sicramasindan_etkilenmez():
    """kapı beklemesinin anlık eta sıçramasıyla bozulmamasını sınar"""
    manager, commander, _, _ = gate_manager(early_at_gate_s=30.0)
    manager.step()
    assert manager._loitering is True

    # canlı eta değerini iki katına çıkarır
    manager._eta_s *= 2.0
    for _ in range(20):
        manager.step()

    assert manager._loitering is True
    assert commander.modes[-1] == "GUIDED"


def test_loiter_sirasinda_aktif_ulasilabilirlik_dondurulur():
    """bekleme sırasında ulaşılabilirliğin dondurulmasını sınar"""
    manager, _, _, _ = gate_manager(early_at_gate_s=30.0)
    manager.step()
    assert manager._loitering is True

    dondurulan = manager.snapshot().earliest_feasible_arrival_monotonic_ns
    for _ in range(10):
        manager.step()

    assert manager.snapshot().earliest_feasible_arrival_monotonic_ns == dondurulan


def test_saglam_pencere_dogru_siralanir():
    """varış sınırlarının aynı rüzgâr zarfından üretilmesini sınar"""
    manager, _, telemetry = make_manager(vehicle_id=1)
    reach_cruise(manager, telemetry)
    konum = telemetry.current.position

    E, L = manager._robust_bounds_s(konum)
    assert E > 0.0
    assert L > 0.0

    # rüzgâr zarfı nominal rüzgârsız süreyi içermek zorunda değil
    from oasy_uav_agent.estimation.wind_estimator import WindEstimate, route_duration_with_wind_s
    nominal = route_duration_with_wind_s(
        konum, ROUTE[manager._active_wp_index:],
        manager._config.nominal_cruise_speed_mps, WindEstimate(0.0, 0.0),
    )
    assert E != pytest.approx(nominal)



def test_ha3_kapisi_duz_mesafeye_degil_kalan_rota_suffixine_baglidir():
    """ha3 kapısında kalan rota mesafesinin düz mesafeden büyük olmasını sınar"""
    from dataclasses import replace
    from oasy_uav_agent.estimation.eta_estimator import route_length_m

    ha3_home = LatLon(47.492515, -122.215659)
    ha3_route = (
        LatLon(47.506321, -122.204110),
        LatLon(47.519036, -122.196368),
        LatLon(47.533029, -122.204368),
        LatLon(47.543977, -122.240829),
        TARGET,
    )
    telemetry = FakeTelemetry()
    manager = MissionManager(
        replace(
            make_config(3),
            home=ha3_home,
            route=ha3_route,
            min_airspeed_mps=13.0,
        ),
        FakeCommander(), telemetry, guided_commander=FakeGuided(),
    )

    gate = manager._hold_gate
    assert gate is not None
    assert gate.active_wp_index == 2
    assert geodesic_distance_m(gate.position, TARGET) == pytest.approx(2500.0, abs=3.0)
    assert route_length_m(gate.position, gate.downstream_route) > 5000.0
    assert gate.terminal_earliest_s < gate.terminal_latest_s


def test_terminal_erken_rezervi_biterken_asgari_hiz_zorlanir():
    """terminal erkenlik rezervi bitince asgari hızın zorlanmasını sınar"""
    import time as _time
    from oasy_uav_agent.estimation.wind_estimator import WindEstimate, route_duration_with_wind_s

    manager, _, telemetry, _ = terminal_manager(early_s=0.0)
    manager._gate_crossed = True
    position = telemetry.current.position
    slow_s = route_duration_with_wind_s(
        position, ROUTE[manager._active_wp_index:],
        manager._config.min_airspeed_mps, WindEstimate(0.0, 0.0),
    )
    manager._planned_arrival_ns = _time.monotonic_ns() + int((slow_s + 20.0) * SECOND_NS)

    forced, reserves = manager._terminal_reserve_override(_time.monotonic_ns())

    assert reserves is not None and reserves[0] < 0.0
    assert forced == manager._config.min_airspeed_mps


def test_terminal_gec_rezervi_biterken_azami_hiz_zorlanir():
    """terminal gecikme rezervi bitince azami hızın zorlanmasını sınar"""
    import time as _time
    from oasy_uav_agent.estimation.wind_estimator import WindEstimate, route_duration_with_wind_s

    manager, _, telemetry, _ = terminal_manager(early_s=0.0)
    manager._gate_crossed = True
    position = telemetry.current.position
    fast_s = route_duration_with_wind_s(
        position, ROUTE[manager._active_wp_index:],
        manager._config.max_airspeed_mps, WindEstimate(0.0, 0.0),
    )
    manager._planned_arrival_ns = _time.monotonic_ns() + int(max(fast_s - 10.0, 1.0) * SECOND_NS)

    forced, reserves = manager._terminal_reserve_override(_time.monotonic_ns())

    assert reserves is not None and reserves[1] < 0.0
    assert forced == manager._config.max_airspeed_mps


class FakeGuided:
    def __init__(self):
        """gönderilen guided hedeflerini tutacak listeyi hazırlar"""
        self.sent = []

    def send(self, position, altitude_msl_m):
        """gönderilen guided konum ve irtifa hedefini kaydeder"""
        self.sent.append((position, altitude_msl_m))


def cruise_manager(early_s: float, hedefe_mesafe_m: float = 4000.0):
    """seyirde erken ve düşük hızlı araç durumu kurar"""
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
    # aktif rota bacağını ilk noktaya ilerletir
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



def test_hiz_yetkisi_varken_loiter_yapilmaz():
    """hızla giderilebilen erkenlikte bekleme yapılmamasını sınar"""
    import time as _time

    manager, commander, telemetry, guided = cruise_manager(early_s=0.0)
    # asgari hızın giderebildiği 30 saniyelik erkenlik kurar
    konum = telemetry.current.position
    manager._controller._commanded_mps = manager._config.nominal_cruise_speed_mps
    plan_ns = _time.monotonic_ns() + int(
        (manager._model_eta_s(konum) + 30.0) * SECOND_NS
    )
    manager._planned_arrival_ns = plan_ns
    manager._nominal_plan_ns = plan_ns

    drive_trigger(manager)

    assert "GUIDED" not in commander.modes
    assert guided.sent == []
