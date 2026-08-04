"""Gorev durum makinesi.

Kendi thread'inde calisir: MAVLink cagrilari bloklayici oldugu icin rclpy
executor'larindan cagrilamaz. Telemetriyi AP_DDS anlik durumundan okur,
yalnizca kendi aracina komut gonderir.

Durum degerleri VehicleStatus mesajindaki sabitlerle ayni sayilardir;
esleme birim testinde dogrulanir.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Dict, Optional, Tuple

from pymavlink import mavutil

from .autopilot_adapter.mavlink_link import MissionItem
from .autopilot_adapter.mission_builder import (
    S_SLOT_ACCEPT_RADIUS_M,
    S_SLOT_COUNT,
    build_mission,
    point_along_leg,
    spare_slot_range,
)
from .config_model import VehicleConfig
from .coordination.arrival_schedule import (
    NANOSECONDS_PER_SECOND,
    compute_feasible_anchor,
    compute_gate_release_window,
    compute_reference_arrival,
    compute_takeoff_time,
    target_arrival,
)
from .control.arrival_controller import ArrivalController
from .control.maneuver_planner import (
    follow_path,
    plan_s_maneuver,
    turn_radius_m,
)
from .estimation.arrival_detector import ArrivalDetector
from .estimation.eta_estimator import EtaEstimator, route_length_m
from .estimation.wind_estimator import (
    WindEstimate,
    WindFilter,
    estimate_wind,
    route_duration_with_airspeed_ramp_s,
    route_duration_with_wind_s,
    wind_from_speed_direction,
)
from .estimation.geodesy import (
    LatLon,
    cross_track_distance_m,
    geodesic_distance_m,
    last_circle_entry_on_route,
    to_local_xy,
)

logger = logging.getLogger(__name__)

TICK_INTERVAL_S = 0.05
PEER_WAIT_LOG_INTERVAL_S = 10.0
LATE_TAKEOFF_TOLERANCE_S = 1.0
# Hedefin 2 km cevresi dokumanda loiter yasagi olan kritik bolge; terminal
# faza gecis bu sinirdan baslar.
TERMINAL_RADIUS_M = 2000.0
ALTITUDE_REACHED_MARGIN_M = 15.0
TELEMETRY_TIMEOUT_S = 3.0
# Vaka dokumani madde 4: mesafe yedirme manevralarinda rotadan sapma en
# fazla 500 m olabilir. Asilirsa uyari uretilir.
MAX_ROUTE_DEVIATION_M = 500.0
# Loiter, hedefe 2 km'den yakinda yasaktir (madde 6). Daire yaricapi
# WP_LOITER_RAD kadar oldugu icin cemberin hicbir noktasi yasak bolgeye
# girmemeli; ustune olcum ve cikis gecikmesi icin pay birakilir.
MIN_LOITER_DISTANCE_M = 2500.0
# Kapidan cikis, sabit-ruzgar zarfiyla bulunan en gec guvenli ana yakin
# secilir. Bu, terminale asgari hizda degil, iki yone de hiz yetkisiyle
# girmeyi saglar. Gec kalma marji, olculen 12-16 s son-basamak etkisini ve
# hiz komutunun rate-limit ile oturmasini kapsayacak 20 s rezerv birakir;
# erken marji pencere sinirinda titremeyi onler.
GATE_EARLY_MARGIN_S = 3.0
GATE_LATE_MARGIN_S = 20.0
GATE_HOLD_TRIGGER_S = 2.0
GATE_CROSSING_HYSTERESIS_M = 40.0
# WP_LOITER_RAD parametre dosyalarinda 80 m'ye sabitlenir. Beklenmeyen L1
# tasmasi bu paylari tuketirse, 2 km yasak bolgeye veya 500 m rota sapmasina
# gelmeden AUTO'ya donulur.
GATE_TARGET_DISTANCE_ABORT_M = 2200.0
GATE_ROUTE_DEVIATION_ABORT_M = 400.0
# Kapidan sonra olculen ruzgara gore iki hiz rezervi korunur. Bu bir gelecek
# ruzgar kehaneti degildir; degisim algilandiginda kalan min/max hiz yetkisini
# son metreye kadar tuketmemeyi saglayan kapali-cevrim bariyerdir.
TERMINAL_EARLY_RESERVE_S = 18.0
TERMINAL_LATE_RESERVE_S = 10.0
TERMINAL_RESERVE_HYSTERESIS_S = 2.0
# Capa bu esikten az kaydiginda log uretilmez.
GATE_LOG_INTERVAL_S = 10.0
ANCHOR_LOG_THRESHOLD_S = 1.0
# S-manevrasi yalnizca hiz yetkisi tukendiginde devreye girer: arac
# minimum hava hizinda oldugu halde hala bu kadar erken variyorsa.
S_MANEUVER_TRIGGER_S = 3.0
# Tetikleyici anlik ETA sicramalarina basmamali: donuslerde ilerleme hizi
# dustugu icin -274 s gibi gecici degerler goruldu ve bunlar geri donulemez
# bir manevra planlatiyordu. Kosul bu kadar ardisik adim surmelidir.
S_MANEUVER_CONFIRM_TICKS = 40
# Planlanan yanal ofset ile ucular sapma ayni degil: pursuit gudumu zikzak
# koselerinde tasiyor (olculen 497 m plan -> 816 m ucus). Dokumandaki 500 m
# sinirinin ucusta da tutmasi icin plan bu daha dar sinirla yapilir.
MANEUVER_PLAN_LATERAL_LIMIT_M = 280.0
MANEUVER_BANK_ANGLE_DEG = 30.0
# Yuvalar son bacak boyunca yeniden yazilabilir. Tek seferlik karar
# basarisizliklari cozemiyordu: olculdu, uc basarisiz kosuda da arac son
# bacaga GIRERKEN zamanindaydi (-0.4/-0.3/-8.0 s) ve seyir hizindaydi;
# erkenligin tamami bacak icinde, ruzgar donunce dogdu. Karar aninda bilgi
# henuz yok, dolayisiyla duzeltme surekli olmali.
SLOT_UPDATE_INTERVAL_S = 5.0
# Otopilot gitmekte oldugu noktayi next_WP_loc'ta onbellege alir; konumdan
# turetilen tahminle arasinda pay birakilir. Bacak uzunlugunun orani.
SLOT_WRITE_MARGIN_RATIO = 0.10
# Bu esigin altindaki duzeltme icin yazmaya degmez; her tick yazmak MAVLink
# trafigini ve gorev deposunu bosuna yorar.
SLOT_UPDATE_DEADBAND_S = 1.5
# Ek sureyi saglayan mesafeyi ikiye bolerek arar; 8 adim 1 s'nin altina iner.
MANEUVER_SEARCH_ITERATIONS = 8
# Takip noktasi mesafesi. SITL plane modelinde WP_LOITER_RAD 80 m; takip
# noktasi bunun belirgin uzerinde tutulmazsa arac hedefi yakalayip cember
# atmaya basliyor ve manevra hic ilerlemiyor.
MANEUVER_LOOKAHEAD_M = 250.0
MANEUVER_COMPLETION_M = 50.0
# Manevra icin kalan rotanin en az bu kadar olmasi gerekir. Kisa mesafede
# istenen ek yol, bacak uzunlugunun yanina yaklasir ve zikzak son anda buyuk
# bir savrulmaya donusur: olculen kosuda 212 m kala 170 m ek mesafe istendi,
# 159 m yanal ofsetle arac hedefi 66 m ile isakaladi (sinir 5 m). Boyle bir
# durumda manevra etmemek etmekten iyidir.
MIN_MANEUVER_ROUTE_M = 800.0
# Ruzgar kestiriminin gecerli sayilmasi icin gereken en dusuk hava hizi;
# yerde ve kalkis kosusunda olculen degerler anlamsizdir.
MIN_WIND_ESTIMATE_AIRSPEED_MPS = 10.0
# Konum, yer hizi ve hava hizi 33 ms'de bir yayinlanir; bu esigi asan
# yayilim, konulardan birinin durdugu (AP_DDS yayini tikandigi) anlamina
# gelir ve o ornekten cikarilan ruzgar gercek degildir.
MAX_WIND_SAMPLE_SPREAD_S = 0.2
# Operasyonel bozucu zarfi: degisken dogrulama profili 4-10 m/s arasindadir;
# sifir ruzgar adayi da sakin regresyonu ayni hesaba dahil eder.
# Yon taramasi kapidaki karar icin bilinmeyen sabit yonleri kapsar; zamanla
# degisen kisim terminal rezerv bariyeriyle kapali cevrimde karsilanir.
# Dokumanda rüzgar icin sayisal/rate siniri olmadigindan bu deger formal bir
# tum-gelecek garantisi olarak sunulmaz.
DISTURBANCE_WIND_MAX_MPS = 10.0
# Siddet boyutu da taranir. Yalnizca 10 m/s'yi taramak, cok bacakli bir
# rotada toplam sure siddetle monoton olmadigi icin ara siddetleri kacirir.
DISTURBANCE_WIND_SPEED_STEP_MPS = 2.0
# DENENDI VE GERI ALINDI: zarfi olculen ruzgarin etrafinda bant olarak kurmak
# (blok 0'da +-2 m/s / +-30 derece, her blokta +3 m/s / +45 derece genisleyen).
# Ortak pencere doluluk %51'den %87'ye cikti ama zamanlama duzelmedi (-11.97 s)
# ve kapi cok daha sik tetiklendigi icin havada bekleme 44 s'den 115 s'ye
# firladi. Dayandigi teshis yanlisti: bagli kisit pencere kullanilabilirligi
# degil, hiz degisim limiti. Ayrica bant ihlal edildi; ruzgar kapi girisinden
# son yaklasmaya 109 derece dondu.
# Ruzgarin ayni kaldigi sure. Dogrulama profili 180 saniyede bir basamak
# degistiriyor (scripts/wind_profile.py). Zarfi tum rotaya TEK sabit ruzgar
# olarak uygulamak, "erken bacaklarda karsidan, son bacakta arkadan" gibi
# gercekte olan bilesimi imkansiz sayiyor ve yavaslama kapasitesini
# abartiyordu: HA-3 rotada 4532 m kala plan T+253 s iken L=436 s goruyordu.
# Rota bu sureye gore bloklara ayrilir; her blok kendi en kotu ruzgarini alir.
DISTURBANCE_COHERENCE_S = 180.0
# Ayni aday yon tum terminal suffix'ine uygulanir; bu, profili birebir
# tahmin etmek yerine ucuz ve deterministik bir operasyonel zarf verir.
DISTURBANCE_DIRECTION_STEP_DEG = 30.0
# Pencere siddet ve yon izgara taramasi yaptigi icin 20 Hz kontrol dongusunde
# her tick calistirmak gereksiz yuk. Sinirlar yavas degistigi icin saniyede
# bir yenilemek yeterli.
ROBUST_BOUNDS_INTERVAL_S = 1.0
# Ortak aralik bos kaldiginda uyari bu araliktan sik yazilmaz.


class MissionState(IntEnum):
    """VehicleStatus.STATE_* sabitleriyle ayni sayisal degerler."""

    INIT = 0
    CONNECTING = 1
    MISSION_UPLOAD = 2
    WAIT_PEERS = 3
    WAIT_TAKEOFF_SLOT = 4
    ARMING = 5
    TAKEOFF = 6
    CLIMB = 7
    CRUISE = 8
    TERMINAL = 9
    ARRIVED = 10
    RTL = 11
    DONE = 12
    FAILSAFE = 13


@dataclass(frozen=True)
class HoldGate:
    """Nominal rota uzerindeki son yasal loiter kapisi."""

    position: LatLon
    active_wp_index: int
    downstream_route: Tuple[LatLon, ...]
    terminal_earliest_s: float
    terminal_latest_s: float


# Ruzgar kestiriminin yapildigi durumlar. Tirmanis 200 saniyeyi bulabiliyor;
# ruzgar yalnizca seyirde olculseydi, yerde bekleyen araclar kalkis
# slotlarini duzeltemeden havalaniyordu.
AIRBORNE_STATES = frozenset({
    MissionState.TAKEOFF, MissionState.CLIMB,
    MissionState.CRUISE, MissionState.TERMINAL,
})

# Ulasilabilirlik yalnizca gorevi henuz tamamlamamis araclar icin anlamlidir.
# Varan bir arac capaya katilmayi surduremez: hedefe gitmedigi icin "en erken
# varis" degeri anlamsizlasir ve hala ucan araclari yanlis yonlendirir.
PRE_ARRIVAL_STATES = frozenset(
    state for state in MissionState if state < MissionState.ARRIVED
)


@dataclass(frozen=True)
class MissionSnapshot:
    """Durum makinesinin diger thread'lerden okunabilen anlik gorunumu."""

    state: MissionState
    target_reached: bool
    arrival_monotonic_ns: int
    arrival_min_distance_m: float
    arrival_interpolated: bool
    planned_arrival_monotonic_ns: int
    # Peer'lara yayinlanan deger budur: capa duzeltmesi iceren
    # calisma plani degil, taahhut edilmis nominal plan. Capa
    # zaten herkesin ayni veriden bagimsiz hesapladigi ortak bir
    # kaydirmadir; taahhut uzerinden tasinirsa geri beslenir.
    committed_plan_monotonic_ns: int
    arrival_committed: bool
    earliest_feasible_arrival_monotonic_ns: int
    max_route_deviation_m: float
    eta_s: float
    remaining_distance_m: float
    active_wp_index: int
    commanded_airspeed_mps: float
    wind_valid: bool
    wind_speed_mps: float
    wind_from_direction_deg: float
    # Secilen operasyonel ruzgar zarfindaki varis penceresi.
    robust_earliest_s: float
    robust_latest_s: float
    # Mutlak anlar: peer'lara bunlar yayinlanir, goreli saniyeler yalnizca log icin.


class MissionManager:
    """Kalkistan RTL'e kadar gorev akisini yuruten durum makinesi."""

    def __init__(
        self,
        config: VehicleConfig,
        commander,
        telemetry,
        peer_commitments: Optional[Callable[[int], Dict[int, int]]] = None,
        peer_feasible_arrivals: Optional[Callable[[int], Dict[int, int]]] = None,
        guided_commander=None,
        peer_wind: Optional[Callable[[int], Optional[Tuple[float, float]]]] = None,
    ) -> None:
        self._guided = guided_commander
        self._peer_wind = peer_wind or (lambda _now_ns: None)
        self._config = config
        self._commander = commander
        self._telemetry = telemetry
        # Peer taahhutlerini saglayan geri cagri; tek arac testlerinde bos.
        self._peer_commitments = peer_commitments or (lambda _now_ns: {})
        self._peer_feasible_arrivals = peer_feasible_arrivals or (lambda _now_ns: {})

        self._nominal_flight_s = (
            route_length_m(config.home, config.route) / config.nominal_cruise_speed_mps
        )
        self._state = MissionState.INIT
        self._detector = ArrivalDetector(config.target)
        self._estimator = EtaEstimator(config.route, config.home)
        self._controller = ArrivalController(
            config.nominal_cruise_speed_mps,
            config.min_airspeed_mps,
            config.max_airspeed_mps,
            config.airspeed_rate_limit_mps2,
            config.timing_deadband_s,
        )
        # Kapi kurulumu E/L hesaplar, o da olculen ruzgari sorar; bu alan
        # kapidan once tanimli olmali. Yalnizca filtre oturduktan sonra
        # doldurulur, oturmamis kestirim ne planda ne peer yayininda kullanilir.
        self._wind: Optional[WindEstimate] = None
        self._hold_gate = self._build_hold_gate()
        self._terminal_entry = last_circle_entry_on_route(
            config.home, config.route, config.target, TERMINAL_RADIUS_M
        )
        self._planned_arrival_ns = 0
        self._arrival_committed = False
        self._takeoff_time_ns = 0
        self._next_peer_log = 0.0
        self._next_gate_log = 0.0
        self._eta_s = 0.0
        self._remaining_distance_m = 0.0
        self._active_wp_index = 0
        self._last_control_ns = 0
        self._max_route_deviation_m = 0.0
        self._deviation_warned = False
        self._progress_speed_mps = 0.0
        self._feasible_arrival_ns = 0
        # Taahhut aninda sabitlenen plan; capa kaymasi daima buna gore
        # olculur, bir onceki kaymaya gore degil. Aksi halde her tick'teki
        # kucuk kaymalar birikip plani sonsuza kadar ileri itiyor.
        self._nominal_plan_ns = 0
        self._maneuver_path: Tuple[LatLon, ...] = ()
        self._maneuver_index = 0
        self._maneuver_attempted = False
        self._trigger_streak = 0
        # Yuvalara en son yazilan konumlar; hangi yuvanin aracin onunde
        # kaldigini bacak uzerindeki izdusumden hesaplamak icin saklanir.
        self._slot_positions: Tuple[LatLon, ...] = ()
        self._next_slot_write = 0.0
        self._loitering = False
        self._gate_crossed = self._hold_gate is None
        self._gate_hold_used = False
        self._gate_bounds_refreshed = False
        self._reserve_mode: Optional[str] = None
        self._robust_earliest_s = 0.0
        self._robust_latest_s = 0.0
        self._robust_bounds_ns = 0
        self._wind_filter = WindFilter()
        self._last_wind_sample_ns = 0
        # Gercek kalkis ani; ruzgar sonradan ogrenilince plan
        # bu ana gore yeniden hesaplanir.
        self._takeoff_actual_ns = 0
        # Oncu planini yalnizca bir kez revize eder. Her tick'te
        # yeniden hesaplanirsa gurultulu ruzgar olcumu tek yonde
        # birikip plani sonsuza kadar ileri itiyor.
        self._plan_revised = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def nominal_flight_s(self) -> float:
        return self._nominal_flight_s

    @property
    def state(self) -> MissionState:
        with self._lock:
            return self._state

    def snapshot(self) -> MissionSnapshot:
        with self._lock:
            actual_arrival_ns = self._detector.arrival_monotonic_ns or 0
            arrived = self._state >= MissionState.ARRIVED and actual_arrival_ns > 0
            return MissionSnapshot(
                state=self._state,
                target_reached=self._detector.arrived,
                arrival_monotonic_ns=actual_arrival_ns,
                arrival_min_distance_m=self._detector.min_distance_m,
                arrival_interpolated=self._detector.interpolated,
                planned_arrival_monotonic_ns=self._planned_arrival_ns,
                committed_plan_monotonic_ns=self._nominal_plan_ns,
                arrival_committed=self._arrival_committed,
                earliest_feasible_arrival_monotonic_ns=self._feasible_arrival_ns,
                max_route_deviation_m=self._max_route_deviation_m,
                eta_s=self._eta_s,
                remaining_distance_m=self._remaining_distance_m,
                active_wp_index=self._active_wp_index,
                commanded_airspeed_mps=self._controller.commanded_airspeed_mps,
                # Ruzgar yalnizca havadayken guncellenir; varistan sonra
                # deger donar. Gecerli isaretlenirse peer'lar donmus bir
                # olcumu taze sanar ve en kucuk id tercihi yuzunden inmis
                # araci kaynak secebilir.
                # self.state property'si ayni kilidi alir; snapshot zaten
                # kilit altinda oldugu icin alan dogrudan okunur.
                wind_valid=self._wind is not None and self._state in AIRBORNE_STATES,
                wind_speed_mps=self._wind.speed_mps if self._wind else 0.0,
                wind_from_direction_deg=(
                    self._wind.from_direction_deg if self._wind else 0.0
                ),
                robust_earliest_s=0.0 if arrived else self._robust_earliest_s,
                robust_latest_s=0.0 if arrived else self._robust_latest_s,
            )

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="mission_manager")
        self._thread.start()

    def stop(self, join_timeout_s: float = 5.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=join_timeout_s)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self.step()
                # Okunmayan MAVLink mesajlari birikirse SITL'in ana dongusu
                # tikaniyor ve AP_DDS yayini duruyor.
                if self._commander.connected:
                    self._commander.drain()
            except Exception:  # noqa: BLE001 - durum makinesi thread'i olmemeli
                logger.exception("gorev adimi basarisiz, failsafe'e geciliyor")
                self._transition(MissionState.FAILSAFE)
            self._stop.wait(TICK_INTERVAL_S)

    def _transition(self, new_state: MissionState) -> None:
        with self._lock:
            if self._state == new_state:
                return
            previous = self._state
            self._state = new_state
        logger.info("durum: %s -> %s", previous.name, new_state.name)

    def step(self) -> None:
        """Tek bir durum makinesi adimi. Testlerden dogrudan cagrilabilir."""
        if self.state in AIRBORNE_STATES:
            self._update_wind(self._telemetry.snapshot())
        # Prospektif sinirlar yerde de hesaplanir; ucuz kaldirac olan kalkis
        # gecikmesi ancak arac havalanmadan once bilgi varsa kullanilabilir.
        if self.state in PRE_ARRIVAL_STATES:
            self._refresh_robust_bounds()
        if self._arrival_committed:
            # Yerde de calismali: oncu tirmanista ruzgari ogrenip planini
            # ileri cekince, henuz kalkmamis takipciler bunu izleyip kendi
            # slotlarini guncelleyebilmeli.
            now_ns = time.monotonic_ns()
            self._revise_plan(now_ns)
            # Ulasilabilirlik ve capa da yerde islemeli: yerdeki aracin
            # kalkis slotu _planned_arrival_ns'den turetildigi icin capa
            # oraya ulastiginda duzeltme bedava (havada bekleme yok).
            self._update_feasible_arrival(now_ns)
            if self.state in PRE_ARRIVAL_STATES:
                self._apply_feasible_anchor(now_ns)
        handler = _HANDLERS.get(self.state)
        if handler is not None:
            handler(self)

    # --- durum isleyicileri ---

    def _on_init(self) -> None:
        self._transition(MissionState.CONNECTING)

    def _on_connecting(self) -> None:
        if not self._commander.connected and not self._commander.connect():
            return
        if not self._commander.wait_gps_ready():
            return
        if not self._telemetry.snapshot().valid:
            return
        self._transition(MissionState.MISSION_UPLOAD)

    def _on_mission_upload(self) -> None:
        mission = build_mission(
            self._config.home,
            self._config.route,
            self._config.cruise_alt_msl_m,
            self._config.takeoff_alt_msl_m,
            self._config.wp_accept_radius_m,
            self._config.s_maneuver_enabled,
        )
        if self._commander.upload_mission(mission):
            # Yuvalar gorevde zaten var (bacak dogrusu uzerinde). Konumlari
            # buradan alinmazsa surekli guncelleme hic calisamaz: tek
            # seferlik yazmayi bekler, o da arac karar aninda zamanindayken
            # tetiklenmez. Olculdu, mekanizma bu yuzden bir kosu boyunca atil
            # kaldi.
            if self._config.s_maneuver_enabled:
                first_slot, last_slot = spare_slot_range(self._config.route)
                with self._lock:
                    self._slot_positions = tuple(
                        LatLon(item.lat, item.lon)
                        for item in mission[first_slot:last_slot + 1]
                    )
            self._transition(MissionState.WAIT_PEERS)

    def _known_wind(self, now_ns: int) -> Optional[WindEstimate]:
        """Kendi kestirimi oturmussa onu, yoksa bir peer'inkini kullanir.

        Kalkis aninda kendi ham olcumune atlanmaz: filtre oturana kadar
        peer'in oturmus degeri kullanilir. Aksi halde plan tam kurulurken
        tek ornekten cikan sapmali bir ruzgara dayanirdi.
        """
        if self._wind is not None:
            return self._wind
        wind_pair = self._peer_wind(now_ns)
        return wind_from_speed_direction(*wind_pair) if wind_pair else None

    def _refresh_nominal_flight_time(self, now_ns: int) -> None:
        """Nominal ucus suresini bilinen ruzgara gore duzeltir.

        Ruzgarsiz hesaplanan sure, ruzgar altinda ulasilamaz bir plan
        uretir; plan yanlis kurulunca hata havadayken duzeltilemeyecek kadar
        gec fark ediliyor. Ilk kalkan arac ruzgar sondasi gorevi gorur.
        """
        wind = self._known_wind(now_ns)
        if wind is None:
            return
        wind_pair = (wind.speed_mps, wind.from_direction_deg)
        corrected_s = route_duration_with_wind_s(
            self._config.home, self._config.route,
            self._config.nominal_cruise_speed_mps, wind,
        )
        if math.isclose(corrected_s, self._nominal_flight_s, rel_tol=0.01):
            return

        logger.info(
            "nominal ucus suresi ruzgara gore duzeltildi: %.0f s -> %.0f s "
            "(peer olcumu %.1f m/s, %.0f dereceden)",
            self._nominal_flight_s, corrected_s, wind_pair[0], wind_pair[1],
        )
        self._nominal_flight_s = corrected_s

    def _on_wait_peers(self) -> None:
        """Referans varis anini belirler ve kendi planini taahhut eder."""
        now_ns = time.monotonic_ns()
        self._refresh_nominal_flight_time(now_ns)
        reference = compute_reference_arrival(
            self._config.vehicle_id, self._peer_commitments(now_ns)
        )

        if reference.resolved:
            self._commit(reference.monotonic_ns, f"peer {reference.source_vehicle_ids}")
        elif self._is_leader():
            # Oncu arac taahhudunu burada degil, gercekten kalktigi anda verir.
            # Ilk arm isleminde EKF oturmasi 15 saniyeye kadar surebiliyor ve bu
            # sure plana girerse takipciler oncuyle ayni anda kalkiyor.
            with self._lock:
                self._takeoff_time_ns = now_ns
        else:
            self._log_peer_wait()
            return

        self._transition(MissionState.WAIT_TAKEOFF_SLOT)

    def _is_leader(self) -> bool:
        return self._config.vehicle_id == 1

    def _build_hold_gate(self) -> Optional[HoldGate]:
        """Nominal rotanin 2.5 km guvenlik cemberine son girisini kurar."""
        found = last_circle_entry_on_route(
            self._config.home,
            self._config.route,
            self._config.target,
            MIN_LOITER_DISTANCE_M,
        )
        if found is None:
            return None
        position, active_wp_index = found
        downstream = self._config.route[active_wp_index:]
        earliest_s, latest_s = self._bounds_for_route_s(
            position, downstream, self._config.nominal_cruise_speed_mps
        )
        return HoldGate(
            position=position,
            active_wp_index=active_wp_index,
            downstream_route=downstream,
            terminal_earliest_s=earliest_s,
            terminal_latest_s=latest_s,
        )

    def _log_peer_wait(self) -> None:
        if time.monotonic() < self._next_peer_log:
            return
        self._next_peer_log = time.monotonic() + PEER_WAIT_LOG_INTERVAL_S
        logger.info("onceki araclarin taahhudu bekleniyor")

    def _commit(self, planned_arrival_ns: int, source: str) -> None:
        """Varis planini kilitler; peer'lar yalnizca bu degeri referans alir."""
        takeoff_ns = compute_takeoff_time(planned_arrival_ns, self._nominal_flight_s)
        with self._lock:
            self._planned_arrival_ns = planned_arrival_ns
            self._nominal_plan_ns = planned_arrival_ns
            self._arrival_committed = True
            self._takeoff_time_ns = takeoff_ns

        delay_s = (takeoff_ns - time.monotonic_ns()) / 1e9
        logger.info(
            "varis plani taahhut edildi (%s) | nominal ucus %.0f s | "
            "yerde bekleme %.1f s",
            source, self._nominal_flight_s, max(delay_s, 0.0),
        )
        # Oncu aracin kalkis ani tanimi geregi "simdi"dir; kucuk negatif
        # degerler normaldir ve uyari uretmemelidir.
        if delay_s < -LATE_TAKEOFF_TOLERANCE_S:
            logger.warning(
                "kalkis slotu %.1f s gecmiste; hemen kalkiliyor, fark hiz "
                "kontroluyle kapatilmali", -delay_s,
            )

    def _on_wait_takeoff_slot(self) -> None:
        # Yerde beklerken ruzgar olcumu gelebilir; slot ona gore guncellenir.
        # Taahhut ani cok erken oldugu icin (oncu daha havalanmadan) duzeltme
        # yalnizca WAIT_PEERS'te yapilsaydi hicbir zaman uygulanmazdi.
        self._resync_takeoff_slot(time.monotonic_ns())
        if time.monotonic_ns() >= self._takeoff_time_ns:
            self._transition(MissionState.ARMING)

    def _resync_takeoff_slot(self, now_ns: int) -> None:
        """Kalkis anini guncel plan ve ruzgar duzeltmesine gore yeniden kurar.

        Hem nominal sure hem de plan yerde beklerken degisebiliyor; slot her
        adimda ikisinden yeniden turetilir.
        """
        previous_s = self._nominal_flight_s
        self._refresh_nominal_flight_time(now_ns)

        with self._lock:
            new_takeoff_ns = compute_takeoff_time(
                self._planned_arrival_ns, self._nominal_flight_s
            )
            changed_s = abs(new_takeoff_ns - self._takeoff_time_ns) / 1e9
            self._takeoff_time_ns = new_takeoff_ns
            delay_s = (new_takeoff_ns - now_ns) / 1e9

        if changed_s >= ANCHOR_LOG_THRESHOLD_S or not math.isclose(
            previous_s, self._nominal_flight_s, rel_tol=1e-9
        ):
            logger.info(
                "kalkis slotu guncellendi | nominal ucus %.0f s | yerde kalan bekleme %.1f s",
                self._nominal_flight_s, max(delay_s, 0.0),
            )

    def _on_arming(self) -> None:
        if not self._commander.set_mode("AUTO"):
            return
        if not self._commander.arm():
            return
        takeoff_ns = time.monotonic_ns()
        with self._lock:
            self._takeoff_actual_ns = takeoff_ns
        if not self._arrival_committed:
            self._commit(
                takeoff_ns + int(self._nominal_flight_s * NANOSECONDS_PER_SECOND),
                "kalkis ani",
            )
        self._transition(MissionState.TAKEOFF)

    def _on_takeoff(self) -> None:
        if self._altitude_reached(self._config.takeoff_alt_msl_m):
            self._transition(MissionState.CLIMB)

    def _on_climb(self) -> None:
        if self._altitude_reached(self._config.cruise_alt_msl_m):
            self._transition(MissionState.CRUISE)

    def _on_cruise(self) -> None:
        position = self._track_arrival()
        if position is None:
            return
        if self._handle_hold_gate(position):
            return
        self._coordinate_and_regulate()
        if self._at_final_terminal_entry(position):
            self._transition(MissionState.TERMINAL)

    def _at_final_terminal_entry(self, position: LatLon) -> bool:
        """Rota 2 km cemberine birden cok giriyorsa yalniz son girisi sec."""
        if self._terminal_entry is None:
            return geodesic_distance_m(position, self._config.target) <= TERMINAL_RADIUS_M
        _entry_position, active_wp_index = self._terminal_entry
        if self._active_wp_index > active_wp_index:
            return True
        return (
            self._active_wp_index == active_wp_index
            and geodesic_distance_m(position, self._config.target) <= TERMINAL_RADIUS_M
        )

    def _gate_release_window(self) -> Optional[Tuple[int, int]]:
        if self._hold_gate is None:
            return None
        return compute_gate_release_window(
            self._planned_arrival_ns,
            self._hold_gate.terminal_earliest_s,
            self._hold_gate.terminal_latest_s,
            GATE_EARLY_MARGIN_S,
            GATE_LATE_MARGIN_S,
        )

    def _update_gate_crossing(self, position: LatLon) -> None:
        """AUTO ile son guvenli kapinin gecildigini tek yonlu mandallar."""
        gate = self._hold_gate
        if gate is None or self._gate_crossed or self._loitering:
            return
        inside = geodesic_distance_m(position, self._config.target) < (
            MIN_LOITER_DISTANCE_M - GATE_CROSSING_HYSTERESIS_M
        )
        # Rota cembere daha once girip yeniden cikabilir. Geometri son girisi
        # sectigi icin radyal kontrol ancak secilen bacak aktifken anlamlidir.
        selected_leg_inside = (
            self._active_wp_index == gate.active_wp_index and inside
        )
        if self._active_wp_index > gate.active_wp_index or selected_leg_inside:
            with self._lock:
                self._gate_crossed = True
            logger.info("SON BEKLEME KAPISI GECILDI | terminalde loiter kilitlendi")

    def _eta_to_gate_s(self, position: LatLon) -> Optional[float]:
        gate = self._hold_gate
        if gate is None or self._active_wp_index > gate.active_wp_index:
            return None
        route_to_gate = (
            *self._config.route[self._active_wp_index:gate.active_wp_index],
            gate.position,
        )
        wind = self._known_wind(time.monotonic_ns()) or WindEstimate(0.0, 0.0)
        return route_duration_with_wind_s(
            position, route_to_gate, self._controller.commanded_airspeed_mps, wind
        )

    def _refresh_gate_bounds_for_entry(self) -> None:
        """Secilen bacaga gelince E/L'yi olculen hizdan bir kez yenile."""
        gate = self._hold_gate
        if (
            gate is None
            or self._gate_bounds_refreshed
            or self._gate_crossed
            or self._active_wp_index != gate.active_wp_index
        ):
            return
        earliest_s, latest_s = self._bounds_for_route_s(
            gate.position,
            gate.downstream_route,
            self._live_initial_airspeed(self._telemetry.snapshot()),
        )
        self._hold_gate = HoldGate(
            position=gate.position,
            active_wp_index=gate.active_wp_index,
            downstream_route=gate.downstream_route,
            terminal_earliest_s=earliest_s,
            terminal_latest_s=latest_s,
        )
        self._gate_bounds_refreshed = True

    def _handle_hold_gate(self, position: LatLon) -> bool:
        """Son yasal rota kapisinda en fazla bir kez kontrollu loiter uygular."""
        self._update_gate_crossing(position)
        self._refresh_gate_bounds_for_entry()
        gate = self._hold_gate
        if (
            gate is None
            or not self._config.loiter_enabled
            or self._guided is None
            or self._gate_crossed
        ):
            return False

        window = self._gate_release_window()
        if window is None:
            return False
        lower_ns, upper_ns = window
        if lower_ns > upper_ns:
            if self._loitering:
                self._exit_gate_hold("kapi penceresi bosaldi")
            self._log_gate_problem(
                "KAPI PENCERESI BOS: terminal hiz yetkisi secilen zarf ve "
                f"marjlar icin yetersiz ({(lower_ns - upper_ns) / 1e9:.1f} s)"
            )
            return self._loitering

        now_ns = time.monotonic_ns()
        release_ns = upper_ns
        if self._loitering:
            # /ap/cmd_gps_pose BEST_EFFORT'tur; tek ornek kaybolursa arac
            # eski GUIDED hedefine gidebilir. S takipcisi gibi her tick yenile.
            self._guided.send(gate.position, self._config.cruise_alt_msl_m)
            target_distance_m = geodesic_distance_m(position, self._config.target)
            route_deviation_m = self._route_deviation_from_nominal(position)
            if (
                target_distance_m <= GATE_TARGET_DISTANCE_ABORT_M
                or route_deviation_m >= GATE_ROUTE_DEVIATION_ABORT_M
            ):
                self._exit_gate_hold(
                    "guvenlik payi azaldi "
                    f"(hedef {target_distance_m:.0f} m, sapma {route_deviation_m:.0f} m)"
                )
                return self._loitering
            if now_ns >= release_ns:
                self._exit_gate_hold("rezervli cikis ani geldi")
                return self._loitering
            return True

        if self._gate_hold_used or self._active_wp_index != gate.active_wp_index:
            return False
        eta_to_gate_s = self._eta_to_gate_s(position)
        if eta_to_gate_s is None:
            return False
        predicted_crossing_ns = now_ns + int(eta_to_gate_s * NANOSECONDS_PER_SECOND)
        if predicted_crossing_ns > upper_ns:
            self._log_gate_problem(
                "KAPI PENCERESI KACIRILDI: tahmini gecis ust sinirdan "
                f"{(predicted_crossing_ns - upper_ns) / 1e9:.1f} s sonra"
            )
            return False
        if predicted_crossing_ns >= release_ns - int(
            GATE_HOLD_TRIGGER_S * NANOSECONDS_PER_SECOND
        ):
            return False

        if not self._commander.set_mode("GUIDED"):
            self._log_gate_problem("SON BEKLEME KAPISINDA GUIDED moda gecilemedi")
            return False
        self._guided.send(gate.position, self._config.cruise_alt_msl_m)
        with self._lock:
            self._loitering = True
            self._gate_hold_used = True
        logger.info(
            "KAPI LOITER BASLADI | hedefe %.0f m | planlanan cikisa %.1f s | "
            "terminal E %.1f L %.1f s",
            geodesic_distance_m(gate.position, self._config.target),
            max((release_ns - now_ns) / 1e9, 0.0),
            gate.terminal_earliest_s,
            gate.terminal_latest_s,
        )
        return True

    def _exit_gate_hold(self, reason: str) -> None:
        if not self._commander.set_mode("AUTO"):
            logger.error("KAPI LOITER bitirilemedi: AUTO moda donulemedi")
            return
        with self._lock:
            self._loitering = False
            # Cikistan sonra ayni kapida ikinci kez donulmez; bu hem havada
            # beklemeyi hem de ortak zamanlama salinimini sinirlar.
            self._gate_hold_used = True
        logger.info("KAPI LOITER BITTI | %s", reason)

    def _log_gate_problem(self, message: str) -> None:
        if time.monotonic() < self._next_gate_log:
            return
        self._next_gate_log = time.monotonic() + GATE_LOG_INTERVAL_S
        logger.warning(message)

    def _model_eta_s(self, position: LatLon) -> float:
        """Kalan rotanin komut edilen hava hiziyla ve ruzgar altinda suresi.

        Kalan mesafeyi olculen ilerleme hizina bolmek, o hizin rotanin geri
        kalaninda da gecerli olacagini varsayar. 8 m/s ruzgarda bacaklar
        farkli yonlere baktigi icin yer hizi 15.9-18.8 arasinda gercekten
        degisiyor; ETA her donuste ~30 saniye ziplyor ve zamanlama hatasi
        +-15 s salinyordu. Salinan bir hataya kontrolcu kalici duzeltme
        uygulayamaz: HA-3, 79 saniyelik yavaslama yetkisi oldugu halde
        asgari hava hizina hic inmiyordu.

        Model her bacagin kendi ruzgar bilesenini ayri hesaplar ve olculen
        hiz terimi icermedigi icin pürüzsüzdur.
        """
        wind = self._known_wind(time.monotonic_ns()) or WindEstimate(
            east_mps=0.0, north_mps=0.0
        )
        # Manevra sirasinda arac orijinal rotayi degil uretilen yorungeyi
        # uctugu icin ETA da ondan hesaplanmali; aksi halde sistem kendi
        # ekledigi yolu gormez ve zamanlamayi yanlis degerlendirir.
        remaining = (
            # segment_index mevcut segmentin BASLANGIC indeksidir. Onu tekrar
            # eklemek aracin geriye gidip segmenti yeniden ucacagini varsayar
            # ve ETA'yi sisirir; mevcut konumdan segment sonuna devam edilir.
            self._maneuver_path[self._maneuver_index + 1:]
            if self._maneuver_path
            else self._config.route[self._active_wp_index:]
        )
        return route_duration_with_wind_s(
            position, remaining, self._controller.commanded_airspeed_mps, wind
        )

    def _on_terminal(self) -> None:
        position = self._track_arrival()
        if position is None:
            return
        # Genel bir rota 2 km cemberine girip yeniden cikabilir. TERMINAL
        # durumu tek yonlu oldugu icin son yasal kapinin isleyicisi burada da
        # calisir; secilen son rota girisi boylece atlanmaz.
        if self._handle_hold_gate(position):
            return
        self._coordinate_and_regulate()

        if self._maneuver_path:
            self._fly_maneuver(position)
        else:
            self._consider_s_maneuver(position)
        # Son bacaga girdikten sonra da ilerideki yuvalar guncellenebilir;
        # bu, tek seferlik karari kapali cevrime cevirir.
        self._update_maneuver_slots(position)

    def _consider_s_maneuver(self, position: LatLon) -> None:
        """Hiz yetkisi tukendiyse yorunge uzatma manevrasi planlar.

        Dokuman hedefin 2 km cevresinde loiter'i yasakladigi icin terminal
        fazda tek secenek S-manevrasidir.
        """
        if not self._config.s_maneuver_enabled:
            return
        if self._maneuver_attempted:
            return
        # Yuvalar son bacakta duruyor; arac o bacaga girdikten sonra yazmak
        # etkisiz kalir cunku otopilot gitmekte oldugu noktayi next_WP_loc'ta
        # onbellege alir. Karar bir onceki bacakta verilmeli.
        if self._active_wp_index >= len(self._config.route) - 1:
            return

        early_s = self._timing_error_s()
        at_min_airspeed = math.isclose(
            self._controller.commanded_airspeed_mps, self._config.min_airspeed_mps, abs_tol=0.2
        )
        if early_s > -S_MANEUVER_TRIGGER_S or not at_min_airspeed:
            self._trigger_streak = 0
            return

        self._trigger_streak += 1
        if self._trigger_streak < S_MANEUVER_CONFIRM_TICKS:
            return

        if self._remaining_distance_m < MIN_MANEUVER_ROUTE_M:
            # Tek sefer uyarilir; tekrar denemenin anlami yok.
            self._maneuver_attempted = True
            logger.warning(
                "S-manevrasi atlandi: kalan rota %.0f m (en az %.0f m gerekli). "
                "Erkenlik %.1f s bu mesafede kapatilamaz.",
                self._remaining_distance_m, MIN_MANEUVER_ROUTE_M, -early_s,
            )
            return

        self._maneuver_attempted = True
        # Manevra son bacaga hapsedilir: yuvalar orada duruyor ve dokumanin
        # loiter yasakladigi bolge de orasi. Bacak poligonu kullanilir, duz
        # cizgi degil; aksi halde sapma manevradan degil rotanin kesilmesinden
        # gelir (olculen: 91 m planlanan ofsete karsi 812 m gercek sapma).
        # Donus yaricapi ASGARI hizdan hesaplanirsa arac o donusleri yapamaz ve
        # daha genis yay cizer. Olculdu: 13 m/s varsayilip 28 m/s uculdugunda
        # ek mesafe 407 m yerine 489 m, sapma 138 m yerine 165 m cikti.
        found = self._maneuver_for_delay(
            self._config.route[-2], self._config.route[-1], -early_s, S_SLOT_COUNT
        )
        if found is None:
            logger.warning(
                "S-manevrasi uretilemedi: %.1f s gecikme 500 m sapma ve donus "
                "yaricapi kisitlari altinda saglanamiyor", -early_s,
            )
            return
        maneuver, added_s = found

        if not self._write_maneuver_slots(maneuver):
            return

        with self._lock:
            self._maneuver_path = maneuver.waypoints
            self._maneuver_index = 0

        # Raporlanan degerler uretilen yorungeden olculmustur, istenen
        # degerler degil; boylece 500 m denetimi ucusla ayni referansi kullanir.
        logger.info(
            "S-MANEVRASI BASLADI | kazanilan %.1f s (istenen %.1f) | %d dongu | "
            "ek %.0f m | rota sapmasi %.0f m (sinir %.0f m)",
            added_s, -early_s, maneuver.cycles,
            maneuver.planned_extra_distance_m,
            maneuver.max_route_deviation_m, MAX_ROUTE_DEVIATION_M,
        )

    def _maneuver_for_delay(
        self, start: LatLon, end: LatLon, delay_s: float, max_cycles: int
    ):
        """Istenen ek SUREYI saglayan en buyuk guvenli manevrayi arar.

        Ek mesafeden gitmek ruzgar altinda bozuluyor: S'in yanal bacaklari
        farkli yonlere baktigi icin ayni mesafe cok farkli sureler tutar.
        Olculdu: 242 m'lik ek yolun bir bacagi ruzgara donunce yer hizi
        3.5 m/s'ye dustu, beklenen 11 s'lik gecikme 77 s oldu ve varis
        sirasi bozuldu.

        Arama bilerek EKSIK teslime yanlidir: kalan erkenligi hiz kontrolcusu
        kismen kapatabilir, fazla gecikmenin ise geri donusu yoktur.
        """
        wind = self._known_wind(time.monotonic_ns()) or WindEstimate(0.0, 0.0)
        airspeed_mps = self._controller.commanded_airspeed_mps
        direct_s = route_duration_with_wind_s(start, (end,), airspeed_mps, wind)
        radius_m = turn_radius_m(airspeed_mps, MANEUVER_BANK_ANGLE_DEG)

        best = None
        low_m = 0.0
        high_m = delay_s * self._config.max_airspeed_mps
        for _ in range(MANEUVER_SEARCH_ITERATIONS):
            mid_m = 0.5 * (low_m + high_m)
            candidate = plan_s_maneuver(
                (start, end), mid_m, radius_m,
                max_lateral_offset_m=MANEUVER_PLAN_LATERAL_LIMIT_M,
                max_cycles=max_cycles,
            )
            if candidate is None:
                high_m = mid_m
                continue
            added_s = route_duration_with_wind_s(
                start, candidate.waypoints[1:], airspeed_mps, wind
            ) - direct_s
            if added_s <= delay_s:
                best = (candidate, added_s)
                low_m = mid_m
            else:
                high_m = mid_m
        return best

    def _leg_progress_fraction(self, position: LatLon) -> float:
        """Aracin son bacak uzerindeki ilerleme orani (0 basta, 1 sonda)."""
        leg_start, leg_end = self._config.route[-2], self._config.route[-1]
        sx, sy = to_local_xy(leg_start, leg_start)
        ex, ey = to_local_xy(leg_end, leg_start)
        px, py = to_local_xy(position, leg_start)
        dx, dy = ex - sx, ey - sy
        length_sq = dx * dx + dy * dy
        if length_sq <= 0.0:
            return 1.0
        return max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / length_sq))

    def _writable_slot_count(self, position: LatLon) -> int:
        """Aracin henuz gecmedigi, guvenle yazilabilecek yuva sayisi."""
        if not self._slot_positions:
            return 0
        limit = self._leg_progress_fraction(position) + SLOT_WRITE_MARGIN_RATIO
        return sum(
            1 for slot in self._slot_positions
            if self._leg_progress_fraction(slot) > limit
        )

    def _update_maneuver_slots(self, position: LatLon) -> None:
        """Son bacakta, ilerideki yuvalari guncel erkenlige gore yeniden yazar.

        Tek seferlik karar yetmiyordu: erkenlik son bacak icinde, ruzgar
        donunce doguyor ve karar aninda henuz yok. Otopilotun onbellege
        aldigi ogeye dokunulmaz, yalnizca ilerideki yuvalar degistirilir.
        """
        if not self._config.s_maneuver_enabled or not self._slot_positions:
            return
        if self._active_wp_index < len(self._config.route) - 1:
            return

        now = time.monotonic()
        if now < self._next_slot_write:
            return

        early_s = -self._timing_error_s()
        if early_s <= SLOT_UPDATE_DEADBAND_S:
            return
        writable = self._writable_slot_count(position)
        if writable <= 0:
            return

        leg_end = self._config.route[-1]
        found = self._maneuver_for_delay(position, leg_end, early_s, writable)
        self._next_slot_write = now + SLOT_UPDATE_INTERVAL_S
        if found is None:
            logger.warning(
                "S guncellemesi uretilemedi: %.1f s erkenlik, kalan %d yuva",
                early_s, writable,
            )
            return
        maneuver, added_s = found

        points = list(maneuver.waypoints[1:-1])[:writable]
        if not points:
            return
        first_slot, _ = spare_slot_range(self._config.route)
        start = first_slot + (S_SLOT_COUNT - writable)
        if not self._write_slot_items(start, points):
            return

        with self._lock:
            self._slot_positions = (
                self._slot_positions[: S_SLOT_COUNT - writable] + tuple(points)
            )
            self._maneuver_path = maneuver.waypoints
            self._maneuver_index = 0
        logger.info(
            "S YUVALARI GUNCELLENDI | %.1f s erkenlik | %d yuva | "
            "kazanilan %.1f s | ek %.0f m | sapma %.0f m",
            early_s, len(points), added_s, maneuver.planned_extra_distance_m,
            maneuver.max_route_deviation_m,
        )

    def _write_maneuver_slots(self, maneuver) -> bool:
        """S noktalarini son bacaktaki bos gorev yuvalarina yazar.

        GUIDED kullanilmaz: ModeGuided::navigate update_loiter cagirir ve
        set_guided_WP crosstrack'i kapatir, yani noktalar arasi yol takibi
        yoktur. Iki denemede de arac noktalarda takilip zamanlamayi bozdu.
        AUTO'da crosstrack acik oldugu icin L1 yorungeyi gercekten izler.
        """
        first_slot, _ = spare_slot_range(self._config.route)
        points = list(maneuver.waypoints[1:-1])
        if len(points) > S_SLOT_COUNT:
            logger.warning(
                "S-manevrasi atlandi: %d nokta %d yuvaya sigmiyor",
                len(points), S_SLOT_COUNT,
            )
            return False
        # Artan yuvalar hedefe yakin, bacak dogrusu uzerinde birakilir; oradan
        # gecmek yorungeyi degistirmez. Oran 1.0'a ULASMAMALI: hedefin uzerine
        # dusen bir yuva, hedefin dar kabul yaricapini 20 m'lik yuvayla
        # golgeler ve arac 5 m'ye girmeden gorevi tamamlanmis sayar.
        leg_start, leg_end = self._config.route[-2], self._config.route[-1]
        bos = S_SLOT_COUNT - len(points)
        for index in range(bos):
            points.append(
                point_along_leg(leg_start, leg_end, 0.90 + 0.05 * (index + 1) / bos)
            )

        if not self._write_slot_items(first_slot, points):
            return False
        with self._lock:
            self._slot_positions = tuple(points)
        return True

    def _write_slot_items(self, start_index: int, points) -> bool:
        """Verilen konumlari gorev yuvalarina yazar."""
        items = [
            MissionItem(
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                point.lat, point.lon, self._config.cruise_alt_msl_m,
                param2=S_SLOT_ACCEPT_RADIUS_M,
            )
            for point in points
        ]
        if not self._commander.write_mission_slots(start_index, items):
            logger.error("S-manevrasi yuvalari yazilamadi")
            return False
        return True

    def _fly_maneuver(self, position: LatLon) -> None:
        """Manevra boyunca ilerlemeyi izler; komut vermez.

        Yorungeyi otopilot AUTO gorevinden ucar. Buradaki takip yalnizca ETA
        modeli icindir: kalan yol manevra poligonundan olculmezse kontrolcu
        aracin gecikeecegini gorup hizlanir ve manevranin kazandirdigi zamani
        geri harcar.
        """
        state = follow_path(
            self._maneuver_path, position, MANEUVER_LOOKAHEAD_M, self._maneuver_index
        )
        if state is None:
            self._finish_maneuver("takip noktasi hesaplanamadi")
            return

        with self._lock:
            self._maneuver_index = state.segment_index

        if state.remaining_to_end_m <= MANEUVER_COMPLETION_M:
            self._finish_maneuver("yorunge tamamlandi")

    def _finish_maneuver(self, reason: str) -> None:
        """Izlemeyi birakir. Mod degismez: AUTO'dan hic cikilmadi."""
        with self._lock:
            self._maneuver_path = ()
        logger.info("S-MANEVRASI BITTI | %s", reason)

    def _timing_error_s(self) -> float:
        """Pozitif deger gec kalindigini gosterir."""
        if self._planned_arrival_ns <= 0:
            return 0.0
        remaining_s = (self._planned_arrival_ns - time.monotonic_ns()) / 1e9
        return self._eta_s - remaining_s

    def _coordinate_and_regulate(self) -> None:
        """Hizi duzenler; ulasilabilirlik ve capa her tick'te step()'te isler."""
        self._regulate_speed()

    def _revise_plan(self, now_ns: int) -> None:
        """Ruzgar ogrenilince plani ileri ceker; asla one almaz.

        Taahhut anlarinda ruzgar bilinmiyor: oncu henuz havalanmadigi icin
        kimse olcmemis oluyor. Ruzgarsiz nominal sure fazla iyimser oldugu
        icin butun takvim sikisiyor.

        Duzeltmeyi yalnizca oncu kendi olcumunden yapar; takipciler oncunun
        taahhudundeki kaymayi izler. Takipciye de kendi olcumune dayali
        duzeltme verilmesi denendi ve geri alindi: havadaki arac kestirimi
        oturur oturmaz tek seferlik duzeltmeyi yapiyor, tirmanis sirasinda
        olculen ruzgar ise henuz sapmali oluyor ve bu deger kalicilasiyor.
        Olcumu tekrar tekrar duzeltebilen yerdeki arac yakinsiyor, havadaki
        arac rastgele bir ana kilitleniyordu (bkz. run_20260802_015849:
        HA-2 6.3 m/s @ 24 derece olcup nominali 538 -> 728 s yapti,
        HA-2 - HA-1 arasi 105 s'ye cikti).

        Kaydirma tek yonlu: geriye alinsaydi doygun bir araca imkansiz bir
        hedef verilebilirdi.
        """
        if self._nominal_plan_ns <= 0:
            return

        if self._is_leader():
            if self._takeoff_actual_ns <= 0 or self._plan_revised:
                return
            if self._wind is None:
                return
            self._refresh_nominal_flight_time(now_ns)
            revised_ns = self._takeoff_actual_ns + int(
                self._nominal_flight_s * NANOSECONDS_PER_SECOND
            )
            self._plan_revised = True
            reason = "ruzgar duzeltmesi (tek seferlik)"
        else:
            reference = compute_reference_arrival(
                self._config.vehicle_id, self._peer_commitments(now_ns)
            )
            if not reference.resolved:
                return
            revised_ns = reference.monotonic_ns
            reason = f"peer {reference.source_vehicle_ids} taahhudu guncellendi"

        self._revise_plan_later(revised_ns, reason)

    def _revise_plan_later(self, revised_ns: int, reason: str) -> None:
        with self._lock:
            if revised_ns <= self._nominal_plan_ns:
                return
            shift_s = (revised_ns - self._nominal_plan_ns) / 1e9
            self._nominal_plan_ns = revised_ns
            self._planned_arrival_ns = max(self._planned_arrival_ns, revised_ns)

        if shift_s >= ANCHOR_LOG_THRESHOLD_S:
            logger.info("varis plani %.1f s ileri cekildi (%s)", shift_s, reason)

    def _update_feasible_arrival(self, now_ns: int) -> None:
        """Azami hava hiziyla ulasilabilecek en erken varis anini gunceller.

        Tahmin, anlik rota ilerlemesine bolmek yerine ruzgar duzeltmeli rota
        modelinden uretilir. Anlik ilerleme tirmanista ve donuslerde sifira
        yaklastigi icin bolum patliyordu; olculen capa on saniyede yuzlerce
        saniye salinabiliyordu. Model, olcumu rota ilerlemesi yerine ruzgar
        kestirimi uzerinden alir: ayni bilgi, cok daha az gurultu.

        Modelin ikinci faydasi, aracin yerde de ulasilabilirlik bildirebilmesi.
        Yerdeki arac icin en ucuz duzeltme kalkisi ertelemektir (madde 8:
        havada bekleme en aza indirilmeli) ama bunu yapabilmesi icin takvimin
        sikistigini kalkmadan once ogrenmesi gerekir.
        """
        # Manevra sirasinda arac bilerek yoldan sapiyor; rota dogrultusundaki
        # ilerlemesi duser ama YAPABILECEGI degismez. Olculen degeri kullanmak
        # ulasilabilirligi cokertip capayi geriye itiyor, bu da daha fazla
        # manevra gerektiriyordu: pozitif geri besleme. Manevra boyunca son
        # temiz tahmin dondurulur.
        if self._maneuver_path or self._loitering:
            return

        # Varistan sonra arac RTL'e gecip hedeften uzaklasir; "kalan rota"
        # tanimsizlasir ve model dali kullanilsaydi gorev bastan
        # baslayacakmis gibi tum rota suresi eklenirdi.
        #
        # Yerine gercek varis ani yayinlanir. Bu bir tahmin degil olgudur ve
        # capayi gerceklige baglar. Sifir yayinlamak denenip geri alindi:
        # varan araclar capa kumesinden dustukce capa cokuyor ve en son
        # varacak arac, digerlerinin ucmus oldugu kaymayi kaybedip ham
        # nominal planina donuyordu (olculen: [1,2,3] uzerinde +15.0 s olan
        # capa, [3] tek basina kalinca 0.0 s).
        if self.state not in PRE_ARRIVAL_STATES:
            with self._lock:
                self._feasible_arrival_ns = self._detector.arrival_monotonic_ns or 0
            return

        wind = self._known_wind(now_ns) or WindEstimate(east_mps=0.0, north_mps=0.0)
        max_airspeed = self._config.max_airspeed_mps
        snapshot = self._telemetry.snapshot()

        if self.state in AIRBORNE_STATES and snapshot.valid:
            start_ns = now_ns
            seconds = route_duration_with_wind_s(
                snapshot.position,
                self._config.route[self._active_wp_index:],
                max_airspeed,
                wind,
            )
        else:
            # Henuz kalkmadi: en erken varis, planlanan kalkis anina bagli.
            start_ns = max(now_ns, self._takeoff_time_ns)
            seconds = route_duration_with_wind_s(
                self._config.home, self._config.route, max_airspeed, wind
            )

        with self._lock:
            self._feasible_arrival_ns = start_ns + int(seconds * NANOSECONDS_PER_SECOND)

    def _wind_blocks(
        self, position: LatLon, remaining: Tuple[LatLon, ...]
    ) -> Tuple[Tuple[LatLon, Tuple[LatLon, ...]], ...]:
        """Kalan rotayi ruzgar tutarlilik suresine gore bloklara ayirir.

        Bolme ruzgarsiz seyir suresine gore yapilir; blok siniri bacak
        sonlarina denk getirilir. Boylece her blok icin bagimsiz bir en kotu
        ruzgar secilebilir ve zamanla degisen ruzgar modellenmis olur.
        """
        bloklar = []
        blok_baslangic = position
        blok_noktalar: list = []
        blok_sure_s = 0.0
        onceki = position
        for nokta in remaining:
            leg_m = geodesic_distance_m(onceki, nokta)
            leg_s = leg_m / max(self._config.nominal_cruise_speed_mps, 1e-6)
            blok_noktalar.append(nokta)
            blok_sure_s += leg_s
            if blok_sure_s >= DISTURBANCE_COHERENCE_S:
                bloklar.append((blok_baslangic, tuple(blok_noktalar)))
                blok_baslangic = nokta
                blok_noktalar = []
                blok_sure_s = 0.0
            onceki = nokta
        if blok_noktalar:
            bloklar.append((blok_baslangic, tuple(blok_noktalar)))
        return tuple(bloklar)

    def _bounds_for_route_s(
        self,
        position: LatLon,
        remaining: Tuple[LatLon, ...],
        initial_airspeed_mps: float,
    ) -> Tuple[float, float]:
        """Ayni operasyonel ruzgar zarfiyla bir rota suffix'inin E/L'si.

        Doner: (E, L) saniye cinsinden, simdiye gore.

        E azami hava hizinda en yavas, L asgari hava hizinda en hizli sabit
        ruzgar adayindan gelir. Mevcut hizdan bu sinirlara gecis rate-limit
        ile modellenir. Iki taraf ayni modelden uretilir. Dokuman
        ruzgarin degisim hizini sinirlamadigi icin bu pencere formal adversarial
        garanti degil, secilen 0-10 m/s dogrulama zarfinin operasyonel siniridir.

        L'ye yasal loiter dahil edilmez: hedefe 2 km'den uzakta bekleme
        suresinin ust siniri yok, dolayisiyla L sonsuza gider ve hicbir
        bilgi tasimaz. Bekleme imkani ayri bir bayrakla temsil edilir.
        Dogrulanmamis S kapasitesi de dahil edilmez.
        """
        if not remaining:
            return 0.0, 0.0

        # Ruzgar zamanla degistigi icin her blok kendi en kotu ruzgarini alir;
        # tek sabit ruzgar varsayimi yavaslama kapasitesini abartiyordu.
        bloklar = self._wind_blocks(position, remaining)
        if len(bloklar) > 1:
            toplam_yavas_s = 0.0
            toplam_hizli_s = 0.0
            # Iki sinir iki ayri ucusu temsil eder: E azami hiza, L asgari
            # hiza kosar. Ikisine de ayni hizi devretmek, L zincirine her
            # blokta yavaslama rampasini yeniden odetip L'yi kucultuyordu.
            yavas_hiz = initial_airspeed_mps
            hizli_hiz = initial_airspeed_mps
            for blok_index, (blok_baslangic, blok_noktalar) in enumerate(bloklar):
                yavas_s, hizli_s = self._block_bounds_s(
                    blok_baslangic, blok_noktalar, yavas_hiz, hizli_hiz, blok_index
                )
                toplam_yavas_s += yavas_s
                toplam_hizli_s += hizli_s
                # Ilk bloktan sonra her zincir kendi hedef hizina ulasmistir;
                # rampa yalnizca ilk blokta anlamli.
                yavas_hiz = self._config.max_airspeed_mps
                hizli_hiz = self._config.min_airspeed_mps
            return toplam_yavas_s, toplam_hizli_s

        return self._block_bounds_s(
            position, remaining, initial_airspeed_mps, initial_airspeed_mps, 0
        )

    def _wind_candidates(self, block_index: int) -> Tuple[WindEstimate, ...]:
        """Bir blok icin taranacak sabit ruzgar adaylari.

        Zarf mutlaktir: dokuman ruzgarin siddet ve yon degisimini sinirlamadigi
        icin, olculen degerin etrafinda dar bir bant varsaymak dogrulanmadi
        (denendi, bkz. DISTURBANCE_WIND_SPEED_STEP_MPS yanindaki not).
        block_index imzada kalir: bloklarin ufka gore farkli zarf kullanmasi
        gerekirse tek nokta burasidir.
        """
        adaylar = []
        ruzgar_hizi = 0.0
        while ruzgar_hizi <= DISTURBANCE_WIND_MAX_MPS + 1e-9:
            # Sifir ruzgarda yonler ozdes; gereksiz on iki hesap yapma.
            yonler = (0.0,) if ruzgar_hizi == 0.0 else tuple(
                float(degree)
                for degree in range(0, 360, int(DISTURBANCE_DIRECTION_STEP_DEG))
            )
            adaylar.extend(
                wind_from_speed_direction(ruzgar_hizi, derece) for derece in yonler
            )
            ruzgar_hizi += DISTURBANCE_WIND_SPEED_STEP_MPS
        return tuple(adaylar)

    def _block_bounds_s(
        self,
        position: LatLon,
        remaining: Tuple[LatLon, ...],
        slow_initial_airspeed_mps: float,
        fast_initial_airspeed_mps: float,
        block_index: int,
    ) -> Tuple[float, float]:
        """Tek bir blok icin sabit ruzgar adaylarindan E/L."""
        if not remaining:
            return 0.0, 0.0

        en_yavas_s = 0.0
        en_hizli_s = math.inf
        for ruzgar in self._wind_candidates(block_index):
            en_yavas_s = max(
                en_yavas_s,
                route_duration_with_airspeed_ramp_s(
                    position,
                    remaining,
                    slow_initial_airspeed_mps,
                    self._config.max_airspeed_mps,
                    self._config.airspeed_rate_limit_mps2,
                    ruzgar,
                ),
            )
            en_hizli_s = min(
                en_hizli_s,
                route_duration_with_airspeed_ramp_s(
                    position,
                    remaining,
                    fast_initial_airspeed_mps,
                    self._config.min_airspeed_mps,
                    self._config.airspeed_rate_limit_mps2,
                    ruzgar,
                ),
            )
        return en_yavas_s, en_hizli_s

    def _robust_bounds_s(self, position: LatLon) -> Tuple[float, float]:
        """Mevcut konumdan kalan nominal rota icin operasyonel E/L."""
        return self._bounds_for_route_s(
            position,
            self._config.route[self._active_wp_index:],
            self._live_initial_airspeed(self._telemetry.snapshot()),
        )

    def _live_initial_airspeed(self, snapshot) -> float:
        """Rate-limit modelini komuttan degil, gecerli olculen hizdan baslat."""
        measured_mps = getattr(snapshot, "airspeed_mps", 0.0)
        if (
            math.isfinite(measured_mps)
            and 1.0 <= measured_mps <= 1.5 * self._config.max_airspeed_mps
        ):
            return measured_mps
        return self._controller.commanded_airspeed_mps

    def _refresh_robust_bounds(self) -> None:
        """Ortak pencere icin ayni modelden E/L uretir; yerde de calisir."""
        now_ns = time.monotonic_ns()
        if (now_ns - self._robust_bounds_ns) / NANOSECONDS_PER_SECOND < ROBUST_BOUNDS_INTERVAL_S:
            return
        # S sirasinda nominal suffix gercek yolu temsil etmez. Loiter'da ise
        # mutlak E/L zamanlari her saniye ileri gitmelidir; dondurulursa
        # peer'lara gecmiste kalmis bir A_min yayinlanir.
        if self._maneuver_path:
            return

        state = self.state
        snapshot = self._telemetry.snapshot()
        if state in AIRBORNE_STATES:
            if not snapshot.valid:
                return
            position = snapshot.position
            remaining = self._config.route[self._active_wp_index:]
        else:
            position = self._config.home
            remaining = self._config.route

        self._robust_bounds_ns = now_ns
        initial_airspeed_mps = (
            self._live_initial_airspeed(snapshot)
            if state in AIRBORNE_STATES
            else self._config.nominal_cruise_speed_mps
        )
        earliest_s, latest_s = self._bounds_for_route_s(
            position, remaining, initial_airspeed_mps
        )
        with self._lock:
            self._robust_earliest_s = earliest_s
            self._robust_latest_s = latest_s

    def _apply_feasible_anchor(self, now_ns: int) -> None:
        """Ortak capayi hesaplar ve plan ulasilamaz kaldiysa ileri kaydirir.

        Taahhut edilmis nominal plan geriye alinmaz; gecici calisma hedefi
        ulasilabilir robust [A_min, A_max] icinde iki yone duzeltilebilir.
        Butun araclar ayni yayin verisinden ayni capayi hesapladigi icin
        20 saniyelik aralik korunur.
        """
        feasible = dict(self._peer_feasible_arrivals(now_ns))
        feasible[self._config.vehicle_id] = self._feasible_arrival_ns

        anchor_ns = compute_feasible_anchor(feasible)
        if anchor_ns is None:
            return

        # KALDIRILDI: capayi peer'lardan toplanan robust E/L ortak araligina
        # kirpma katmani. Olculdu (5 kosu): ortak aralik orneklerin %92'sinde
        # bos kaliyordu, capa tam sifirdaydi; aciksa da 41-57 s'lik gecici
        # sicramalar uretip kontrolcuye bunlari kovalatiyordu. Yerel sinir
        # makinesi (kapi ve terminal rezerv) korundu, dagitik degis tokus
        # kaldirildi.
        target_ns = target_arrival(anchor_ns, self._config.vehicle_id)
        with self._lock:
            if self._nominal_plan_ns <= 0:
                return
            # Kaymayi daima sabit nominal plana gore olcuyoruz. Bir onceki
            # kaymis degere gore olculurse ulasilabilirligin dogal suruklenmesi
            # birikip plani sonsuza kadar oteler. Robust ortak pencere mevcutsa
            # calisma hedefi geriye gelebilir ama nominal taahhudun onune gecmez.
            new_plan_ns = max(self._nominal_plan_ns, target_ns)
            previous_ns = self._planned_arrival_ns
            if new_plan_ns == previous_ns:
                return
            self._planned_arrival_ns = new_plan_ns
            shift_s = (new_plan_ns - self._nominal_plan_ns) / 1e9
            change_s = abs(new_plan_ns - previous_ns) / 1e9

        if change_s >= ANCHOR_LOG_THRESHOLD_S:
            logger.info(
                "zamanlama capasi nominal plandan %.1f s ileride | "
                "ulasilabilirlik veren araclar: %s",
                shift_s, sorted(feasible),
            )

    def _terminal_reserve_override(
        self, now_ns: int
    ) -> Tuple[Optional[float], Optional[Tuple[float, float]]]:
        """Kapidan sonra erken/gec hiz rezervi azalirsa min/max hiz ister.

        Donen ikinci cift (erken_rezerv, gec_rezerv) yalnizca log icindir.
        Normal ETA kontrolu pencerenin icinde serbesttir; bariyer ancak bir
        kenara yaklasildiginda devreye girer.
        """
        if (
            self._hold_gate is None
            or not self._gate_crossed
            or self.state not in AIRBORNE_STATES
            or self._loitering
            or self._maneuver_path
            or self._planned_arrival_ns <= 0
        ):
            self._reserve_mode = None
            return None, None

        snapshot = self._telemetry.snapshot()
        if not snapshot.valid:
            return None, None
        remaining_route = self._config.route[self._active_wp_index:]
        if not remaining_route:
            return None, None
        wind = self._known_wind(now_ns) or WindEstimate(0.0, 0.0)
        current_airspeed_mps = self._live_initial_airspeed(snapshot)
        fast_s = route_duration_with_airspeed_ramp_s(
            snapshot.position,
            remaining_route,
            current_airspeed_mps,
            self._config.max_airspeed_mps,
            self._config.airspeed_rate_limit_mps2,
            wind,
        )
        slow_s = route_duration_with_airspeed_ramp_s(
            snapshot.position,
            remaining_route,
            current_airspeed_mps,
            self._config.min_airspeed_mps,
            self._config.airspeed_rate_limit_mps2,
            wind,
        )
        target_in_s = (self._planned_arrival_ns - now_ns) / NANOSECONDS_PER_SECOND
        late_reserve_s = target_in_s - fast_s
        early_reserve_s = slow_s - target_in_s

        early_low = early_reserve_s < TERMINAL_EARLY_RESERVE_S
        late_low = late_reserve_s < TERMINAL_LATE_RESERVE_S
        if early_low and late_low:
            # Secilen ruzgar modeli altinda pencere gecici olarak bos. Gercek
            # zamanlama hatasinin yonune gore kurtarilabilir tarafa basilir.
            self._reserve_mode = "fast" if self._timing_error_s() > 0.0 else "slow"
        elif early_low:
            self._reserve_mode = "slow"
        elif late_low:
            self._reserve_mode = "fast"
        elif self._reserve_mode == "slow" and early_reserve_s < (
            TERMINAL_EARLY_RESERVE_S + TERMINAL_RESERVE_HYSTERESIS_S
        ):
            pass
        elif self._reserve_mode == "fast" and late_reserve_s < (
            TERMINAL_LATE_RESERVE_S + TERMINAL_RESERVE_HYSTERESIS_S
        ):
            pass
        else:
            self._reserve_mode = None

        if self._reserve_mode == "slow":
            return self._config.min_airspeed_mps, (early_reserve_s, late_reserve_s)
        if self._reserve_mode == "fast":
            return self._config.max_airspeed_mps, (early_reserve_s, late_reserve_s)
        return None, (early_reserve_s, late_reserve_s)

    def _regulate_speed(self) -> None:
        """Zamanlama hatasina gore seyir hizini duzenler ve karari loglar."""
        now_ns = time.monotonic_ns()
        dt_s = (now_ns - self._last_control_ns) / 1e9 if self._last_control_ns else TICK_INTERVAL_S
        self._last_control_ns = now_ns

        forced_airspeed_mps, reserves = self._terminal_reserve_override(now_ns)

        command = self._controller.update(
            self._eta_s, self._remaining_distance_m,
            self._planned_arrival_ns, now_ns, dt_s,
            forced_airspeed_mps=forced_airspeed_mps,
        )
        if command is None or not command.changed:
            return

        self._commander.set_airspeed(command.airspeed_mps)
        reserve_text = (
            f" | rezerv erken/gec {reserves[0]:+.1f}/{reserves[1]:+.1f} s"
            if reserves is not None else ""
        )
        logger.info(
            "kontrol: %s | zamanlama hatasi %+.1f s | gerekli %.1f m/s | "
            "komut %.1f m/s | kalan %.0f m | WP%d | saturation=%s rate_limit=%s%s",
            command.action.value, command.timing_error_s, command.required_speed_mps,
            command.airspeed_mps, self._remaining_distance_m, self._active_wp_index,
            command.saturated, command.rate_limited, reserve_text,
        )

    def _on_arrived(self) -> None:
        if self._commander.set_mode("RTL"):
            self._transition(MissionState.RTL)

    def _on_rtl(self) -> None:
        if self._commander.flight_mode == "RTL":
            self._transition(MissionState.DONE)

    # --- yardimcilar ---

    def _update_wind(self, snapshot) -> None:
        """Havadayken ruzgari kestirir; yerdeki peer'lar bunu kullanir."""
        # Ruzgar yer yuzeyine dogru azalir (SITL'de karekok yasasi, tam
        # siddete SIM_WIND_T_ALT=60 m'de ulasilir). Alcakta olculen deger
        # dogrudur ama seyir irtifasindaki ruzgari temsil etmez; kalkis
        # irtifasinin altindaki ornekler kullanilmaz.
        if not snapshot.valid or snapshot.airspeed_mps < MIN_WIND_ESTIMATE_AIRSPEED_MPS:
            return
        if snapshot.altitude_msl_m < self._config.takeoff_alt_msl_m:
            return
        if snapshot.wind_sample_spread_s > MAX_WIND_SAMPLE_SPREAD_S:
            return
        wind = estimate_wind(
            (snapshot.velocity_east_mps, snapshot.velocity_north_mps),
            (
                snapshot.airspeed_forward_mps,
                snapshot.airspeed_left_mps,
                snapshot.airspeed_up_mps,
            ),
            snapshot.orientation_xyzw,
        )
        if wind is None:
            return

        # Sure, isleme ani degil ornegin kendi damgasi uzerinden olculur:
        # telemetri duraklarsa ayni ornek tekrar tekrar okunur ve isleme
        # anina gore hesaplanan dt filtreyi olmayan veriyle ilerletirdi.
        sample_ns = snapshot.updated_monotonic_ns
        # Ilk ornekte gecen sure bilinmedigi icin tick araligi varsayilir.
        dt_s = (
            (sample_ns - self._last_wind_sample_ns) / NANOSECONDS_PER_SECOND
            if self._last_wind_sample_ns > 0
            else TICK_INTERVAL_S
        )
        self._last_wind_sample_ns = sample_ns
        self._wind_filter.update(wind, dt_s)
        if not self._wind_filter.settled:
            return
        with self._lock:
            self._wind = self._wind_filter.estimate

    def _route_deviation_from_nominal(self, position: LatLon) -> float:
        """Konumun tam nominal rota polylinesine en kisa uzakligi."""
        points = (self._config.home, *self._config.route)
        return min(
            cross_track_distance_m(position, start, end)
            for start, end in zip(points, points[1:])
        )

    def _track_route_deviation(self, position: LatLon, _active_index: int) -> None:
        """Konumun verilen nominal rota polylinesine en kisa uzakligini izler."""
        deviation_m = self._route_deviation_from_nominal(position)
        with self._lock:
            if deviation_m <= self._max_route_deviation_m:
                return
            self._max_route_deviation_m = deviation_m

        if deviation_m > MAX_ROUTE_DEVIATION_M and not self._deviation_warned:
            self._deviation_warned = True
            logger.warning(
                "ROTA SAPMASI SINIRI ASILDI: %.0f m (sinir %.0f m)",
                deviation_m, MAX_ROUTE_DEVIATION_M,
            )

    def _altitude_reached(self, target_alt_msl_m: float) -> bool:
        snapshot = self._telemetry.snapshot()
        if not snapshot.valid:
            return False
        return snapshot.altitude_msl_m >= target_alt_msl_m - ALTITUDE_REACHED_MARGIN_M

    def _track_arrival(self) -> Optional[LatLon]:
        """Varis tespitini besler; varis olustuysa durumu ilerletir."""
        snapshot = self._telemetry.snapshot()
        if not snapshot.valid:
            return None

        if snapshot.age_s(time.monotonic_ns()) > TELEMETRY_TIMEOUT_S:
            logger.warning("telemetri eskimis, varis tespiti guvenilmez")
            return None

        # Varistan sonra arac RTL'e girip uzaklastigi icin ETA anlamini yitirir;
        # son gecerli tahmin dondurulur.
        if not self._detector.arrived:
            eta = self._estimator.update(
                snapshot.position,
                (snapshot.velocity_east_mps, snapshot.velocity_north_mps),
                snapshot.updated_monotonic_ns,
            )
            with self._lock:
                self._remaining_distance_m = eta.remaining_distance_m
                self._active_wp_index = eta.active_index
                self._progress_speed_mps = eta.progress_speed_mps
                self._eta_s = self._model_eta_s(snapshot.position)
            self._track_route_deviation(snapshot.position, eta.active_index)

        with self._lock:
            arrived = self._detector.update(snapshot.position, snapshot.updated_monotonic_ns)
            min_distance_m = self._detector.min_distance_m
            interpolated = self._detector.interpolated

        if arrived:
            logger.info(
                "HEDEFE VARILDI | en yakin gecis %.2f m | tespit %s | "
                "max rota sapmasi %.0f m (sinir %.0f m)",
                min_distance_m, "interpolasyon" if interpolated else "dogrudan ornek",
                self._max_route_deviation_m, MAX_ROUTE_DEVIATION_M,
            )
            self._transition(MissionState.ARRIVED)
        return snapshot.position


_HANDLERS = {
    MissionState.INIT: MissionManager._on_init,
    MissionState.CONNECTING: MissionManager._on_connecting,
    MissionState.MISSION_UPLOAD: MissionManager._on_mission_upload,
    MissionState.WAIT_PEERS: MissionManager._on_wait_peers,
    MissionState.WAIT_TAKEOFF_SLOT: MissionManager._on_wait_takeoff_slot,
    MissionState.ARMING: MissionManager._on_arming,
    MissionState.TAKEOFF: MissionManager._on_takeoff,
    MissionState.CLIMB: MissionManager._on_climb,
    MissionState.CRUISE: MissionManager._on_cruise,
    MissionState.TERMINAL: MissionManager._on_terminal,
    MissionState.ARRIVED: MissionManager._on_arrived,
    MissionState.RTL: MissionManager._on_rtl,
}
