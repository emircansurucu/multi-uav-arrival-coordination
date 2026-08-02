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

from .autopilot_adapter.mission_builder import build_mission
from .config_model import VehicleConfig
from .coordination.arrival_schedule import (
    NANOSECONDS_PER_SECOND,
    compute_feasible_anchor,
    compute_reference_arrival,
    compute_takeoff_time,
    target_arrival,
)
from .control.arrival_controller import ArrivalController
from .control.maneuver_planner import follow_path, plan_s_maneuver, turn_radius_m
from .estimation.arrival_detector import ArrivalDetector
from .estimation.eta_estimator import EtaEstimator, route_length_m
from .estimation.wind_estimator import (
    WindEstimate,
    WindFilter,
    estimate_wind,
    route_duration_with_wind_s,
    wind_from_speed_direction,
)
from .estimation.geodesy import LatLon, cross_track_distance_m, geodesic_distance_m

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
# Bu kadar erken kalindiginda loiter'a girilir. Hiz yetkisi once tuketilir;
# esik, asgari hizda bile kapatilamayan farki yakalayacak kadar buyuk.
LOITER_TRIGGER_S = 8.0
# Anlik ETA sicramalari loiter baslatmasin diye ardisik tick dogrulamasi.
LOITER_CONFIRM_TICKS = 40
# Loiter'dan cikis esigi. Girisden kucuk tutulur ki cikar cikmaz tekrar
# girilmesin (histerezis).
LOITER_EXIT_S = 1.0
# Capa bu esikten az kaydiginda log uretilmez.
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
# Takip noktasi mesafesi. SITL plane modelinde WP_LOITER_RAD 80 m; takip
# noktasi bunun belirgin uzerinde tutulmazsa arac hedefi yakalayip cember
# atmaya basliyor ve manevra hic ilerlemiyor.
MANEUVER_LOOKAHEAD_M = 250.0
# Yorungenin sonuna bu kadar kalinca AUTO gorevine geri donulur.
MANEUVER_HANDOVER_M = 300.0
# Ruzgar kestiriminin gecerli sayilmasi icin gereken en dusuk hava hizi;
# yerde ve kalkis kosusunda olculen degerler anlamsizdir.
MIN_WIND_ESTIMATE_AIRSPEED_MPS = 10.0
# Konum, yer hizi ve hava hizi 33 ms'de bir yayinlanir; bu esigi asan
# yayilim, konulardan birinin durdugu (AP_DDS yayini tikandigi) anlamina
# gelir ve o ornekten cikarilan ruzgar gercek degildir.
MAX_WIND_SAMPLE_SPREAD_S = 0.2


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
        self._planned_arrival_ns = 0
        self._arrival_committed = False
        self._takeoff_time_ns = 0
        self._next_peer_log = 0.0
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
        self._loitering = False
        self._loiter_streak = 0
        self._loiter_entry_eta_s = 0.0
        self._wind_filter = WindFilter()
        self._last_wind_sample_ns = 0
        # Yalnizca filtre oturduktan sonra doldurulur; oturmamis kestirim
        # ne plan hesabinda kullanilir ne de peer'lara yayinlanir.
        self._wind: Optional[WindEstimate] = None
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
            return MissionSnapshot(
                state=self._state,
                target_reached=self._detector.arrived,
                arrival_monotonic_ns=self._detector.arrival_monotonic_ns or 0,
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
                wind_valid=self._wind is not None,
                wind_speed_mps=self._wind.speed_mps if self._wind else 0.0,
                wind_from_direction_deg=(
                    self._wind.from_direction_deg if self._wind else 0.0
                ),
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
        )
        if self._commander.upload_mission(mission):
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
        self._coordinate_and_regulate()
        if self._handle_loiter(position):
            return
        if geodesic_distance_m(position, self._config.target) <= TERMINAL_RADIUS_M:
            self._transition(MissionState.TERMINAL)

    def _handle_loiter(self, position: LatLon) -> bool:
        """Fazla erken kalindiginda daire cizerek zaman kaybettirir.

        Hiz yetkisi tukendiginde (asgari hava hizinda hala erken) havada
        zaman kaybetmenin dokumandaki tek serbest yolu budur; S-manevrasi
        henuz kullanilabilir degil. Daire kapali oldugu icin arac rota
        boyunca ilerlemez ve gecen surenin tamami kazanilir.

        Loiter yalnizca hedefe 2 km'den uzakta serbesttir (madde 6), bu
        yuzden yalnizca CRUISE'da ve mesafe payiyla birlikte denenir.
        Cikis kapali cevrimdir: zamanlama hatasi kapaninca AUTO'ya donulur,
        boylece tur sayisi hesaplamak ve ruzgarin tur suresine etkisini
        modellemek gerekmez.

        True donerse bu tick'te gorev akisi ilerletilmez.
        """
        if not self._config.loiter_enabled or self._guided is None:
            return False

        if self._loitering:
            if self._loiter_remaining_early_s() <= LOITER_EXIT_S:
                self._exit_loiter()
                return False
            return True

        distance_m = geodesic_distance_m(position, self._config.target)
        early_s = self._timing_error_s()
        at_min_airspeed = math.isclose(
            self._controller.commanded_airspeed_mps,
            self._config.min_airspeed_mps,
            abs_tol=0.2,
        )
        if (
            early_s > -LOITER_TRIGGER_S
            or not at_min_airspeed
            or distance_m < MIN_LOITER_DISTANCE_M
        ):
            self._loiter_streak = 0
            return False

        self._loiter_streak += 1
        if self._loiter_streak < LOITER_CONFIRM_TICKS:
            return False

        self._enter_loiter(position, early_s, distance_m)
        return True

    def _enter_loiter(self, position: LatLon, early_s: float, distance_m: float) -> None:
        # Daire merkezi mevcut konumdur: arac rotanin uzerindeyken girdigi
        # icin cemberin rotaya uzakligi yaricap kadar kalir ve 500 m sinirinin
        # cok altindadir.
        self._commander.set_mode("GUIDED")
        self._guided.send(position, self._config.cruise_alt_msl_m)
        with self._lock:
            self._loitering = True
            # Daire kapali oldugu icin arac ayni noktaya doner ve hedefe
            # uzakligi degismez: giristeki ETA cikisa kadar gecerli kalir.
            self._loiter_entry_eta_s = self._eta_s
        logger.info(
            "LOITER BASLADI | %.1f s erken | hedefe %.0f m (sinir %.0f m)",
            -early_s, distance_m, MIN_LOITER_DISTANCE_M,
        )

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
        return route_duration_with_wind_s(
            position,
            self._config.route[self._active_wp_index:],
            self._controller.commanded_airspeed_mps,
            wind,
        )

    def _loiter_remaining_early_s(self) -> float:
        """Daire cizerken kapatilmasi kalan erkenlik.

        Canli ETA kullanilamaz: daire cizerken rota dogrultusundaki ilerleme
        cokuyor, ETA siisiyor ve zamanlama hatasi gercekte zaman kazanilmadan
        kapanmis gorunuyor (olculen: 16.5 s erken girilip 4 s sonra cikildi).
        Giristeki ETA sabit tutulur; erkenlik yalnizca gecen sureyle kapanir.
        """
        remaining_s = (self._planned_arrival_ns - time.monotonic_ns()) / 1e9
        return remaining_s - self._loiter_entry_eta_s

    def _exit_loiter(self) -> None:
        kalan_s = self._loiter_remaining_early_s()
        self._commander.set_mode("AUTO")
        with self._lock:
            self._loitering = False
            self._loiter_streak = 0
        logger.info("LOITER BITTI | kalan erkenlik %.1f s", kalan_s)

    def _on_terminal(self) -> None:
        position = self._track_arrival()
        if position is None:
            return
        self._coordinate_and_regulate()

        if self._maneuver_path:
            self._fly_maneuver(position)
        else:
            self._consider_s_maneuver(position)

    def _consider_s_maneuver(self, position: LatLon) -> None:
        """Hiz yetkisi tukendiyse yorunge uzatma manevrasi planlar.

        Dokuman hedefin 2 km cevresinde loiter'i yasakladigi icin terminal
        fazda tek secenek S-manevrasidir.
        """
        if not self._config.s_maneuver_enabled:
            return
        if self._maneuver_attempted or self._guided is None:
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

        self._maneuver_attempted = True
        extra_distance_m = -early_s * max(self._progress_speed_mps, 1.0)
        maneuver = plan_s_maneuver(
            position,
            self._config.target,
            extra_distance_m,
            turn_radius_m(self._config.min_airspeed_mps, MANEUVER_BANK_ANGLE_DEG),
            max_lateral_offset_m=MANEUVER_PLAN_LATERAL_LIMIT_M,
        )
        if maneuver is None:
            logger.warning(
                "S-manevrasi uretilemedi: %.0f m ek mesafe 500 m sapma ve donus "
                "yaricapi kisitlari altinda saglanamiyor", extra_distance_m,
            )
            return

        if not self._commander.set_mode("GUIDED"):
            logger.error("GUIDED moduna gecilemedi, manevra iptal")
            return

        # Yorunge, planlama anindaki konumdan baslar ve hedefte biter.
        # Hedefin kendisi hicbir zaman komut edilmez; son yaklasma AUTO'ya
        # devredilir, aksi halde arac hedefin etrafinda cember atar.
        with self._lock:
            self._maneuver_path = (position, *maneuver.waypoints)
            self._maneuver_index = 0

        logger.info(
            "S-MANEVRASI BASLADI | %.0f m ek mesafe | %d dongu | "
            "yanal sapma %.0f m (sinir 500 m) | %.1f s erken",
            extra_distance_m, maneuver.cycles, maneuver.lateral_offset_m, -early_s,
        )

    def _fly_maneuver(self, position: LatLon) -> None:
        """Aracin onunde kayan takip noktasini komut eder.

        Ulasilabilir bir nokta komut edilmez; ArduPlane GUIDED'da hedefe
        varan arac WP_LOITER_RAD yaricapiyla cember atmaya baslar.
        """
        state = follow_path(
            self._maneuver_path, position, MANEUVER_LOOKAHEAD_M, self._maneuver_index
        )
        if state is None:
            self._abort_maneuver("takip noktasi hesaplanamadi")
            return

        with self._lock:
            self._maneuver_index = state.segment_index

        if state.remaining_to_end_m <= MANEUVER_HANDOVER_M:
            self._finish_maneuver()
            return

        self._guided.send(state.carrot, self._config.cruise_alt_msl_m)

    def _abort_maneuver(self, reason: str) -> None:
        logger.error("S-manevrasi iptal edildi: %s", reason)
        with self._lock:
            self._maneuver_path = ()
        self._finish_maneuver()

    def _finish_maneuver(self) -> None:
        """Manevra bitince gorevi devralmasi icin AUTO'ya donulur."""
        with self._lock:
            self._maneuver_path = ()
        if self._commander.set_mode("AUTO"):
            logger.info("S-MANEVRASI BITTI | son yaklasma AUTO gorevine birakildi")
        else:
            logger.error("AUTO moduna donulemedi, arac GUIDED'da kaldi")

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

    def _apply_feasible_anchor(self, now_ns: int) -> None:
        """Ortak capayi hesaplar ve plan ulasilamaz kaldiysa ileri kaydirir.

        Kaydirma yalnizca ileri yondedir: doygun bir araca tekrar imkansiz
        bir hedef verilmemesi icin. Butun araclar ayni yayin verisinden ayni
        capayi hesapladigi icin 20 saniyelik aralik korunur.
        """
        feasible = dict(self._peer_feasible_arrivals(now_ns))
        feasible[self._config.vehicle_id] = self._feasible_arrival_ns

        anchor_ns = compute_feasible_anchor(feasible)
        if anchor_ns is None:
            return

        target_ns = target_arrival(anchor_ns, self._config.vehicle_id)
        with self._lock:
            if self._nominal_plan_ns <= 0:
                return
            # Kaymayi daima sabit nominal plana gore olcuyoruz. Bir onceki
            # kaymis degere gore olculurse, azami hizin altinda ucarken
            # ulasilabilirligin dogal ileri suruklenmesi birikip plani
            # sonsuza kadar oteliyor.
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

    def _regulate_speed(self) -> None:
        """Zamanlama hatasina gore seyir hizini duzenler ve karari loglar."""
        now_ns = time.monotonic_ns()
        dt_s = (now_ns - self._last_control_ns) / 1e9 if self._last_control_ns else TICK_INTERVAL_S
        self._last_control_ns = now_ns

        command = self._controller.update(
            self._eta_s, self._remaining_distance_m,
            self._planned_arrival_ns, now_ns, dt_s,
        )
        if command is None or not command.changed:
            return

        self._commander.set_airspeed(command.airspeed_mps)
        logger.info(
            "kontrol: %s | zamanlama hatasi %+.1f s | gerekli %.1f m/s | "
            "komut %.1f m/s | kalan %.0f m | WP%d | saturation=%s rate_limit=%s",
            command.action.value, command.timing_error_s, command.required_speed_mps,
            command.airspeed_mps, self._remaining_distance_m, self._active_wp_index,
            command.saturated, command.rate_limited,
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

    def _track_route_deviation(self, position: LatLon, active_index: int) -> None:
        """Aktif bacaga olan dik uzakligi izler ve 500 m sinirini denetler."""
        leg_start = self._config.home if active_index == 0 else self._config.route[active_index - 1]
        deviation_m = cross_track_distance_m(
            position, leg_start, self._config.route[active_index]
        )
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
