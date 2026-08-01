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
from typing import Callable, Dict, Optional

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
from .estimation.arrival_detector import ArrivalDetector
from .estimation.eta_estimator import EtaEstimator, route_length_m
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
# Ulasilabilirlik tahmini donuslerdeki anlik ilerleme dususlerine tepki
# vermemeli; ETA filtresinden cok daha yavas bir zaman sabiti kullanilir.
FEASIBILITY_FILTER_TAU_S = 30.0
# Filtrelenmis ilerleme bu degerin altina duserse tahmin anlamsizlasir.
MIN_FEASIBLE_PROGRESS_MPS = 5.0
# Capa bu esikten az kaydiginda log uretilmez.
ANCHOR_LOG_THRESHOLD_S = 1.0


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
class MissionSnapshot:
    """Durum makinesinin diger thread'lerden okunabilen anlik gorunumu."""

    state: MissionState
    target_reached: bool
    arrival_monotonic_ns: int
    arrival_min_distance_m: float
    arrival_interpolated: bool
    planned_arrival_monotonic_ns: int
    arrival_committed: bool
    earliest_feasible_arrival_monotonic_ns: int
    max_route_deviation_m: float
    eta_s: float
    remaining_distance_m: float
    active_wp_index: int
    commanded_airspeed_mps: float


class MissionManager:
    """Kalkistan RTL'e kadar gorev akisini yuruten durum makinesi."""

    def __init__(
        self,
        config: VehicleConfig,
        commander,
        telemetry,
        peer_commitments: Optional[Callable[[int], Dict[int, int]]] = None,
        peer_feasible_arrivals: Optional[Callable[[int], Dict[int, int]]] = None,
    ) -> None:
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
        self._filtered_progress_mps: Optional[float] = None
        self._feasibility_filter_ns = 0
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
                arrival_committed=self._arrival_committed,
                earliest_feasible_arrival_monotonic_ns=self._feasible_arrival_ns,
                max_route_deviation_m=self._max_route_deviation_m,
                eta_s=self._eta_s,
                remaining_distance_m=self._remaining_distance_m,
                active_wp_index=self._active_wp_index,
                commanded_airspeed_mps=self._controller.commanded_airspeed_mps,
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

    def _on_wait_peers(self) -> None:
        """Referans varis anini belirler ve kendi planini taahhut eder."""
        now_ns = time.monotonic_ns()
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
        if time.monotonic_ns() >= self._takeoff_time_ns:
            self._transition(MissionState.ARMING)

    def _on_arming(self) -> None:
        if not self._commander.set_mode("AUTO"):
            return
        if not self._commander.arm():
            return
        if not self._arrival_committed:
            self._commit(
                time.monotonic_ns() + int(self._nominal_flight_s * NANOSECONDS_PER_SECOND),
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
        if geodesic_distance_m(position, self._config.target) <= TERMINAL_RADIUS_M:
            self._transition(MissionState.TERMINAL)

    def _on_terminal(self) -> None:
        if self._track_arrival() is not None:
            self._coordinate_and_regulate()

    def _coordinate_and_regulate(self) -> None:
        """Ulasilabilirligi yayina hazirlar, capayi uygular, hizi duzenler."""
        now_ns = time.monotonic_ns()
        self._update_feasible_arrival(now_ns)
        self._apply_feasible_anchor(now_ns)
        self._regulate_speed()

    def _update_feasible_arrival(self, now_ns: int) -> None:
        """Azami hava hiziyla ulasilabilecek en erken varis anini gunceller.

        Ruzgar hava hizina eklenir, onu olceklemez: azami hizdaki ilerleme
        olculen ilerlemeye kalan hiz yetkisinin eklenmesiyle bulunur.
        Carpimsal model donuslerde ilerleme dustugunde tahmini cokertiyor.

        Ilerleme, donus ve gecici bozulmalara tepki vermemesi icin ayrica
        yavas bir filtreden gecirilir.
        """
        max_airspeed = self._config.max_airspeed_mps
        commanded = self._controller.commanded_airspeed_mps

        if self._progress_speed_mps <= 0.0 or self._remaining_distance_m <= 0.0:
            seconds = route_length_m(self._config.home, self._config.route) / max_airspeed
        else:
            filtered = self._filter_progress(self._progress_speed_mps, now_ns)
            max_progress_mps = max(
                filtered + (max_airspeed - commanded), MIN_FEASIBLE_PROGRESS_MPS
            )
            seconds = self._remaining_distance_m / max_progress_mps

        with self._lock:
            self._feasible_arrival_ns = now_ns + int(seconds * NANOSECONDS_PER_SECOND)

    def _filter_progress(self, raw_mps: float, now_ns: int) -> float:
        if self._filtered_progress_mps is None or self._feasibility_filter_ns == 0:
            self._filtered_progress_mps = raw_mps
        else:
            dt_s = (now_ns - self._feasibility_filter_ns) / 1e9
            if dt_s > 0.0:
                alpha = 1.0 - math.exp(-dt_s / FEASIBILITY_FILTER_TAU_S)
                self._filtered_progress_mps += alpha * (raw_mps - self._filtered_progress_mps)
        self._feasibility_filter_ns = now_ns
        return self._filtered_progress_mps

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
                self._eta_s = eta.eta_s
                self._remaining_distance_m = eta.remaining_distance_m
                self._active_wp_index = eta.active_index
                self._progress_speed_mps = eta.progress_speed_mps
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
