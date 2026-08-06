"""kalkıştan rtl aşamasına kadar görev akışını yönetir"""
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
    compute_gate_release_window,
    compute_reference_arrival,
    compute_takeoff_time,
    target_arrival,
)
from .control.arrival_controller import ArrivalController
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
)

logger = logging.getLogger(__name__)

TICK_INTERVAL_S = 0.05  # görev döngüsü aralığı
PEER_WAIT_LOG_INTERVAL_S = 10.0  # araç bekleme kayıt aralığı
LATE_TAKEOFF_TOLERANCE_S = 1.0  # geç kalkış toleransı
TERMINAL_RADIUS_M = 2000.0  # terminal bölgesi yarıçapı
MAX_ROUTE_DEVIATION_M = 500.0  # izin verilen en büyük rota sapması
ALTITUDE_REACHED_MARGIN_M = 15.0  # seyir irtifası kabul payı
TELEMETRY_TIMEOUT_S = 3.0  # telemetri zaman aşımı
MIN_LOITER_DISTANCE_M = 2500.0  # bekleme için en kısa hedef mesafesi
GATE_EARLY_MARGIN_S = 3.0  # kapı erken geçiş payı
GATE_LATE_MARGIN_S = 20.0  # kapı geç geçiş payı
GATE_HOLD_TRIGGER_S = 2.0  # bekleme başlatma eşiği
GATE_CROSSING_HYSTERESIS_M = 40.0  # kapı geçiş kararlılık payı
GATE_TARGET_DISTANCE_ABORT_M = 2200.0  # hedefe yaklaşınca beklemeyi bırakma sınırı
GATE_ROUTE_DEVIATION_ABORT_M = 400.0  # rota sapınca beklemeyi bırakma sınırı
TERMINAL_EARLY_RESERVE_S = 18.0  # terminal erken varış rezervi
TERMINAL_LATE_RESERVE_S = 10.0  # terminal geç varış rezervi
TERMINAL_RESERVE_HYSTERESIS_S = 2.0  # rezerv modu kararlılık payı
GATE_LOG_INTERVAL_S = 10.0  # kapı kayıt aralığı
ANCHOR_LOG_THRESHOLD_S = 1.0  # çıpa kayması kayıt eşiği
MIN_WIND_ESTIMATE_AIRSPEED_MPS = 10.0  # rüzgâr hesabı için en düşük hava hızı
MAX_WIND_SAMPLE_SPREAD_S = 0.2  # rüzgâr verileri arasındaki en büyük zaman farkı
DISTURBANCE_WIND_MAX_MPS = 10.0  # rüzgâr zarfındaki en yüksek hız
DISTURBANCE_WIND_SPEED_STEP_MPS = 2.0  # rüzgâr zarfı hız adımı
DISTURBANCE_COHERENCE_S = 180.0  # rüzgârın sabit kabul edildiği süre
DISTURBANCE_DIRECTION_STEP_DEG = 30.0  # rüzgâr zarfı yön adımı
ROBUST_BOUNDS_INTERVAL_S = 1.0  # varış sınırlarını yenileme aralığı


class MissionState(IntEnum):
    # araç durum mesajıyla ortak görev durumlarını tanımlar
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
    # nominal rota üzerindeki son yasal bekleme kapısını tutar
    position: LatLon
    active_wp_index: int
    downstream_route: Tuple[LatLon, ...]
    terminal_earliest_s: float
    terminal_latest_s: float


AIRBORNE_STATES = frozenset({  # rüzgâr ölçülen havadaki durumlar
    MissionState.TAKEOFF, MissionState.CLIMB,
    MissionState.CRUISE, MissionState.TERMINAL,
})

PRE_ARRIVAL_STATES = frozenset(
    state for state in MissionState if state < MissionState.ARRIVED
)  # çıpa hesabına katılan varış öncesi durumlar


@dataclass(frozen=True)
class MissionSnapshot:
    # görev yöneticisinin anlık durumunu tutar

    state: MissionState
    target_reached: bool
    arrival_monotonic_ns: int
    arrival_min_distance_m: float
    arrival_interpolated: bool
    planned_arrival_monotonic_ns: int
    committed_plan_monotonic_ns: int  # diğer araçlara yayınlanan taahhüt anı
    arrival_committed: bool
    earliest_feasible_arrival_monotonic_ns: int
    max_route_deviation_m: float
    eta_s: float
    remaining_distance_m: float
    active_wp_index: int
    commanded_airspeed_mps: float
    wind_valid: bool
    wind_speed_mps: float
    wind_from_direction_deg: float  # rüzgârın geldiği yön
    robust_earliest_s: float  # rüzgâr zarfındaki en erken varış süresi
    robust_latest_s: float  # rüzgâr zarfındaki en geç varış süresi


class MissionManager:
    # kalkıştan rtl aşamasına kadar görev akışını yürütür
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
        # görev bileşenlerini ve başlangıç durumunu hazırlar
        self._guided = guided_commander
        self._peer_wind = peer_wind or (lambda _now_ns: None)
        self._config = config
        self._commander = commander
        self._telemetry = telemetry
        # diğer araçların taahhütlerini okur
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
        # kapı hesabından önce rüzgâr alanını hazırlar
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
        # çıpa kaymasını sabit nominal plana göre ölçer
        self._nominal_plan_ns = 0
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
        # gerçek kalkış anını saklar
        self._takeoff_actual_ns = 0
        # öncü planının tek sefer yenilenmesini izler
        self._plan_revised = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def nominal_flight_s(self) -> float:
        # rüzgâra göre güncellenebilen nominal uçuş süresini döner
        return self._nominal_flight_s

    @property
    def state(self) -> MissionState:
        # görevin güncel durumunu kilit altında döner
        with self._lock:
            return self._state

    def snapshot(self) -> MissionSnapshot:
        # görev durumunun güvenli bir anlık kopyasını oluşturur
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
                # rüzgârı yalnızca havadaki durumlarda geçerli sayar
                wind_valid=self._wind is not None and self._state in AIRBORNE_STATES,
                wind_speed_mps=self._wind.speed_mps if self._wind else 0.0,
                wind_from_direction_deg=(
                    self._wind.from_direction_deg if self._wind else 0.0
                ),
                robust_earliest_s=0.0 if arrived else self._robust_earliest_s,
                robust_latest_s=0.0 if arrived else self._robust_latest_s,
            )

    def start(self) -> None:
        # görev döngüsünü ayrı bir thread içinde başlatır
        self._thread = threading.Thread(target=self._run, name="mission_manager")
        self._thread.start()

    def stop(self, join_timeout_s: float = 5.0) -> None:
        # görev döngüsünü durdurup thread kapanışını bekler
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=join_timeout_s)

    def _run(self) -> None:
        # görev adımlarını sabit aralıklarla çalıştırır
        while not self._stop.is_set():
            try:
                self.step()
                # mavlink mesaj kuyruğunu boşaltır
                if self._commander.connected:
                    self._commander.drain()
            except Exception:  # noqa: BLE001
                logger.exception("görev adımı başarısız, güvenli duruma geçiliyor")
                self._transition(MissionState.FAILSAFE)
            self._stop.wait(TICK_INTERVAL_S)

    def _transition(self, new_state: MissionState) -> None:
        # görevi yeni duruma geçirip değişikliği loglar
        with self._lock:
            if self._state == new_state:
                return
            previous = self._state
            self._state = new_state
        logger.info("durum: %s -> %s", previous.name, new_state.name)

    def step(self) -> None:
        # tek bir görev durumu adımını çalıştırır
        if self.state in AIRBORNE_STATES:
            self._update_wind(self._telemetry.snapshot())
        # varış sınırlarını yerdeyken de hesaplar
        if self.state in PRE_ARRIVAL_STATES:
            self._refresh_robust_bounds()
        if self._arrival_committed:
            # yerdeki araçların planı izlemesini sağlar
            now_ns = time.monotonic_ns()
            self._revise_plan(now_ns)
            # çıpa düzeltmesini kalkıştan önce uygular
            self._update_feasible_arrival(now_ns)
            if self.state in PRE_ARRIVAL_STATES:
                self._apply_feasible_anchor(now_ns)
        handler = _HANDLERS.get(self.state)
        if handler is not None:
            handler(self)

    # görev durumu işleyicileri

    def _on_init(self) -> None:
        # ilk görev adımında bağlantı durumuna geçer
        self._transition(MissionState.CONNECTING)

    def _on_connecting(self) -> None:
        # mavlink gps ve telemetri hazır olunca görev yüklemeye geçer
        if not self._commander.connected and not self._commander.connect():
            return
        if not self._commander.wait_gps_ready():
            return
        if not self._telemetry.snapshot().valid:
            return
        self._transition(MissionState.MISSION_UPLOAD)

    def _on_mission_upload(self) -> None:
        # auto görevini oluşturup otopilota yükler
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
        # yerel rüzgâr yoksa diğer araçların ölçümünü kullanır
        if self._wind is not None:
            return self._wind
        wind_pair = self._peer_wind(now_ns)
        return wind_from_speed_direction(*wind_pair) if wind_pair else None

    def _refresh_nominal_flight_time(self, now_ns: int) -> None:
        # nominal uçuş süresini bilinen rüzgâra göre düzeltir
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
            "nominal uçuş süresi rüzgâra göre düzeltildi: %.0f s -> %.0f s "
            "(diğer araç ölçümü %.1f m/s, %.0f dereceden)",
            self._nominal_flight_s, corrected_s, wind_pair[0], wind_pair[1],
        )
        self._nominal_flight_s = corrected_s

    def _on_wait_peers(self) -> None:
        # referans varış anını belirler ve planı taahhüt eder
        now_ns = time.monotonic_ns()
        self._refresh_nominal_flight_time(now_ns)
        reference = compute_reference_arrival(
            self._config.vehicle_id, self._peer_commitments(now_ns)
        )

        if reference.resolved:
            self._commit(reference.monotonic_ns, f"peer {reference.source_vehicle_ids}")
        elif self._is_leader():
            # öncünün gerçek kalkış anını başlangıç kabul eder
            with self._lock:
                self._takeoff_time_ns = now_ns
        else:
            self._log_peer_wait()
            return

        self._transition(MissionState.WAIT_TAKEOFF_SLOT)

    def _is_leader(self) -> bool:
        # aracın zamanlama öncüsü olup olmadığını döner
        return self._config.vehicle_id == 1

    def _build_hold_gate(self) -> Optional[HoldGate]:
        # nominal rotanın güvenli bekleme kapısını kurar
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
        # diğer araçların taahhüdünü bekleme durumunu aralıklı loglar
        if time.monotonic() < self._next_peer_log:
            return
        self._next_peer_log = time.monotonic() + PEER_WAIT_LOG_INTERVAL_S
        logger.info("önceki araçların taahhüdü bekleniyor")

    def _commit(self, planned_arrival_ns: int, source: str) -> None:
        # varış planını kilitler ve diğer araçlara açar
        takeoff_ns = compute_takeoff_time(planned_arrival_ns, self._nominal_flight_s)
        with self._lock:
            self._planned_arrival_ns = planned_arrival_ns
            self._nominal_plan_ns = planned_arrival_ns
            self._arrival_committed = True
            self._takeoff_time_ns = takeoff_ns

        delay_s = (takeoff_ns - time.monotonic_ns()) / 1e9
        logger.info(
            "varış planı taahhüt edildi (%s) | nominal uçuş %.0f s | "
            "yerde bekleme %.1f s",
            source, self._nominal_flight_s, max(delay_s, 0.0),
        )
        # küçük kalkış gecikmelerini normal kabul eder
        if delay_s < -LATE_TAKEOFF_TOLERANCE_S:
            logger.warning(
                "planlanan kalkış zamanı %.1f s önce geçti; hemen kalkılıyor, "
                "fark hız kontrolüyle kapatılmalı", -delay_s,
            )

    def _on_wait_takeoff_slot(self) -> None:
        # kalkış planını yeniler ve zamanı gelince arm durumuna geçer
        # yerdeki rüzgâr bilgisiyle kalkış anını yeniler
        self._resync_takeoff_slot(time.monotonic_ns())
        if time.monotonic_ns() >= self._takeoff_time_ns:
            self._transition(MissionState.ARMING)

    def _resync_takeoff_slot(self, now_ns: int) -> None:
        # kalkış anını güncel plan ve rüzgâra göre yeniden kurar
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
                "kalkış zamanı güncellendi | nominal uçuş %.0f s | yerde kalan bekleme %.1f s",
                self._nominal_flight_s, max(delay_s, 0.0),
            )

    def _on_arming(self) -> None:
        # auto modunu ve arm işlemini tamamlayıp kalkışı başlatır
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
        # kalkış irtifasına ulaşılınca tırmanış durumuna geçer
        if self._altitude_reached(self._config.takeoff_alt_msl_m):
            self._transition(MissionState.CLIMB)

    def _on_climb(self) -> None:
        # seyir irtifasına ulaşılınca seyir durumuna geçer
        if self._altitude_reached(self._config.cruise_alt_msl_m):
            self._transition(MissionState.CRUISE)

    def _on_cruise(self) -> None:
        # seyirde varışı kapıyı ve hız kontrolünü işler
        position = self._track_arrival()
        if position is None:
            return
        if self._handle_hold_gate(position):
            return
        self._coordinate_and_regulate()
        if self._at_final_terminal_entry(position):
            self._transition(MissionState.TERMINAL)

    def _at_final_terminal_entry(self, position: LatLon) -> bool:
        # terminal çemberine son rota girişini denetler
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
        # son bekleme kapısının geçiş zaman aralığını döner
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
        # son güvenli kapının geçildiğini tek yönlü kaydeder
        gate = self._hold_gate
        if gate is None or self._gate_crossed or self._loitering:
            return
        inside = geodesic_distance_m(position, self._config.target) < (
            MIN_LOITER_DISTANCE_M - GATE_CROSSING_HYSTERESIS_M
        )
        # yalnızca seçilen rota bacağındaki girişi izler
        selected_leg_inside = (
            self._active_wp_index == gate.active_wp_index and inside
        )
        if self._active_wp_index > gate.active_wp_index or selected_leg_inside:
            with self._lock:
                self._gate_crossed = True
            logger.info("SON BEKLEME KAPISI GEÇİLDİ | terminalde havada bekleme kilitlendi")

    def _eta_to_gate_s(self, position: LatLon) -> Optional[float]:
        # güncel konumdan son bekleme kapısına kalan süreyi hesaplar
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
        # seçilen bacakta varış sınırlarını ölçülen hızla yeniler
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
        # son yasal rota kapısında tek sefer bekleme uygular
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
                self._exit_gate_hold("kapı penceresi boşaldı")
            self._log_gate_problem(
                "KAPI PENCERESİ BOŞ: terminal hız yetkisi seçilen zarf ve "
                f"paylar için yetersiz ({(lower_ns - upper_ns) / 1e9:.1f} s)"
            )
            return self._loitering

        now_ns = time.monotonic_ns()
        release_ns = upper_ns
        if self._loitering:
            # guided hedefini kayıp mesaja karşı tekrarlar
            self._guided.send(gate.position, self._config.cruise_alt_msl_m)
            target_distance_m = geodesic_distance_m(position, self._config.target)
            route_deviation_m = self._route_deviation_from_nominal(position)
            if (
                target_distance_m <= GATE_TARGET_DISTANCE_ABORT_M
                or route_deviation_m >= GATE_ROUTE_DEVIATION_ABORT_M
            ):
                self._exit_gate_hold(
                    "güvenlik payı azaldı "
                    f"(hedef {target_distance_m:.0f} m, sapma {route_deviation_m:.0f} m)"
                )
                return self._loitering
            if now_ns >= release_ns:
                self._exit_gate_hold("rezervli çıkış anı geldi")
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
                "KAPI PENCERESİ KAÇIRILDI: tahmini geçiş üst sınırdan "
                f"{(predicted_crossing_ns - upper_ns) / 1e9:.1f} s sonra"
            )
            return False
        if predicted_crossing_ns >= release_ns - int(
            GATE_HOLD_TRIGGER_S * NANOSECONDS_PER_SECOND
        ):
            return False

        if not self._commander.set_mode("GUIDED"):
            self._log_gate_problem("SON BEKLEME KAPISINDA GUIDED moda geçilemedi")
            return False
        self._guided.send(gate.position, self._config.cruise_alt_msl_m)
        with self._lock:
            self._loitering = True
            self._gate_hold_used = True
        logger.info(
            "KAPI BEKLEMESİ BAŞLADI | hedefe %.0f m | planlanan çıkışa %.1f s | "
            "terminal E %.1f L %.1f s",
            geodesic_distance_m(gate.position, self._config.target),
            max((release_ns - now_ns) / 1e9, 0.0),
            gate.terminal_earliest_s,
            gate.terminal_latest_s,
        )
        return True

    def _exit_gate_hold(self, reason: str) -> None:
        # guided beklemeyi bitirip auto görevine geri döner
        if not self._commander.set_mode("AUTO"):
            logger.error("KAPI BEKLEMESİ bitirilemedi: AUTO moda dönülemedi")
            return
        with self._lock:
            self._loitering = False
            # aynı kapıda ikinci beklemeyi engeller
            self._gate_hold_used = True
        logger.info("KAPI BEKLEMESİ BİTTİ | %s", reason)

    def _log_gate_problem(self, message: str) -> None:
        # kapı sorununu belirli aralıklarla loglar
        if time.monotonic() < self._next_gate_log:
            return
        self._next_gate_log = time.monotonic() + GATE_LOG_INTERVAL_S
        logger.warning(message)

    def _model_eta_s(self, position: LatLon) -> float:
        # kalan rotanın komut hızı ve rüzgâr altındaki süresini hesaplar
        wind = self._known_wind(time.monotonic_ns()) or WindEstimate(
            east_mps=0.0, north_mps=0.0
        )
        # kalan yolu aktif nominal rota üzerinden kurar
        remaining = (
            self._config.route[self._active_wp_index:]
        )
        return route_duration_with_wind_s(
            position, remaining, self._controller.commanded_airspeed_mps, wind
        )

    def _on_terminal(self) -> None:
        # terminal yaklaşmada varış ve hız kontrolünü sürdürür
        position = self._track_arrival()
        if position is None:
            return
        # son kapıyı terminal durumunda da denetler
        if self._handle_hold_gate(position):
            return
        self._coordinate_and_regulate()

    def _timing_error_s(self) -> float:
        # pozitif değeri geç kalma olarak veren zaman hatasını hesaplar
        if self._planned_arrival_ns <= 0:
            return 0.0
        remaining_s = (self._planned_arrival_ns - time.monotonic_ns()) / 1e9
        return self._eta_s - remaining_s

    def _coordinate_and_regulate(self) -> None:
        # varış zamanına göre hava hızını düzenler
        self._regulate_speed()

    def _revise_plan(self, now_ns: int) -> None:
        # rüzgâr öğrenilince planı yalnızca ileri yönde düzeltir
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
            reason = "rüzgâr düzeltmesi (tek seferlik)"
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
        # taahhüt edilen planı yalnızca daha geç bir ana taşır
        with self._lock:
            if revised_ns <= self._nominal_plan_ns:
                return
            shift_s = (revised_ns - self._nominal_plan_ns) / 1e9
            self._nominal_plan_ns = revised_ns
            self._planned_arrival_ns = max(self._planned_arrival_ns, revised_ns)

        if shift_s >= ANCHOR_LOG_THRESHOLD_S:
            logger.info("varış planı %.1f s ileri çekildi (%s)", shift_s, reason)

    def _update_feasible_arrival(self, now_ns: int) -> None:
        # azami hava hızıyla ulaşılabilen en erken varışı günceller
        # bekleme sırasında ulaşılabilirlik hesabını dondurur
        if self._loitering:
            return

        # varıştan sonra gerçek varış anını kullanır
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
            # yerdeki araç için planlanan kalkış anını kullanır
            start_ns = max(now_ns, self._takeoff_time_ns)
            seconds = route_duration_with_wind_s(
                self._config.home, self._config.route, max_airspeed, wind
            )

        with self._lock:
            self._feasible_arrival_ns = start_ns + int(seconds * NANOSECONDS_PER_SECOND)

    def _wind_blocks(
        self, position: LatLon, remaining: Tuple[LatLon, ...]
    ) -> Tuple[Tuple[LatLon, Tuple[LatLon, ...]], ...]:
        # kalan rotayı rüzgâr tutarlılık süresine göre böler
        blocks = []
        block_start = position
        block_points: list = []
        block_duration_s = 0.0
        previous = position
        for point in remaining:
            leg_m = geodesic_distance_m(previous, point)
            leg_s = leg_m / max(self._config.nominal_cruise_speed_mps, 1e-6)
            block_points.append(point)
            block_duration_s += leg_s
            if block_duration_s >= DISTURBANCE_COHERENCE_S:
                blocks.append((block_start, tuple(block_points)))
                block_start = point
                block_points = []
                block_duration_s = 0.0
            previous = point
        if block_points:
            blocks.append((block_start, tuple(block_points)))
        return tuple(blocks)

    def _bounds_for_route_s(
        self,
        position: LatLon,
        remaining: Tuple[LatLon, ...],
        initial_airspeed_mps: float,
    ) -> Tuple[float, float]:
        # kalan rotanın rüzgâr zarfındaki varış sınırlarını hesaplar
        if not remaining:
            return 0.0, 0.0

        # her blok için ayrı rüzgâr zarfı kullanır
        blocks = self._wind_blocks(position, remaining)
        if len(blocks) > 1:
            total_slow_s = 0.0
            total_fast_s = 0.0
            # erken ve geç sınırları ayrı hız zincirleriyle hesaplar
            slow_airspeed = initial_airspeed_mps
            fast_airspeed = initial_airspeed_mps
            for block_index, (block_start, block_points) in enumerate(blocks):
                slow_s, fast_s = self._block_bounds_s(
                    block_start, block_points, slow_airspeed, fast_airspeed, block_index
                )
                total_slow_s += slow_s
                total_fast_s += fast_s
                # hız rampasını yalnızca ilk blokta uygular
                slow_airspeed = self._config.max_airspeed_mps
                fast_airspeed = self._config.min_airspeed_mps
            return total_slow_s, total_fast_s

        return self._block_bounds_s(
            position, remaining, initial_airspeed_mps, initial_airspeed_mps, 0
        )

    def _wind_candidates(self, block_index: int) -> Tuple[WindEstimate, ...]:
        # bir blok için sabit rüzgâr adaylarını üretir
        candidates = []
        wind_speed = 0.0
        while wind_speed <= DISTURBANCE_WIND_MAX_MPS + 1e-9:
            # sıfır rüzgârı tek yönle hesaplar
            directions = (0.0,) if wind_speed == 0.0 else tuple(
                float(degree)
                for degree in range(0, 360, int(DISTURBANCE_DIRECTION_STEP_DEG))
            )
            candidates.extend(
                wind_from_speed_direction(wind_speed, degree) for degree in directions
            )
            wind_speed += DISTURBANCE_WIND_SPEED_STEP_MPS
        return tuple(candidates)

    def _block_bounds_s(
        self,
        position: LatLon,
        remaining: Tuple[LatLon, ...],
        slow_initial_airspeed_mps: float,
        fast_initial_airspeed_mps: float,
        block_index: int,
    ) -> Tuple[float, float]:
        # tek blok için en erken ve en geç süreyi hesaplar
        if not remaining:
            return 0.0, 0.0

        slowest_s = 0.0
        fastest_s = math.inf
        for wind in self._wind_candidates(block_index):
            slowest_s = max(
                slowest_s,
                route_duration_with_airspeed_ramp_s(
                    position,
                    remaining,
                    slow_initial_airspeed_mps,
                    self._config.max_airspeed_mps,
                    self._config.airspeed_rate_limit_mps2,
                    wind,
                ),
            )
            fastest_s = min(
                fastest_s,
                route_duration_with_airspeed_ramp_s(
                    position,
                    remaining,
                    fast_initial_airspeed_mps,
                    self._config.min_airspeed_mps,
                    self._config.airspeed_rate_limit_mps2,
                    wind,
                ),
            )
        return slowest_s, fastest_s

    def _robust_bounds_s(self, position: LatLon) -> Tuple[float, float]:
        # mevcut konumdan kalan rotanın varış sınırlarını hesaplar
        return self._bounds_for_route_s(
            position,
            self._config.route[self._active_wp_index:],
            self._live_initial_airspeed(self._telemetry.snapshot()),
        )

    def _live_initial_airspeed(self, snapshot) -> float:
        # hız değişim modelini geçerli ölçülen hızdan başlatır
        measured_mps = getattr(snapshot, "airspeed_mps", 0.0)
        if (
            math.isfinite(measured_mps)
            and 1.0 <= measured_mps <= 1.5 * self._config.max_airspeed_mps
        ):
            return measured_mps
        return self._controller.commanded_airspeed_mps

    def _refresh_robust_bounds(self) -> None:
        # ortak pencere için varış sınırlarını yeniler
        now_ns = time.monotonic_ns()
        if (now_ns - self._robust_bounds_ns) / NANOSECONDS_PER_SECOND < ROBUST_BOUNDS_INTERVAL_S:
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
        # ortak çıpayı hesaplar ve gerekirse planı ileri kaydırır
        feasible = dict(self._peer_feasible_arrivals(now_ns))
        feasible[self._config.vehicle_id] = self._feasible_arrival_ns

        anchor_ns = compute_feasible_anchor(feasible)
        if anchor_ns is None:
            return

        target_ns = target_arrival(anchor_ns, self._config.vehicle_id)
        with self._lock:
            if self._nominal_plan_ns <= 0:
                return
            # kaymayı sabit nominal plana göre ölçer
            new_plan_ns = max(self._nominal_plan_ns, target_ns)
            previous_ns = self._planned_arrival_ns
            if new_plan_ns == previous_ns:
                return
            self._planned_arrival_ns = new_plan_ns
            shift_s = (new_plan_ns - self._nominal_plan_ns) / 1e9
            change_s = abs(new_plan_ns - previous_ns) / 1e9

        if change_s >= ANCHOR_LOG_THRESHOLD_S:
            logger.info(
                "zamanlama çıpası nominal plandan %.1f s ileride | "
                "ulaşılabilirlik veren araçlar: %s",
                shift_s, sorted(feasible),
            )

    def _terminal_reserve_override(
        self, now_ns: int
    ) -> Tuple[Optional[float], Optional[Tuple[float, float]]]:
        # terminal hız rezervi azalınca sınır hızı uygular
        if (
            self._hold_gate is None
            or not self._gate_crossed
            or self.state not in AIRBORNE_STATES
            or self._loitering
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
            # boş pencerede zaman hatasının yönünü kullanır
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
        # zamanlama hatasına göre seyir hızını düzenler
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
            f" | rezerv erken/geç {reserves[0]:+.1f}/{reserves[1]:+.1f} s"
            if reserves is not None else ""
        )
        logger.info(
            "kontrol: %s | zamanlama hatası %+.1f s | gerekli %.1f m/s | "
            "komut %.1f m/s | kalan %.0f m | WP%d | saturation=%s rate_limit=%s%s",
            command.action.value, command.timing_error_s, command.required_speed_mps,
            command.airspeed_mps, self._remaining_distance_m, self._active_wp_index,
            command.saturated, command.rate_limited, reserve_text,
        )

    def _on_arrived(self) -> None:
        # varıştan sonra rtl moduna geçmeyi dener
        if self._commander.set_mode("RTL"):
            self._transition(MissionState.RTL)

    def _on_rtl(self) -> None:
        # rtl modu doğrulanınca görevi tamamlanmış sayar
        if self._commander.flight_mode == "RTL":
            self._transition(MissionState.DONE)

    # yardımcı yöntemler

    def _update_wind(self, snapshot) -> None:
        # havadayken rüzgârı kestirir
        # alçak irtifadaki rüzgâr örneklerini atlar
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

        # filtre süresini örneğin kendi damgasından ölçer
        sample_ns = snapshot.updated_monotonic_ns
        # ilk örnekte görev döngüsü aralığını kullanır
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
        # konumun nominal rotaya en kısa uzaklığını hesaplar
        points = (self._config.home, *self._config.route)
        return min(
            cross_track_distance_m(position, start, end)
            for start, end in zip(points, points[1:])
        )

    def _track_route_deviation(self, position: LatLon, _active_index: int) -> None:
        # konumun nominal rotaya en büyük sapmasını izler
        deviation_m = self._route_deviation_from_nominal(position)
        with self._lock:
            if deviation_m <= self._max_route_deviation_m:
                return
            self._max_route_deviation_m = deviation_m

        if deviation_m > MAX_ROUTE_DEVIATION_M and not self._deviation_warned:
            self._deviation_warned = True
            logger.warning(
                "ROTA SAPMASI SINIRI AŞILDI: %.0f m (sınır %.0f m)",
                deviation_m, MAX_ROUTE_DEVIATION_M,
            )

    def _altitude_reached(self, target_alt_msl_m: float) -> bool:
        # ölçülen irtifanın hedef eşiğe ulaşıp ulaşmadığını denetler
        snapshot = self._telemetry.snapshot()
        if not snapshot.valid:
            return False
        return snapshot.altitude_msl_m >= target_alt_msl_m - ALTITUDE_REACHED_MARGIN_M

    def _track_arrival(self) -> Optional[LatLon]:
        # varış tespitini günceller ve görev durumunu ilerletir
        snapshot = self._telemetry.snapshot()
        if not snapshot.valid:
            return None

        if snapshot.age_s(time.monotonic_ns()) > TELEMETRY_TIMEOUT_S:
            logger.warning("telemetri eskimiş, varış tespiti güvenilmez")
            return None

        # varıştan sonra son eta değerini korur
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
                "HEDEFE VARILDI | en yakın geçiş %.2f m | tespit %s | "
                "max rota sapması %.0f m (sınır %.0f m)",
                min_distance_m, "interpolasyon" if interpolated else "doğrudan örnek",
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
}  # görev durumlarına karşılık gelen işleyiciler
