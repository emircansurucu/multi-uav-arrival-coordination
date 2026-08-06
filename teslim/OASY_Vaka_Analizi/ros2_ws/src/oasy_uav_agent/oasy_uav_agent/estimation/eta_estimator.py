"""rota boyunca kalan mesafeyi ve varış süresini hesaplar"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from .geodesy import LatLon, geodesic_distance_m, to_local_xy

DEFAULT_ADVANCE_RADIUS_M = 150.0  # sonraki rota noktasına geçiş yarıçapı
DEFAULT_SPEED_FILTER_TAU_S = 3.0  # ilerleme hızı filtre zaman sabiti
DEFAULT_MIN_PROGRESS_SPEED_MPS = 3.0  # eta hesabındaki en düşük ilerleme hızı


def route_length_m(home: LatLon, route: Sequence[LatLon]) -> float:
    # kalkıştan hedefe kadar toplam rota mesafesini hesaplar
    points = [home, *route]
    return sum(
        geodesic_distance_m(start, end) for start, end in zip(points, points[1:])
    )


@dataclass(frozen=True)
class EtaResult:
    active_index: int
    remaining_distance_m: float
    progress_speed_mps: float
    eta_s: float


class EtaEstimator:
    # aktif rota noktasını kalan mesafeyi ve eta değerini izler

    def __init__(
        self,
        route: Sequence[LatLon],
        home: LatLon,
        advance_radius_m: float = DEFAULT_ADVANCE_RADIUS_M,
        speed_filter_tau_s: float = DEFAULT_SPEED_FILTER_TAU_S,
        min_progress_speed_mps: float = DEFAULT_MIN_PROGRESS_SPEED_MPS,
    ) -> None:
        # rota bilgisini ve eta hesabının başlangıç durumunu hazırlar
        if len(route) < 1:
            raise ValueError("rota en az bir nokta icermeli")
        self._route = tuple(route)
        self._home = home
        self._advance_radius_m = advance_radius_m
        self._speed_filter_tau_s = speed_filter_tau_s
        self._min_progress_speed_mps = min_progress_speed_mps

        self._suffix_lengths = self._compute_suffix_lengths()
        self._active_index = 0
        self._filtered_speed_mps: Optional[float] = None
        self._last_update_ns: Optional[int] = None

    @property
    def active_index(self) -> int:
        # araçın yöneldiği güncel rota noktası indeksini döner
        return self._active_index

    def _compute_suffix_lengths(self) -> Tuple[float, ...]:
        # her rota noktasından hedefe kalan mesafeyi hesaplar
        suffix = [0.0] * len(self._route)
        for index in range(len(self._route) - 2, -1, -1):
            leg = geodesic_distance_m(self._route[index], self._route[index + 1])
            suffix[index] = suffix[index + 1] + leg
        return tuple(suffix)

    def _leg_start(self, index: int) -> LatLon:
        # verilen rota bacağının başlangıç noktasını döner
        return self._home if index == 0 else self._route[index - 1]

    def _has_passed(self, position: LatLon, index: int) -> bool:
        # rota noktasının geçilip geçilmediğini belirler
        waypoint = self._route[index]
        if geodesic_distance_m(position, waypoint) <= self._advance_radius_m:
            return True

        # köşe kesildiğinde bacak üzerindeki ilerlemeyi kullanır
        origin = self._leg_start(index)
        leg = to_local_xy(waypoint, origin)
        leg_squared = leg[0] ** 2 + leg[1] ** 2
        if leg_squared == 0.0:
            return True
        current = to_local_xy(position, origin)
        along_track = (current[0] * leg[0] + current[1] * leg[1]) / leg_squared
        return along_track > 1.0

    def _advance_active_index(self, position: LatLon) -> None:
        # aktif rota indeksini hedefe doğru ilerletir
        while self._active_index < len(self._route) - 1:
            if not self._has_passed(position, self._active_index):
                break
            self._active_index += 1

    def remaining_distance_m(self, position: LatLon) -> float:
        # güncel konumdan rota sonuna kalan toplam mesafeyi hesaplar
        to_active = geodesic_distance_m(position, self._route[self._active_index])
        return to_active + self._suffix_lengths[self._active_index]

    def _progress_speed_mps(self, position: LatLon, velocity_en: Tuple[float, float]) -> float:
        # hızın aktif rota noktası yönündeki bileşenini hesaplar
        direction = to_local_xy(self._route[self._active_index], position)
        norm = math.hypot(*direction)
        if norm == 0.0:
            return 0.0
        unit = (direction[0] / norm, direction[1] / norm)
        return velocity_en[0] * unit[0] + velocity_en[1] * unit[1]

    def _filter_speed(self, raw_speed_mps: float, now_monotonic_ns: int) -> float:
        # rota yönündeki hızı zamana bağlı alçak geçiren filtreyle süzer
        if self._filtered_speed_mps is None or self._last_update_ns is None:
            self._filtered_speed_mps = raw_speed_mps
        else:
            dt_s = (now_monotonic_ns - self._last_update_ns) / 1e9
            if dt_s > 0.0:
                alpha = 1.0 - math.exp(-dt_s / self._speed_filter_tau_s)
                self._filtered_speed_mps += alpha * (raw_speed_mps - self._filtered_speed_mps)
        self._last_update_ns = now_monotonic_ns
        return self._filtered_speed_mps

    def update(
        self,
        position: LatLon,
        velocity_en: Tuple[float, float],
        now_monotonic_ns: int,
    ) -> EtaResult:
        # konum ve hız örneğinden kalan mesafe ile eta değerini günceller
        self._advance_active_index(position)
        remaining_m = self.remaining_distance_m(position)
        filtered_speed = self._filter_speed(
            self._progress_speed_mps(position, velocity_en), now_monotonic_ns
        )
        effective_speed = max(filtered_speed, self._min_progress_speed_mps)

        return EtaResult(
            active_index=self._active_index,
            remaining_distance_m=remaining_m,
            progress_speed_mps=filtered_speed,
            eta_s=remaining_m / effective_speed,
        )
